import json
import threading
from typing import List, Optional, Dict
from datetime import datetime
from redis_manager.state_manager import RedisStateManager, RedisConnectionPool
from models.drone import (
    GeoFence,
    DroneTelemetry,
    FenceRepulsionResult,
)
from geofencing.engine import GeoFencingEngine


class GeoFenceManager:
    def __init__(self):
        self.redis_mgr = RedisStateManager()
        self.engine = GeoFencingEngine()
        self._lock = threading.Lock()
        self._fences_key = "geofence:all"
        self._violations_key = "geofence:violations"
        self._load_fences()

    def _load_fences(self):
        try:
            with self._lock:
                data = self.redis_mgr._redis.get(self._fences_key)
                if data:
                    fence_list = json.loads(data)
                    fences = []
                    for fd in fence_list:
                        try:
                            fence = GeoFence.model_validate(fd)
                            if self._is_fence_active(fence):
                                fences.append(fence)
                        except Exception:
                            continue
                    self.engine.set_fences(fences)
                    print(f"[GeoFence] Loaded {len(fences)} active fences from Redis")
        except Exception as e:
            print(f"[GeoFence] Load fences error: {e}")

    def _save_fences(self):
        try:
            fences = self.engine.get_fences()
            data = json.dumps([f.model_dump(mode="json") for f in fences])
            self.redis_mgr._redis.set(self._fences_key, data)
            self.redis_mgr._redis.publish(
                "geofence:updates",
                json.dumps({"action": "update", "count": len(fences)}),
            )
        except Exception as e:
            print(f"[GeoFence] Save fences error: {e}")

    def _is_fence_active(self, fence: GeoFence) -> bool:
        if not fence.enabled:
            return False
        if fence.expire_at and fence.expire_at < datetime.now():
            return False
        return True

    def add_fence(self, fence: GeoFence) -> bool:
        try:
            with self._lock:
                self.engine.add_fence(fence)
                self._save_fences()
                print(f"[GeoFence] Added fence: {fence.fence_id} ({fence.name})")
                return True
        except Exception as e:
            print(f"[GeoFence] Add fence error: {e}")
            return False

    def remove_fence(self, fence_id: str) -> bool:
        try:
            with self._lock:
                self.engine.remove_fence(fence_id)
                self._save_fences()
                print(f"[GeoFence] Removed fence: {fence_id}")
                return True
        except Exception as e:
            print(f"[GeoFence] Remove fence error: {e}")
            return False

    def get_all_fences(self) -> List[GeoFence]:
        with self._lock:
            return self.engine.get_fences()

    def get_fence(self, fence_id: str) -> Optional[GeoFence]:
        with self._lock:
            for fence in self.engine.get_fences():
                if fence.fence_id == fence_id:
                    return fence
        return None

    def update_fence(self, fence: GeoFence) -> bool:
        return self.add_fence(fence)

    def clear_all_fences(self) -> bool:
        try:
            with self._lock:
                self.engine.set_fences([])
                self._save_fences()
                print("[GeoFence] Cleared all fences")
                return True
        except Exception as e:
            print(f"[GeoFence] Clear fences error: {e}")
            return False

    def reload_from_redis(self):
        self._load_fences()

    def evaluate_drone(self, drone: DroneTelemetry) -> FenceRepulsionResult:
        with self._lock:
            return self.engine.evaluate_drone(drone)

    def evaluate_batch(self, drones: List[DroneTelemetry]) -> Dict[str, FenceRepulsionResult]:
        results = {}
        with self._lock:
            for drone in drones:
                results[drone.drone_id] = self.engine.evaluate_drone(drone)
        return results

    def get_violations(self, limit: int = 100) -> List[Dict]:
        try:
            raw = self.redis_mgr._redis.lrange(self._violations_key, 0, limit - 1)
            return [json.loads(v) for v in raw if v]
        except Exception as e:
            print(f"[GeoFence] Get violations error: {e}")
            return []

    def log_violation(self, violation_data: Dict):
        try:
            self.redis_mgr._redis.lpush(
                self._violations_key, json.dumps(violation_data)
            )
            self.redis_mgr._redis.ltrim(self._violations_key, 0, 999)
        except Exception as e:
            print(f"[GeoFence] Log violation error: {e}")
