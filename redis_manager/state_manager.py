import json
import time
import redis
import geohash2
from typing import Dict, List, Optional
from config import settings
from models.drone import DroneTelemetry, DroneCommand


class RedisStateManager:
    def __init__(self):
        self.redis = redis.Redis(
            host=settings.redis_host,
            port=settings.redis_port,
            db=settings.redis_db,
            password=settings.redis_password,
            decode_responses=True
        )
        self.telemetry_key_prefix = "drone:telemetry:"
        self.geohash_key = "drone:geohashes"
        self.command_key_prefix = "drone:command:"
        self.telemetry_channel = "drone:telemetry:updates"
        self.command_channel_prefix = "drone:command:"
        self.mission_key = "swarm:mission"
        self.online_drones_key = "drone:online"

    def _get_telemetry_key(self, drone_id: str) -> str:
        return f"{self.telemetry_key_prefix}{drone_id}"

    def _get_command_key(self, drone_id: str) -> str:
        return f"{self.command_key_prefix}{drone_id}"

    def _get_command_channel(self, drone_id: str) -> str:
        return f"{self.command_channel_prefix}{drone_id}"

    def _compute_geohash(self, lat: float, lon: float) -> str:
        return geohash2.encode(lat, lon, precision=settings.geohash_precision)

    def update_telemetry(self, telemetry: DroneTelemetry) -> bool:
        try:
            telemetry.geohash = self._compute_geohash(
                telemetry.latitude, telemetry.longitude
            )
            telemetry.timestamp = int(time.time() * 1000)

            key = self._get_telemetry_key(telemetry.drone_id)
            data = telemetry.model_dump_json()

            pipe = self.redis.pipeline()
            pipe.set(key, data, ex=settings.telemetry_ttl)
            pipe.zadd(self.geohash_key, {telemetry.drone_id: 0})
            pipe.hset(
                f"{key}:geohash",
                mapping={
                    "geohash": telemetry.geohash,
                    "lat": telemetry.latitude,
                    "lon": telemetry.longitude,
                    "alt": telemetry.altitude,
                },
            )
            pipe.sadd(self.online_drones_key, telemetry.drone_id)
            pipe.execute()

            self.redis.publish(
                self.telemetry_channel,
                json.dumps(
                    {
                        "drone_id": telemetry.drone_id,
                        "geohash": telemetry.geohash,
                        "timestamp": telemetry.timestamp,
                    }
                ),
            )

            return True
        except Exception as e:
            print(f"[Redis] Update telemetry error: {e}")
            return False

    def get_telemetry(self, drone_id: str) -> Optional[DroneTelemetry]:
        try:
            key = self._get_telemetry_key(drone_id)
            data = self.redis.get(key)
            if data:
                return DroneTelemetry.model_validate_json(data)
            return None
        except Exception as e:
            print(f"[Redis] Get telemetry error: {e}")
            return None

    def get_all_drones(self) -> List[DroneTelemetry]:
        try:
            drone_ids = self.redis.smembers(self.online_drones_key)
            drones = []
            for drone_id in drone_ids:
                telemetry = self.get_telemetry(drone_id)
                if telemetry:
                    drones.append(telemetry)
                else:
                    self.redis.srem(self.online_drones_key, drone_id)
            return drones
        except Exception as e:
            print(f"[Redis] Get all drones error: {e}")
            return []

    def get_drones_by_geohash(self, geohash_prefix: str) -> List[DroneTelemetry]:
        try:
            all_drones = self.get_all_drones()
            return [
                d
                for d in all_drones
                if d.geohash and d.geohash.startswith(geohash_prefix)
            ]
        except Exception as e:
            print(f"[Redis] Get drones by geohash error: {e}")
            return []

    def get_neighbors(self, drone_id: str, radius_km: float = 1.0) -> List[DroneTelemetry]:
        try:
            telemetry = self.get_telemetry(drone_id)
            if not telemetry:
                return []

            neighbors = []
            all_drones = self.get_all_drones()

            for other in all_drones:
                if other.drone_id == drone_id:
                    continue

                distance = self._haversine_distance(
                    telemetry.latitude,
                    telemetry.longitude,
                    other.latitude,
                    other.longitude,
                )

                if distance <= radius_km:
                    neighbors.append(other)

            return neighbors
        except Exception as e:
            print(f"[Redis] Get neighbors error: {e}")
            return []

    def _haversine_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        import math

        R = 6371.0

        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    def store_command(self, command: DroneCommand) -> bool:
        try:
            command.timestamp = int(time.time() * 1000)
            key = self._get_command_key(command.drone_id)
            data = command.model_dump_json()

            self.redis.set(key, data, ex=settings.command_ttl)
            self.redis.publish(self._get_command_channel(command.drone_id), data)

            return True
        except Exception as e:
            print(f"[Redis] Store command error: {e}")
            return False

    def get_command(self, drone_id: str) -> Optional[DroneCommand]:
        try:
            key = self._get_command_key(drone_id)
            data = self.redis.get(key)
            if data:
                return DroneCommand.model_validate_json(data)
            return None
        except Exception as e:
            print(f"[Redis] Get command error: {e}")
            return None

    def subscribe_telemetry(self, callback):
        pubsub = self.redis.pubsub()
        pubsub.subscribe(self.telemetry_channel)

        for message in pubsub.listen():
            if message["type"] == "message":
                try:
                    data = json.loads(message["data"])
                    callback(data)
                except Exception as e:
                    print(f"[Redis] Telemetry subscribe error: {e}")

    def subscribe_commands(self, drone_id: str, callback):
        pubsub = self.redis.pubsub()
        pubsub.subscribe(self._get_command_channel(drone_id))

        for message in pubsub.listen():
            if message["type"] == "message":
                try:
                    data = json.loads(message["data"])
                    callback(data)
                except Exception as e:
                    print(f"[Redis] Command subscribe error: {e}")

    def set_mission(self, mission_data: Dict) -> bool:
        try:
            self.redis.set(self.mission_key, json.dumps(mission_data))
            self.redis.publish("swarm:mission:updates", json.dumps(mission_data))
            return True
        except Exception as e:
            print(f"[Redis] Set mission error: {e}")
            return False

    def get_mission(self) -> Optional[Dict]:
        try:
            data = self.redis.get(self.mission_key)
            if data:
                return json.loads(data)
            return None
        except Exception as e:
            print(f"[Redis] Get mission error: {e}")
            return None

    def get_online_count(self) -> int:
        try:
            return self.redis.scard(self.online_drones_key)
        except Exception:
            return 0

    def cleanup_offline_drones(self):
        try:
            drone_ids = self.redis.smembers(self.online_drones_key)
            for drone_id in drone_ids:
                if not self.redis.exists(self._get_telemetry_key(drone_id)):
                    self.redis.srem(self.online_drones_key, drone_id)
        except Exception as e:
            print(f"[Redis] Cleanup offline drones error: {e}")
