import time
import multiprocessing
from typing import List
from collections import deque
from config import settings
from redis_manager.state_manager import RedisStateManager
from boids.algorithm import BoidsAlgorithm
from geofencing.manager import GeoFenceManager
from models.drone import DroneCommand, DroneTelemetry, Vector3


class BoidsProcessor:
    def __init__(self):
        self.redis_manager = RedisStateManager()
        self.fence_manager = GeoFenceManager()
        self.boids = BoidsAlgorithm(fence_engine=self.fence_manager.engine)
        self.running = False
        self.process = None

        self._max_stale_ms = settings.max_stale_data_ms
        self._state_buffer = deque(maxlen=5)
        self._last_process_time = 0
        self._stats_window = deque(maxlen=100)
        self._violation_count = 0

    def start(self):
        if self.running:
            return

        self.running = True
        self.process = multiprocessing.Process(target=self._run, daemon=True)
        self.process.start()
        print(f"[Boids] Process started, interval: {settings.boids_process_interval}s")

    def stop(self):
        self.running = False
        if self.process:
            self.process.terminate()
            self.process.join()
            print("[Boids] Process stopped")

    def _run(self):
        print("[Boids] Processing loop started")
        while self.running:
            try:
                start_time = time.time()

                self.redis_manager.cleanup_offline_drones()

                min_timestamp = int(time.time() * 1000) - self._max_stale_ms
                drones = self.redis_manager.get_all_drones(min_timestamp=min_timestamp)

                if len(drones) > 0:
                    self._state_buffer.append({
                        "timestamp": int(time.time() * 1000),
                        "count": len(drones),
                        "drones": drones,
                    })

                    commands = self._process_consistent_state(drones)

                    if commands:
                        self.redis_manager.store_command_batch(commands)

                elapsed = time.time() - start_time
                self._stats_window.append(elapsed)

                if len(self._stats_window) >= 100 and len(self._stats_window) % 50 == 0:
                    avg_time = sum(self._stats_window) / len(self._stats_window)
                    max_time = max(self._stats_window)
                    pool_stats = self.redis_manager.get_pool_stats()
                    active_fences = len(self.fence_manager.get_all_fences())
                    fence_active_count = sum(1 for c in commands if c.fence_active) if commands else 0
                    print(
                        f"[Boids] Stats: avg={avg_time*1000:.1f}ms, max={max_time*1000:.1f}ms, "
                        f"drones={len(drones)}, fences={active_fences}, evading={fence_active_count}, "
                        f"violations={self._violation_count}, "
                        f"pool={pool_stats['current_connections']}/{pool_stats['max_connections']}"
                    )

                sleep_time = max(0, settings.boids_process_interval - elapsed)
                time.sleep(sleep_time)

            except Exception as e:
                print(f"[Boids] Processing error: {e}")
                time.sleep(settings.boids_process_interval)

    def _process_consistent_state(self, drones: List[DroneTelemetry]) -> List[DroneCommand]:
        if len(drones) < 2:
            return self._process_single_drone(drones)

        min_ts = min(d.timestamp for d in drones)
        max_ts = max(d.timestamp for d in drones)
        ts_gap = max_ts - min_ts

        if ts_gap > self._max_stale_ms:
            print(f"[Boids] Warning: timestamp gap {ts_gap}ms exceeds threshold {self._max_stale_ms}ms")

            timestamp_threshold = int(time.time() * 1000) - self._max_stale_ms
            valid_drones = [d for d in drones if d.timestamp >= timestamp_threshold]

            if len(valid_drones) < len(drones) * 0.8:
                print(f"[Boids] Warning: only {len(valid_drones)}/{len(drones)} drones have fresh data, using all")
            else:
                drones = valid_drones

        commands = self.boids.compute_all_commands(drones)

        for i, cmd in enumerate(commands):
            if i < len(drones):
                cmd.timestamp = max(cmd.timestamp, drones[i].timestamp)

            if cmd.fence_active:
                fence_result = self.boids.get_fence_result(cmd.drone_id)
                if fence_result:
                    for warning in fence_result.warnings:
                        self.fence_manager.log_violation(warning.model_dump(mode="json"))
                        self._violation_count += 1

        return commands

    def _process_single_drone(self, drones: List[DroneTelemetry]) -> List[DroneCommand]:
        commands = []
        for drone in drones:
            fence_result = self.fence_manager.evaluate_drone(drone)

            fence_force = fence_result.repulsion_force
            fence_magnitude = (fence_force.x ** 2 + fence_force.y ** 2 + fence_force.z ** 2) ** 0.5

            target_velocity = Vector3(
                x=drone.velocity.x + fence_force.x,
                y=drone.velocity.y + fence_force.y,
                z=drone.velocity.z + fence_force.z,
            )

            lat_offset = target_velocity.y * 0.00001
            lon_offset = target_velocity.x * 0.00001

            target_position = Vector3(
                x=drone.longitude + lon_offset,
                y=drone.latitude + lat_offset,
                z=drone.altitude + target_velocity.z,
            )

            cmd = DroneCommand(
                drone_id=drone.drone_id,
                target_velocity=target_velocity,
                target_position=target_position,
                separation_force=0.0,
                alignment_force=0.0,
                cohesion_force=0.0,
                fence_force=fence_magnitude,
                fence_active=fence_result.is_active,
                violating_fences=fence_result.violating_fences,
            )
            commands.append(cmd)

            if fence_result.is_active:
                for warning in fence_result.warnings:
                    self.fence_manager.log_violation(warning.model_dump(mode="json"))
                    self._violation_count += 1

        return commands
