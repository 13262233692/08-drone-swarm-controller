import sys
import os
import time
import threading
import random
import math
from typing import List, Dict
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from models.drone import DroneTelemetry, Vector3, EulerAngles
from redis_manager.state_manager import RedisStateManager, RedisConnectionPool
from grpc_client.client import GrpcClient


class StressTestConfig:
    DRONE_COUNT = 200
    REPORTS_PER_SECOND = 50
    TEST_DURATION_SECONDS = 60
    BASE_LAT = 39.9042
    BASE_LON = 116.4074
    SPREAD = 0.002


class DroneSimulator:
    def __init__(self, drone_id: str, redis_mgr: RedisStateManager):
        self.drone_id = drone_id
        self.redis_mgr = redis_mgr
        self.stop_event = threading.Event()
        self.report_count = 0
        self.error_count = 0
        self.last_report_time = 0

        angle = random.uniform(0, 2 * math.pi)
        radius = random.uniform(0, StressTestConfig.SPREAD)
        self.lat = StressTestConfig.BASE_LAT + math.sin(angle) * radius
        self.lon = StressTestConfig.BASE_LON + math.cos(angle) * radius
        self.alt = random.uniform(40, 60)
        self.vx = random.uniform(-2, 2)
        self.vy = random.uniform(-2, 2)
        self.vz = random.uniform(-0.5, 0.5)

        self.report_times = []

    def stop(self):
        self.stop_event.set()

    def run(self, interval: float):
        next_report = time.time()
        while not self.stop_event.is_set():
            try:
                now = time.time()
                if now >= next_report:
                    self._update_position(interval)
                    self._send_telemetry()
                    next_report += interval
                else:
                    time.sleep(max(0, next_report - now))
            except Exception as e:
                self.error_count += 1
                time.sleep(interval)

    def _update_position(self, dt: float):
        self.lat += self.vy * dt * 0.00001
        self.lon += self.vx * dt * 0.00001
        self.alt += self.vz * dt

        self.vx += random.uniform(-0.1, 0.1)
        self.vy += random.uniform(-0.1, 0.1)
        self.vz += random.uniform(-0.05, 0.05)

        max_speed = 10.0
        speed = math.sqrt(self.vx**2 + self.vy**2 + self.vz**2)
        if speed > max_speed:
            scale = max_speed / speed
            self.vx *= scale
            self.vy *= scale
            self.vz *= scale

    def _send_telemetry(self):
        start = time.time()
        telemetry = DroneTelemetry(
            drone_id=self.drone_id,
            latitude=self.lat,
            longitude=self.lon,
            altitude=self.alt,
            velocity=Vector3(x=self.vx, y=self.vy, z=self.vz),
            attitude=EulerAngles(
                roll=random.uniform(-5, 5),
                pitch=random.uniform(-5, 5),
                yaw=math.degrees(math.atan2(self.vy, self.vx)),
            ),
            battery_level=random.uniform(60, 100),
            timestamp=int(time.time() * 1000),
        )

        success = self.redis_mgr.update_telemetry(telemetry)
        elapsed = (time.time() - start) * 1000

        if success:
            self.report_count += 1
            self.report_times.append(elapsed)
            if len(self.report_times) > 1000:
                self.report_times = self.report_times[-1000:]
        else:
            self.error_count += 1


class StressTest:
    def __init__(self):
        self.redis_mgr = RedisStateManager()
        self.pool = RedisConnectionPool()
        self.drones: List[DroneSimulator] = []
        self.threads: List[threading.Thread] = []
        self.results: Dict = defaultdict(int)
        self.monitor_stop = threading.Event()
        self.consistency_errors = 0
        self.dirty_reads = 0

    def setup(self):
        print(f"[StressTest] Initializing {StressTestConfig.DRONE_COUNT} drone simulators...")
        for i in range(StressTestConfig.DRONE_COUNT):
            drone = DroneSimulator(f"stress-drone-{i:03d}", self.redis_mgr)
            self.drones.append(drone)
        print(f"[StressTest] Setup complete")

    def run(self):
        interval = 1.0 / StressTestConfig.REPORTS_PER_SECOND

        print()
        print("=" * 70)
        print(f"  压测开始")
        print(f"  无人机数量: {StressTestConfig.DRONE_COUNT}")
        print(f"  上报频率: {StressTestConfig.REPORTS_PER_SECOND}/秒/架")
        print(f"  总QPS: {StressTestConfig.DRONE_COUNT * StressTestConfig.REPORTS_PER_SECOND}")
        print(f"  持续时间: {StressTestConfig.TEST_DURATION_SECONDS}秒")
        print("=" * 70)
        print()

        monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        monitor_thread.start()

        consistency_thread = threading.Thread(target=self._consistency_check_loop, daemon=True)
        consistency_thread.start()

        start_time = time.time()

        for drone in self.drones:
            t = threading.Thread(target=drone.run, args=(interval,), daemon=True)
            t.start()
            self.threads.append(t)

        test_end_time = start_time + StressTestConfig.TEST_DURATION_SECONDS
        while time.time() < test_end_time:
            time.sleep(1)

        print()
        print("[StressTest] Stopping all drones...")
        for drone in self.drones:
            drone.stop()

        for t in self.threads:
            t.join(timeout=5)

        self.monitor_stop.set()
        time.sleep(1)

        self._print_results(start_time)

    def _monitor_loop(self):
        last_total = 0
        last_time = time.time()

        while not self.monitor_stop.is_set():
            try:
                time.sleep(1)

                total_reports = sum(d.report_count for d in self.drones)
                total_errors = sum(d.error_count for d in self.drones)

                now = time.time()
                elapsed = now - last_time
                qps = (total_reports - last_total) / elapsed if elapsed > 0 else 0

                pool_stats = self.pool.get_pool_stats()
                online_count = self.redis_mgr.get_online_count()

                all_times = []
                for d in self.drones:
                    all_times.extend(d.report_times)

                avg_latency = sum(all_times) / len(all_times) if all_times else 0
                max_latency = max(all_times) if all_times else 0

                print(
                    f"[Monitor] QPS={qps:.0f} | Total={total_reports} | Errors={total_errors} | "
                    f"Online={online_count} | "
                    f"Pool={pool_stats['current_connections']}/{pool_stats['max_connections']} | "
                    f"Latency: avg={avg_latency:.1f}ms max={max_latency:.1f}ms | "
                    f"DirtyReads={self.dirty_reads}"
                )

                last_total = total_reports
                last_time = now

                if pool_stats['current_connections'] >= pool_stats['max_connections'] * 0.9:
                    print(f"[WARNING] Redis connection pool nearly full! {pool_stats}")

            except Exception as e:
                print(f"[Monitor] Error: {e}")

    def _consistency_check_loop(self):
        while not self.monitor_stop.is_set():
            try:
                time.sleep(0.5)

                min_ts = int(time.time() * 1000) - 2000
                drones = self.redis_mgr.get_all_drones(min_timestamp=min_ts)

                if len(drones) > 1:
                    timestamps = [d.timestamp for d in drones]
                    ts_gap = max(timestamps) - min(timestamps)

                    if ts_gap > 2000:
                        self.dirty_reads += 1
                        print(
                            f"[Consistency] WARNING: Timestamp gap {ts_gap}ms exceeds 2000ms, "
                            f"drones={len(drones)}"
                        )

                    drone_ids = set(d.drone_id for d in drones)
                    expected_ids = set(f"stress-drone-{i:03d}" for i in range(StressTestConfig.DRONE_COUNT))
                    missing = expected_ids - drone_ids

                    if len(missing) > StressTestConfig.DRONE_COUNT * 0.1:
                        self.consistency_errors += 1
                        print(
                            f"[Consistency] WARNING: {len(missing)} drones missing from state, "
                            f"expected {StressTestConfig.DRONE_COUNT}, got {len(drones)}"
                        )

            except Exception as e:
                print(f"[Consistency] Check error: {e}")

    def _print_results(self, start_time):
        total_duration = time.time() - start_time
        total_reports = sum(d.report_count for d in self.drones)
        total_errors = sum(d.error_count for d in self.drones)

        all_times = []
        for d in self.drones:
            all_times.extend(d.report_times)

        all_times.sort()
        avg_latency = sum(all_times) / len(all_times) if all_times else 0
        p50 = all_times[int(len(all_times) * 0.5)] if all_times else 0
        p95 = all_times[int(len(all_times) * 0.95)] if all_times else 0
        p99 = all_times[int(len(all_times) * 0.99)] if all_times else 0
        max_latency = max(all_times) if all_times else 0

        print()
        print("=" * 70)
        print("  压测结果")
        print("=" * 70)
        print(f"  总持续时间: {total_duration:.1f} 秒")
        print(f"  总上报次数: {total_reports:,}")
        print(f"  平均QPS: {total_reports / total_duration:.0f}")
        print(f"  总错误数: {total_errors:,}")
        print(f"  错误率: {total_errors / (total_reports + total_errors) * 100:.4f}%")
        print()
        print("  延迟统计:")
        print(f"    平均: {avg_latency:.2f} ms")
        print(f"    P50:  {p50:.2f} ms")
        print(f"    P95:  {p95:.2f} ms")
        print(f"    P99:  {p99:.2f} ms")
        print(f"    最大: {max_latency:.2f} ms")
        print()
        print("  一致性统计:")
        print(f"    脏读检测: {self.dirty_reads} 次")
        print(f"    数据缺失: {self.consistency_errors} 次")
        print()
        print("  最终连接池状态:")
        pool_stats = self.pool.get_pool_stats()
        print(f"    当前连接: {pool_stats['current_connections']}")
        print(f"    最大连接: {pool_stats['max_connections']}")
        print(f"    可用连接: {pool_stats['available_connections']}")
        print("=" * 70)
        print()

        if total_errors == 0 and pool_stats['current_connections'] < pool_stats['max_connections']:
            print("✅ 压测通过: 无错误，连接池正常")
        else:
            print("⚠️  压测需要关注: 存在错误或连接池问题")


def main():
    print()
    print("╔══════════════════════════════════════════════════════════════════╗")
    print("║          无人机编队控制系统 - 高压并发压测工具                  ║")
    print("╚══════════════════════════════════════════════════════════════════╝")
    print()

    test = StressTest()
    test.setup()

    try:
        test.run()
    except KeyboardInterrupt:
        print("\n[StressTest] Interrupted by user")
        for drone in test.drones:
            drone.stop()


if __name__ == "__main__":
    main()
