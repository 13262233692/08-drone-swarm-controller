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
from grpc_client.client import GrpcClient
from redis_manager.state_manager import RedisStateManager, RedisConnectionPool


class GrpcStressConfig:
    DRONE_COUNT = 200
    REPORTS_PER_SECOND = 50
    TEST_DURATION_SECONDS = 60
    BASE_LAT = 39.9042
    BASE_LON = 116.4074
    SPREAD = 0.002
    USE_STREAM = True


class GrpcDroneSimulator:
    def __init__(self, drone_id: str):
        self.drone_id = drone_id
        self.client = GrpcClient()
        self.stop_event = threading.Event()
        self.report_count = 0
        self.error_count = 0
        self.command_count = 0

        angle = random.uniform(0, 2 * math.pi)
        radius = random.uniform(0, GrpcStressConfig.SPREAD)
        self.lat = GrpcStressConfig.BASE_LAT + math.sin(angle) * radius
        self.lon = GrpcStressConfig.BASE_LON + math.cos(angle) * radius
        self.alt = random.uniform(40, 60)
        self.vx = random.uniform(-2, 2)
        self.vy = random.uniform(-2, 2)
        self.vz = random.uniform(-0.5, 0.5)

        self.report_times = []
        self.command_latencies = []

    def stop(self):
        self.stop_event.set()

    def connect(self):
        self.client.connect()

    def close(self):
        try:
            self.client.close()
        except Exception:
            pass

    def run(self, interval: float):
        if not GrpcStressConfig.USE_STREAM:
            self._run_unary(interval)
        else:
            self._run_stream(interval)

    def _run_unary(self, interval: float):
        next_report = time.time()
        while not self.stop_event.is_set():
            try:
                now = time.time()
                if now >= next_report:
                    self._update_position(interval)
                    latency = self._send_telemetry_unary()
                    if latency >= 0:
                        self.report_times.append(latency)
                        self.report_count += 1
                    else:
                        self.error_count += 1

                    cmd = self.client.get_command(self.drone_id)
                    if cmd:
                        self.command_count += 1
                        self.command_latencies.append(int(time.time() * 1000) - cmd.timestamp)

                    next_report += interval
                else:
                    time.sleep(max(0, next_report - now))
            except Exception as e:
                self.error_count += 1
                time.sleep(interval)

    def _run_stream(self, interval: float):
        def telemetry_generator():
            while not self.stop_event.is_set():
                self._update_position(interval)
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
                )
                yield telemetry
                self.report_count += 1
                time.sleep(interval)

        try:
            for cmd in self.client.stream_telemetry(telemetry_generator()):
                self.command_count += 1
                self.command_latencies.append(int(time.time() * 1000) - cmd.timestamp)
                if self.stop_event.is_set():
                    break
        except Exception as e:
            self.error_count += 1

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

    def _send_telemetry_unary(self) -> float:
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
        )

        success = self.client.send_telemetry(telemetry)
        elapsed = (time.time() - start) * 1000

        if success:
            if len(self.report_times) > 1000:
                self.report_times = self.report_times[-1000:]
            return elapsed
        return -1


class GrpcStressTest:
    def __init__(self):
        self.redis_mgr = RedisStateManager()
        self.pool = RedisConnectionPool()
        self.drones: List[GrpcDroneSimulator] = []
        self.threads: List[threading.Thread] = []
        self.monitor_stop = threading.Event()
        self.results: Dict = defaultdict(int)

    def setup(self):
        print(f"[gRPC StressTest] Initializing {GrpcStressConfig.DRONE_COUNT} gRPC drone simulators...")
        for i in range(GrpcStressConfig.DRONE_COUNT):
            drone = GrpcDroneSimulator(f"grpc-drone-{i:03d}")
            self.drones.append(drone)
        print(f"[gRPC StressTest] Connecting gRPC clients...")
        for drone in self.drones:
            drone.connect()
        print(f"[gRPC StressTest] Setup complete")

    def run(self):
        interval = 1.0 / GrpcStressConfig.REPORTS_PER_SECOND

        print()
        print("=" * 70)
        print(f"  gRPC 压测开始")
        print(f"  模式: {'Stream' if GrpcStressConfig.USE_STREAM else 'Unary'}")
        print(f"  无人机数量: {GrpcStressConfig.DRONE_COUNT}")
        print(f"  上报频率: {GrpcStressConfig.REPORTS_PER_SECOND}/秒/架")
        print(f"  总QPS: {GrpcStressConfig.DRONE_COUNT * GrpcStressConfig.REPORTS_PER_SECOND}")
        print(f"  持续时间: {GrpcStressConfig.TEST_DURATION_SECONDS}秒")
        print("=" * 70)
        print()

        monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        monitor_thread.start()

        start_time = time.time()

        for drone in self.drones:
            t = threading.Thread(target=drone.run, args=(interval,), daemon=True)
            t.start()
            self.threads.append(t)

        test_end_time = start_time + GrpcStressConfig.TEST_DURATION_SECONDS
        while time.time() < test_end_time:
            time.sleep(1)

        print()
        print("[gRPC StressTest] Stopping all drones...")
        for drone in self.drones:
            drone.stop()

        for t in self.threads:
            t.join(timeout=5)

        for drone in self.drones:
            drone.close()

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
                total_commands = sum(d.command_count for d in self.drones)

                now = time.time()
                elapsed = now - last_time
                qps = (total_reports - last_total) / elapsed if elapsed > 0 else 0

                pool_stats = self.pool.get_pool_stats()
                online_count = self.redis_mgr.get_online_count()

                all_times = []
                for d in self.drones:
                    all_times.extend(d.report_times)

                all_cmd_latencies = []
                for d in self.drones:
                    all_cmd_latencies.extend(d.command_latencies)

                avg_latency = sum(all_times) / len(all_times) if all_times else 0
                max_latency = max(all_times) if all_times else 0
                avg_cmd_latency = sum(all_cmd_latencies) / len(all_cmd_latencies) if all_cmd_latencies else 0

                print(
                    f"[Monitor] QPS={qps:.0f} | Total={total_reports} | Cmds={total_commands} | "
                    f"Errors={total_errors} | Online={online_count} | "
                    f"Pool={pool_stats['current_connections']}/{pool_stats['max_connections']} | "
                    f"RPC Lat: avg={avg_latency:.1f}ms max={max_latency:.1f}ms | "
                    f"Cmd Lat: avg={avg_cmd_latency:.0f}ms"
                )

                last_total = total_reports
                last_time = now

                if pool_stats['current_connections'] >= pool_stats['max_connections'] * 0.9:
                    print(f"[WARNING] Redis connection pool nearly full! {pool_stats}")

            except Exception as e:
                print(f"[Monitor] Error: {e}")

    def _print_results(self, start_time):
        total_duration = time.time() - start_time
        total_reports = sum(d.report_count for d in self.drones)
        total_errors = sum(d.error_count for d in self.drones)
        total_commands = sum(d.command_count for d in self.drones)

        all_times = []
        for d in self.drones:
            all_times.extend(d.report_times)

        all_cmd_latencies = []
        for d in self.drones:
            all_cmd_latencies.extend(d.command_latencies)

        all_times.sort()
        all_cmd_latencies.sort()

        avg_latency = sum(all_times) / len(all_times) if all_times else 0
        p50 = all_times[int(len(all_times) * 0.5)] if all_times else 0
        p95 = all_times[int(len(all_times) * 0.95)] if all_times else 0
        p99 = all_times[int(len(all_times) * 0.99)] if all_times else 0
        max_latency = max(all_times) if all_times else 0

        avg_cmd_latency = sum(all_cmd_latencies) / len(all_cmd_latencies) if all_cmd_latencies else 0
        cmd_p99 = all_cmd_latencies[int(len(all_cmd_latencies) * 0.99)] if all_cmd_latencies else 0

        print()
        print("=" * 70)
        print("  gRPC 压测结果")
        print("=" * 70)
        print(f"  总持续时间: {total_duration:.1f} 秒")
        print(f"  总上报次数: {total_reports:,}")
        print(f"  平均QPS: {total_reports / total_duration:.0f}")
        print(f"  接收指令数: {total_commands:,}")
        print(f"  总错误数: {total_errors:,}")
        print(f"  错误率: {total_errors / max(total_reports + total_errors, 1) * 100:.4f}%")
        print()
        print("  RPC 延迟统计:")
        print(f"    平均: {avg_latency:.2f} ms")
        print(f"    P50:  {p50:.2f} ms")
        print(f"    P95:  {p95:.2f} ms")
        print(f"    P99:  {p99:.2f} ms")
        print(f"    最大: {max_latency:.2f} ms")
        print()
        print("  指令延迟统计 (下发→接收):")
        print(f"    平均: {avg_cmd_latency:.0f} ms")
        print(f"    P99:  {cmd_p99:.0f} ms")
        print()
        print("  最终连接池状态:")
        pool_stats = self.pool.get_pool_stats()
        print(f"    当前连接: {pool_stats['current_connections']}")
        print(f"    最大连接: {pool_stats['max_connections']}")
        print(f"    可用连接: {pool_stats['available_connections']}")
        print("=" * 70)
        print()

        if total_errors == 0 and pool_stats['current_connections'] < pool_stats['max_connections']:
            print("✅ gRPC 压测通过: 无错误，连接池正常")
        else:
            print("⚠️  gRPC 压测需要关注: 存在错误或连接池问题")


def main():
    print()
    print("╔══════════════════════════════════════════════════════════════════╗")
    print("║        无人机编队控制系统 - gRPC 高压并发压测工具              ║")
    print("╚══════════════════════════════════════════════════════════════════╝")
    print()

    test = GrpcStressTest()

    try:
        test.setup()
        test.run()
    except KeyboardInterrupt:
        print("\n[gRPC StressTest] Interrupted by user")
        for drone in test.drones:
            drone.stop()
            drone.close()


if __name__ == "__main__":
    main()
