import sys
import os
import time
import random
import math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from models.drone import DroneTelemetry, Vector3, EulerAngles
from boids.algorithm import BoidsAlgorithm


def generate_swarm_drones(count: int, center_lat: float, center_lon: float, spread: float = 0.001):
    drones = []
    for i in range(count):
        angle = (2 * math.pi * i) / count
        radius = spread * (0.5 + random.random() * 0.5)
        lat_offset = math.sin(angle) * radius
        lon_offset = math.cos(angle) * radius

        drone = DroneTelemetry(
            drone_id=f"drone-{i:03d}",
            latitude=center_lat + lat_offset,
            longitude=center_lon + lon_offset,
            altitude=50.0 + random.uniform(-10, 10),
            attitude=EulerAngles(
                roll=random.uniform(-5, 5),
                pitch=random.uniform(-5, 5),
                yaw=random.uniform(0, 360),
            ),
            battery_level=80.0 + random.uniform(-20, 20),
            velocity=Vector3(
                x=random.uniform(-2, 2),
                y=random.uniform(-2, 2),
                z=random.uniform(-1, 1),
            ),
            timestamp=int(time.time() * 1000),
        )
        drones.append(drone)
    return drones


def test_boids_swarm(drone_count: int = 20, iterations: int = 10):
    print(f"=== Boids 算法集群测试 ===")
    print(f"无人机数量: {drone_count}")
    print(f"迭代次数: {iterations}")
    print()

    drones = generate_swarm_drones(drone_count, 39.9042, 116.4074)
    boids = BoidsAlgorithm()

    print(f"初始状态:")
    print(f"  中心纬度: {sum(d.latitude for d in drones)/len(drones):.6f}")
    print(f"  中心经度: {sum(d.longitude for d in drones)/len(drones):.6f}")
    print(f"  平均高度: {sum(d.altitude for d in drones)/len(drones):.2f}m")
    print()

    for iteration in range(iterations):
        commands = boids.compute_all_commands(drones)

        total_separation = sum(c.separation_force for c in commands)
        total_alignment = sum(c.alignment_force for c in commands)
        total_cohesion = sum(c.cohesion_force for c in commands)

        avg_speed = sum(
            math.sqrt(c.target_velocity.x**2 + c.target_velocity.y**2 + c.target_velocity.z**2)
            for c in commands
        ) / len(commands)

        print(f"迭代 {iteration + 1}/{iterations}:")
        print(f"  分离力总和: {total_separation:.2f}")
        print(f"  对齐力总和: {total_alignment:.2f}")
        print(f"  凝聚力总和: {total_cohesion:.2f}")
        print(f"  平均目标速度: {avg_speed:.2f} m/s")

        for i, drone in enumerate(drones):
            cmd = commands[i]
            drone.velocity = Vector3(
                x=cmd.target_velocity.x,
                y=cmd.target_velocity.y,
                z=cmd.target_velocity.z,
            )
            lat_offset = cmd.target_velocity.y * 0.00001
            lon_offset = cmd.target_velocity.x * 0.00001
            drone.latitude += lat_offset
            drone.longitude += lon_offset
            drone.altitude += cmd.target_velocity.z

        print()

    print(f"最终状态:")
    print(f"  中心纬度: {sum(d.latitude for d in drones)/len(drones):.6f}")
    print(f"  中心经度: {sum(d.longitude for d in drones)/len(drones):.6f}")
    print(f"  平均高度: {sum(d.altitude for d in drones)/len(drones):.2f}m")
    print()
    print("测试完成!")


def test_redis_manager():
    print("=== Redis 状态管理器测试 ===")
    print("注意: 此测试需要 Redis 服务运行中")
    print()

    try:
        from redis_manager.state_manager import RedisStateManager

        redis_mgr = RedisStateManager()

        test_drone = DroneTelemetry(
            drone_id="test-drone-001",
            latitude=39.9042,
            longitude=116.4074,
            altitude=50.0,
            velocity=Vector3(x=1.0, y=0.0, z=0.0),
            battery_level=85.5,
        )

        print("写入遥测数据...")
        success = redis_mgr.update_telemetry(test_drone)
        print(f"写入成功: {success}")

        print("读取遥测数据...")
        telemetry = redis_mgr.get_telemetry("test-drone-001")
        if telemetry:
            print(f"  无人机ID: {telemetry.drone_id}")
            print(f"  GeoHash: {telemetry.geohash}")
            print(f"  电量: {telemetry.battery_level}%")
        else:
            print("  未找到数据")

        print("获取在线无人机数量...")
        count = redis_mgr.get_online_count()
        print(f"  在线数量: {count}")

        print()
        print("Redis 测试完成!")

    except Exception as e:
        print(f"Redis 测试失败: {e}")
        print("请确保 Redis 服务已启动")


if __name__ == "__main__":
    print("=" * 60)
    print("  无人机编队控制系统 - 模拟测试")
    print("=" * 60)
    print()

    test_boids_swarm(drone_count=20, iterations=5)

    print()
    print("=" * 60)
    print()

    test_redis_manager()
