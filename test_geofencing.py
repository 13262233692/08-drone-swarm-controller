import sys
import os
import math
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from models.drone import (
    GeoFence,
    GeoPoint,
    DroneTelemetry,
    Vector3,
    FenceType,
    FencePriority,
    EulerAngles,
)
from geofencing.engine import GeoFencingEngine
from geofencing.manager import GeoFenceManager
from boids.algorithm import BoidsAlgorithm


def create_square_fence(
    center_lat: float,
    center_lon: float,
    size_km: float,
    fence_id: str,
    name: str,
    fence_type: FenceType = FenceType.EXCLUSION,
) -> GeoFence:
    half_size = size_km / 2.0
    meters_per_deg = 111320.0
    deg_offset_lat = (half_size * 1000) / meters_per_deg
    deg_offset_lon = deg_offset_lat / math.cos(math.radians(center_lat))

    polygon = [
        GeoPoint(latitude=center_lat + deg_offset_lat, longitude=center_lon - deg_offset_lon),
        GeoPoint(latitude=center_lat + deg_offset_lat, longitude=center_lon + deg_offset_lon),
        GeoPoint(latitude=center_lat - deg_offset_lat, longitude=center_lon + deg_offset_lon),
        GeoPoint(latitude=center_lat - deg_offset_lat, longitude=center_lon - deg_offset_lon),
    ]

    return GeoFence(
        fence_id=fence_id,
        name=name,
        fence_type=fence_type,
        priority=FencePriority.HIGH,
        polygon=polygon,
        min_altitude=0.0,
        max_altitude=200.0,
        repulsion_force=30.0,
        warning_distance_meters=300.0,
        emergency_distance_meters=150.0,
    )


def test_ray_casting():
    print("=" * 70)
    print("  测试 1: 射线交叉算法 (Ray Casting) - 点在多边形内判断")
    print("=" * 70)
    print()

    engine = GeoFencingEngine()
    fence = create_square_fence(39.9042, 116.4074, 1.0, "test-square", "Test Square")

    meters_per_deg = 111320.0
    deg_1km = 1.0 / meters_per_deg * 1000

    print(f"围栏范围: 中心 (39.9042, 116.4074), 1km x 1km")
    print(f"纬度边界: {39.9042 - deg_1km/2:.6f} ~ {39.9042 + deg_1km/2:.6f}")
    print()

    inside_lat_lo = 39.9042 - deg_1km * 0.25
    inside_lat_hi = 39.9042 + deg_1km * 0.25
    outside_lat = 39.9042 + deg_1km * 0.75

    test_cases = [
        (39.9042, 116.4074, True, "中心点 - 应该在内部"),
        (inside_lat_hi, 116.4074, True, "北边界附近(250m) - 应该在内部"),
        (inside_lat_lo, 116.4074, True, "南边界附近(250m) - 应该在内部"),
        (outside_lat, 116.4074, False, "北部750m外 - 应该在外部"),
        (39.9042 - deg_1km * 0.75, 116.4074, False, "南部750m外 - 应该在外部"),
        (0.0, 0.0, False, "原点(0,0) - 应该在外部"),
    ]

    passed = 0
    failed = 0
    for lat, lon, expected, desc in test_cases:
        result = engine.ray_casting_point_in_polygon(lat, lon, fence.polygon)
        status = "PASS" if result == expected else "FAIL"
        if result == expected:
            passed += 1
        else:
            failed += 1
        print(f"  [{status}]: {desc}")
        print(f"         点({lat:.6f}, {lon:.6f}) -> 实际:{result}, 预期:{expected}")

    print()
    print(f"  结果: {passed}/{passed+failed} 通过")
    print()
    return failed == 0


def test_collision_prediction():
    print("=" * 70)
    print("  测试 2: 未来3秒轨迹碰撞预测 (线段相交检测)")
    print("=" * 70)
    print()

    engine = GeoFencingEngine()
    fence = create_square_fence(39.9042, 116.4074, 0.5, "test-square", "Test Square")
    fence.warning_distance_meters = 100.0
    fence.emergency_distance_meters = 50.0
    engine.set_fences([fence])

    meters_per_deg = 111320.0
    deg_250m = 0.25 / meters_per_deg * 1000
    deg_500m = 0.5 / meters_per_deg * 1000

    print(f"围栏: 0.5km x 0.5km 禁飞区, 中心 (39.9042, 116.4074)")
    print()

    deg_100m = 100.0 / meters_per_deg

    drone_outside_flying_in = DroneTelemetry(
        drone_id="test-drone-001",
        latitude=39.9042 + deg_250m + deg_100m,
        longitude=116.4074,
        altitude=50.0,
        velocity=Vector3(x=0.0, y=-80.0, z=0.0),
        attitude=EulerAngles(),
        battery_level=90.0,
    )

    drone_outside_far = DroneTelemetry(
        drone_id="test-drone-002",
        latitude=39.9042 + deg_500m * 5,
        longitude=116.4074,
        altitude=50.0,
        velocity=Vector3(x=0.0, y=0.0, z=0.0),
        attitude=EulerAngles(),
        battery_level=90.0,
    )

    drone_inside = DroneTelemetry(
        drone_id="test-drone-003",
        latitude=39.9042,
        longitude=116.4074,
        altitude=50.0,
        velocity=Vector3(x=0.0, y=0.0, z=0.0),
        attitude=EulerAngles(),
        battery_level=90.0,
    )

    test_cases = [
        (drone_outside_flying_in, "外部高速(80m/s)飞入禁飞区", True, "emergency"),
        (drone_outside_far, "远处静止,距离>1km", False, None),
        (drone_inside, "在禁飞区内部静止", True, "critical"),
    ]

    all_pass = True
    for drone, desc, should_trigger, expected_severity in test_cases:
        result = engine.evaluate_drone(drone)
        severity = None
        if result.warnings:
            severity = result.warnings[0].severity

        triggered = result.is_active
        passed = (triggered == should_trigger) and (
            expected_severity is None or severity == expected_severity
        )

        status = "PASS" if passed else "FAIL"
        if not passed:
            all_pass = False

        speed = math.sqrt(drone.velocity.x**2 + drone.velocity.y**2)
        print(f"  [{status}]: {desc}")
        print(f"         位置({drone.latitude:.6f}, {drone.longitude:.6f}), 速度: {speed:.1f}m/s")
        print(f"         围栏触发: {triggered} (预期: {should_trigger})")
        if triggered:
            print(f"         严重程度: {severity} (预期: {expected_severity})")
            print(f"         排斥力: ({result.repulsion_force.x:.1f}, {result.repulsion_force.y:.1f}, {result.repulsion_force.z:.1f})")
            force_mag = math.sqrt(
                result.repulsion_force.x**2
                + result.repulsion_force.y**2
                + result.repulsion_force.z**2
            )
            print(f"         排斥力大小: {force_mag:.1f}")
        print()

    return all_pass


def test_normal_calculation():
    print("=" * 70)
    print("  测试 3: 边界法线方向与排斥力计算")
    print("=" * 70)
    print()

    engine = GeoFencingEngine()
    fence = create_square_fence(39.9042, 116.4074, 0.5, "test-square", "Test Square")
    fence.warning_distance_meters = 400.0
    fence.emergency_distance_meters = 200.0
    engine.set_fences([fence])

    meters_per_deg = 111320.0
    deg_50m = 50.0 / meters_per_deg
    deg_250m = 250.0 / meters_per_deg

    print(f"正方形围栏(0.5km x 0.5km), 检查四边排斥力方向是否正确(向外)")
    print(f"测试点距离边界约50m")
    print()

    positions = [
        (39.9042 + deg_250m + deg_50m, 116.4074, "北边界", 0.0, 1.0),
        (39.9042 - deg_250m - deg_50m, 116.4074, "南边界", 0.0, -1.0),
        (39.9042, 116.4074 + deg_250m + deg_50m, "东边界", 1.0, 0.0),
        (39.9042, 116.4074 - deg_250m - deg_50m, "西边界", -1.0, 0.0),
    ]

    all_pass = True
    for lat, lon, edge_name, exp_x_sign, exp_y_sign in positions:
        drone = DroneTelemetry(
            drone_id=f"test-edge-{edge_name}",
            latitude=lat,
            longitude=lon,
            altitude=50.0,
            velocity=Vector3(x=0.0, y=0.0, z=0.0),
            attitude=EulerAngles(),
            battery_level=90.0,
        )

        result = engine.evaluate_drone(drone)
        force = result.repulsion_force

        x_sign = 1.0 if force.x > 0.5 else (-1.0 if force.x < -0.5 else 0.0)
        y_sign = 1.0 if force.y > 0.5 else (-1.0 if force.y < -0.5 else 0.0)

        x_ok = (exp_x_sign == 0.0) or (x_sign == exp_x_sign)
        y_ok = (exp_y_sign == 0.0) or (y_sign == exp_y_sign)
        passed = x_ok and y_ok

        force_mag = math.sqrt(force.x**2 + force.y**2 + force.z**2)
        status = "PASS" if passed else "FAIL"
        if not passed:
            all_pass = False

        print(f"  [{status}]: {edge_name}")
        print(f"         位置({lat:.6f}, {lon:.6f})")
        print(f"         排斥力: ({force.x:.2f}, {force.y:.2f}, {force.z:.2f}), 大小: {force_mag:.1f}")
        print(f"         方向: x={x_sign:+.0f}(预期{exp_x_sign:+.0f}), y={y_sign:+.0f}(预期{exp_y_sign:+.0f})")
        print()

    return all_pass


def test_altitude_fence():
    print("=" * 70)
    print("  测试 4: 三维电子围栏 - 高度限制")
    print("=" * 70)
    print()

    engine = GeoFencingEngine()
    fence = create_square_fence(39.9042, 116.4074, 1.0, "test-alt", "Test Altitude")
    fence.min_altitude = 40.0
    fence.max_altitude = 60.0
    fence.warning_distance_meters = 500.0
    fence.emergency_distance_meters = 200.0
    engine.set_fences([fence])

    print(f"禁飞区: 平面1km x 1km, 高度范围: 40m ~ 60m")
    print()

    test_cases = [
        (39.9042, 116.4074, 50.0, True, "禁区中心(50m) - 平面+高度都在范围内,应触发"),
        (39.9042, 116.4074, 10.0, False, "禁区平面内,高度10m(不在40-60) - 三维外,不触发"),
        (39.9042, 116.4074, 100.0, False, "禁区平面内,高度100m(不在40-60) - 三维外,不触发"),
        (39.9042 + 0.02, 116.4074, 50.0, False, "平面外(约2km北),高度50m - 不触发"),
    ]

    all_pass = True
    for lat, lon, alt, should_trigger, desc in test_cases:
        drone = DroneTelemetry(
            drone_id=f"test-alt-{alt}",
            latitude=lat,
            longitude=lon,
            altitude=alt,
            velocity=Vector3(x=0.0, y=0.0, z=0.0),
            attitude=EulerAngles(),
            battery_level=90.0,
        )

        result = engine.evaluate_drone(drone)
        triggered = result.is_active
        z_force = result.repulsion_force.z

        passed = triggered == should_trigger

        status = "PASS" if passed else "FAIL"
        if not passed:
            all_pass = False

        print(f"  [{status}]: {desc}")
        print(f"         高度={alt}m, 围栏触发={triggered}(预期={should_trigger}), Z轴排斥力={z_force:.1f}")
        if triggered and result.warnings:
            print(f"         严重程度: {result.warnings[0].severity}")
        print()

    return all_pass


def test_boids_integration():
    print("=" * 70)
    print("  测试 5: Boids 集成 - 围栏力覆盖常规编队指令")
    print("=" * 70)
    print()

    engine = GeoFencingEngine()
    fence = create_square_fence(39.9042, 116.4074, 0.3, "critical-fence", "Critical Zone")
    fence.priority = FencePriority.MILITARY
    fence.repulsion_force = 80.0
    fence.warning_distance_meters = 500.0
    fence.emergency_distance_meters = 200.0
    engine.set_fences([fence])

    boids = BoidsAlgorithm(fence_engine=engine)

    drone_inside = DroneTelemetry(
        drone_id="boids-test-001",
        latitude=39.9042,
        longitude=116.4074,
        altitude=50.0,
        velocity=Vector3(x=2.0, y=1.0, z=0.0),
        attitude=EulerAngles(),
        battery_level=90.0,
    )

    meters_per_deg = 111320.0
    deg_2km = 2.0 / meters_per_deg * 1000
    drone_outside = DroneTelemetry(
        drone_id="boids-test-002",
        latitude=39.9042 + deg_2km,
        longitude=116.4074,
        altitude=50.0,
        velocity=Vector3(x=0.0, y=0.0, z=0.0),
        attitude=EulerAngles(),
        battery_level=90.0,
    )

    drones = [drone_inside, drone_outside]

    print("场景: 1架无人机在军事禁区内, 1架在2km外安全区域")
    print()

    all_pass = True
    for i, drone in enumerate(drones):
        command = boids.compute_command(drone, drones)
        fence_result = boids.get_fence_result(drone.drone_id)

        in_fence = drone.drone_id == "boids-test-001"

        print(f"  无人机 {drone.drone_id}:")
        print(f"    在禁区内: {in_fence}")
        print(f"    fence_active: {command.fence_active}")
        print(f"    fence_force: {command.fence_force:.2f}")
        print(f"    violating_fences: {command.violating_fences}")
        print(f"    目标速度: ({command.target_velocity.x:.2f}, {command.target_velocity.y:.2f}, {command.target_velocity.z:.2f})")

        if in_fence:
            if command.fence_active and command.fence_force > 10:
                print(f"    [PASS]: 禁区内无人机被强排斥力推离")
            else:
                print(f"    [FAIL]: 禁区内无人机未被正确排斥!")
                all_pass = False
        else:
            if not command.fence_active:
                print(f"    [PASS]: 外部无人机正常编队飞行")
            else:
                print(f"    [WARN]: 外部无人机也触发了围栏, 检查距离阈值")

        if fence_result and fence_result.warnings:
            print(f"    违规警告:")
            for w in fence_result.warnings:
                print(f"      - [{w.severity.upper()}] {w.violation_type}, 距离边界 {w.distance_to_boundary:.1f}m")
        print()

    return all_pass


def test_flight_simulation():
    print("=" * 70)
    print("  测试 6: 动态模拟 - 无人机飞向禁飞区并被强制推离")
    print("=" * 70)
    print()

    engine = GeoFencingEngine()
    fence = create_square_fence(39.9042, 116.4074, 0.3, "sim-fence", "Restricted Airspace")
    fence.warning_distance_meters = 500.0
    fence.emergency_distance_meters = 200.0
    engine.set_fences([fence])

    boids = BoidsAlgorithm(fence_engine=engine)

    meters_per_deg = 111320.0
    deg_800m = 0.8 / meters_per_deg * 1000

    start_lat = 39.9042 + deg_800m
    start_lon = 116.4074

    drone = DroneTelemetry(
        drone_id="sim-001",
        latitude=start_lat,
        longitude=start_lon,
        altitude=50.0,
        velocity=Vector3(x=0.0, y=-40.0, z=0.0),
        attitude=EulerAngles(),
        battery_level=95.0,
    )

    print(f"初始位置: ({start_lat:.6f}, {start_lon:.6f}), 速度向南 40m/s")
    print(f"目标: 飞向 0.3km x 0.3km 禁飞区 (中心: 39.9042, 116.4074)")
    print()
    print(f"{'时间(s)':<10} {'纬度':<12} {'速度Y':<10} {'围栏力':<10} {'状态'}")
    print("-" * 70)

    dt = 0.5
    evaded = False
    all_pass = True
    reversal_detected = False

    for step in range(0, 50):
        t = step * dt
        command = boids.compute_command(drone, [drone])

        current_state = "正常飞行"
        if command.fence_active and command.fence_force > 0.5:
            evaded = True
            fence_result = boids.get_fence_result(drone.drone_id)
            if fence_result and fence_result.warnings:
                sev = fence_result.warnings[0].severity
                current_state = f"[{sev.upper()}] 避让中"

        if drone.velocity.y > 0 and not reversal_detected:
            reversal_detected = True
            print(
                f"* {t:<9.1f} "
                f"{drone.latitude:<12.6f} "
                f"{drone.velocity.y:<10.1f} "
                f"{command.fence_force:<10.1f} "
                f"方向反转! 开始逃离"
            )
        else:
            print(
                f"  {t:<9.1f} "
                f"{drone.latitude:<12.6f} "
                f"{drone.velocity.y:<10.1f} "
                f"{command.fence_force:<10.1f} "
                f"{current_state}"
            )

        drone.velocity = command.target_velocity

        lat_offset = drone.velocity.y * dt * 0.00001
        lon_offset = drone.velocity.x * dt * 0.00001

        drone.latitude += lat_offset
        drone.longitude += lon_offset
        drone.altitude += drone.velocity.z * dt

        if reversal_detected and drone.velocity.y > 10 and command.fence_force < 1.0:
            break

    print()
    if evaded and reversal_detected:
        print("[PASS]: 无人机成功检测到禁飞区并被强制推离!")
    else:
        if not evaded:
            print("[FAIL]: 无人机未能触发紧急避让!")
        if not reversal_detected:
            print("[FAIL]: 无人机未能被推离(速度方向未反转)!")
        all_pass = False

    print()
    return all_pass


def main():
    print()
    print("=" * 70)
    print("     动态三维电子围栏 (Geo-fencing) 强制避让系统 - 综合测试")
    print("=" * 70)
    print()

    tests = [
        ("射线交叉算法", test_ray_casting),
        ("轨迹碰撞预测", test_collision_prediction),
        ("法线方向与排斥力", test_normal_calculation),
        ("三维高度限制", test_altitude_fence),
        ("Boids 集成覆盖", test_boids_integration),
        ("动态飞行避让模拟", test_flight_simulation),
    ]

    results = []
    for name, test_fn in tests:
        try:
            passed = test_fn()
            results.append((name, passed))
        except Exception as e:
            print(f"  [FAIL]: 测试异常: {e}")
            import traceback
            traceback.print_exc()
            results.append((name, False))
        time.sleep(0.3)

    print()
    print("=" * 70)
    print("  测试总结")
    print("=" * 70)
    for name, passed in results:
        status = "PASS" if passed else "FAIL"
        print(f"  [{status}]: {name}")

    total_pass = sum(1 for _, p in results if p)
    total = len(results)
    print()
    print(f"  总计: {total_pass}/{total} 通过")

    if total_pass == total:
        print()
        print(" 所有 Geo-fencing 测试通过!")
    else:
        print()
        print(" 部分测试未通过, 请检查上述失败项")


if __name__ == "__main__":
    main()
