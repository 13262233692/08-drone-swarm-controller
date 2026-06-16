import math
from typing import List, Tuple, Optional
from models.drone import (
    GeoFence,
    GeoPoint,
    DroneTelemetry,
    Vector3,
    FenceViolation,
    FenceRepulsionResult,
    FenceType,
    FencePriority,
)
from config import settings


class GeoFencingEngine:
    def __init__(self):
        self.fences: List[GeoFence] = []
        self._prediction_seconds = 3.0
        self._earth_radius_km = 6371.0
        self._meters_per_deg_lat = 111320.0
        self._max_repulsion_force = 100.0

    def set_fences(self, fences: List[GeoFence]):
        self.fences = [f for f in fences if f.enabled]

    def add_fence(self, fence: GeoFence):
        if not fence.enabled:
            return
        existing = next((f for f in self.fences if f.fence_id == fence.fence_id), None)
        if existing:
            self.fences.remove(existing)
        self.fences.append(fence)

    def remove_fence(self, fence_id: str):
        self.fences = [f for f in self.fences if f.fence_id != fence_id]

    def get_fences(self) -> List[GeoFence]:
        return list(self.fences)

    def _latlon_to_meters(
        self, lat: float, lon: float, ref_lat: float, ref_lon: float
    ) -> Tuple[float, float]:
        x = (lon - ref_lon) * self._meters_per_deg_lat * math.cos(math.radians(ref_lat))
        y = (lat - ref_lat) * self._meters_per_deg_lat
        return x, y

    def _meters_to_latlon(
        self, x: float, y: float, ref_lat: float, ref_lon: float
    ) -> Tuple[float, float]:
        lon = ref_lon + x / (self._meters_per_deg_lat * math.cos(math.radians(ref_lat)))
        lat = ref_lat + y / self._meters_per_deg_lat
        return lat, lon

    def ray_casting_point_in_polygon(
        self, point_lat: float, point_lon: float, polygon: List[GeoPoint]
    ) -> bool:
        if len(polygon) < 3:
            return False

        x, y = point_lon, point_lat
        n = len(polygon)
        inside = False

        j = n - 1
        for i in range(n):
            xi, yi = polygon[i].longitude, polygon[i].latitude
            xj, yj = polygon[j].longitude, polygon[j].latitude

            if ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / (yj - yi) + xi
            ):
                inside = not inside
            j = i

        return inside

    def _segments_intersect(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        p3: Tuple[float, float],
        p4: Tuple[float, float],
    ) -> bool:
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

        return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

    def _segment_intersection_point(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        p3: Tuple[float, float],
        p4: Tuple[float, float],
    ) -> Optional[Tuple[float, float]]:
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4

        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-12:
            return None

        t_numer = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)
        t = t_numer / denom

        u_numer = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3))
        u = u_numer / denom

        if 0 <= t <= 1 and 0 <= u <= 1:
            ix = x1 + t * (x2 - x1)
            iy = y1 + t * (y2 - y1)
            return (ix, iy)

        return None

    def _point_to_segment_distance(
        self,
        px: float,
        py: float,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
    ) -> Tuple[float, float, float]:
        dx = x2 - x1
        dy = y2 - y1

        if abs(dx) < 1e-12 and abs(dy) < 1e-12:
            dist = math.sqrt((px - x1) ** 2 + (py - y1) ** 2)
            return dist, x1, y1

        t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))

        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        dist = math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)

        return dist, closest_x, closest_y

    def _point_to_polygon_distance(
        self, lat: float, lon: float, polygon: List[GeoPoint]
    ) -> Tuple[float, float, float, int]:
        n = len(polygon)
        min_dist = float("inf")
        closest_x = 0.0
        closest_y = 0.0
        closest_edge = -1

        ref_lat = lat
        ref_lon = lon

        px, py = self._latlon_to_meters(lat, lon, ref_lat, ref_lon)

        for i in range(n):
            j = (i + 1) % n
            p1 = polygon[i]
            p2 = polygon[j]

            x1, y1 = self._latlon_to_meters(p1.latitude, p1.longitude, ref_lat, ref_lon)
            x2, y2 = self._latlon_to_meters(p2.latitude, p2.longitude, ref_lat, ref_lon)

            dist, cx, cy = self._point_to_segment_distance(px, py, x1, y1, x2, y2)

            if dist < min_dist:
                min_dist = dist
                closest_x = cx
                closest_y = cy
                closest_edge = i

        return min_dist, closest_x, closest_y, closest_edge

    def _compute_edge_normal(
        self,
        polygon: List[GeoPoint],
        edge_index: int,
        ref_lat: float,
        ref_lon: float,
    ) -> Tuple[float, float]:
        n = len(polygon)
        i = edge_index % n
        j = (i + 1) % n

        p1 = polygon[i]
        p2 = polygon[j]

        x1, y1 = self._latlon_to_meters(p1.latitude, p1.longitude, ref_lat, ref_lon)
        x2, y2 = self._latlon_to_meters(p2.latitude, p2.longitude, ref_lat, ref_lon)

        edge_x = x2 - x1
        edge_y = y2 - y1

        length = math.sqrt(edge_x ** 2 + edge_y ** 2)
        if length < 1e-6:
            return (0.0, -1.0)

        nx = -edge_y / length
        ny = edge_x / length

        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2

        test_x = mid_x + nx * 10
        test_y = mid_y + ny * 10

        test_lat, test_lon = self._meters_to_latlon(test_x, test_y, ref_lat, ref_lon)
        inside = self.ray_casting_point_in_polygon(test_lat, test_lon, polygon)

        if inside:
            nx = -nx
            ny = -ny

        return nx, ny

    def _predict_future_position(
        self, drone: DroneTelemetry, seconds: float
    ) -> Tuple[float, float, float]:
        lat_offset = drone.velocity.y * seconds * 0.00001
        lon_offset = drone.velocity.x * seconds * 0.00001
        alt_offset = drone.velocity.z * seconds

        return (
            drone.latitude + lat_offset,
            drone.longitude + lon_offset,
            drone.altitude + alt_offset,
        )

    def _check_trajectory_intersection(
        self,
        drone: DroneTelemetry,
        polygon: List[GeoPoint],
    ) -> List[Tuple[float, int, float, float]]:
        intersections = []

        future_lat, future_lon, _ = self._predict_future_position(
            drone, self._prediction_seconds
        )

        ref_lat = drone.latitude
        ref_lon = drone.longitude

        px1, py1 = self._latlon_to_meters(drone.latitude, drone.longitude, ref_lat, ref_lon)
        px2, py2 = self._latlon_to_meters(future_lat, future_lon, ref_lat, ref_lon)

        n = len(polygon)
        for i in range(n):
            j = (i + 1) % n
            p1 = polygon[i]
            p2 = polygon[j]

            ex1, ey1 = self._latlon_to_meters(p1.latitude, p1.longitude, ref_lat, ref_lon)
            ex2, ey2 = self._latlon_to_meters(p2.latitude, p2.longitude, ref_lat, ref_lon)

            if self._segments_intersect((px1, py1), (px2, py2), (ex1, ey1), (ex2, ey2)):
                ip = self._segment_intersection_point(
                    (px1, py1), (px2, py2), (ex1, ey1), (ex2, ey2)
                )
                if ip is not None:
                    traj_dist = math.sqrt((ip[0] - px1) ** 2 + (ip[1] - py1) ** 2)
                    total_traj_dist = math.sqrt((px2 - px1) ** 2 + (py2 - py1) ** 2)

                    time_to_collision = self._prediction_seconds
                    if total_traj_dist > 1e-6:
                        time_to_collision = (traj_dist / total_traj_dist) * self._prediction_seconds

                    intersections.append((time_to_collision, i, ip[0], ip[1]))

        intersections.sort(key=lambda x: x[0])
        return intersections

    def _check_altitude_in_range(
        self, drone: DroneTelemetry, fence: GeoFence
    ) -> bool:
        return fence.min_altitude <= drone.altitude <= fence.max_altitude

    def _check_altitude_approaching(
        self, drone: DroneTelemetry, fence: GeoFence
    ) -> Optional[float]:
        future_alt = drone.altitude + drone.velocity.z * self._prediction_seconds
        if fence.min_altitude <= future_alt <= fence.max_altitude:
            current_in = self._check_altitude_in_range(drone, fence)
            if not current_in:
                return min(
                    abs(future_alt - fence.min_altitude),
                    abs(future_alt - fence.max_altitude),
                )
        return None

    def _check_3d_inside(
        self, drone: DroneTelemetry, fence: GeoFence
    ) -> bool:
        inside_2d = self.ray_casting_point_in_polygon(
            drone.latitude, drone.longitude, fence.polygon
        )
        in_altitude = self._check_altitude_in_range(drone, fence)
        return inside_2d and in_altitude

    def evaluate_drone(
        self, drone: DroneTelemetry
    ) -> FenceRepulsionResult:
        total_repulsion = Vector3(x=0.0, y=0.0, z=0.0)
        active_fences: List[str] = []
        violating_fences: List[str] = []
        warnings: List[FenceViolation] = []
        is_active = False

        now_ms = int(__import__("time").time() * 1000)

        for fence in self.fences:
            if not fence.enabled:
                continue

            inside_2d = self.ray_casting_point_in_polygon(
                drone.latitude, drone.longitude, fence.polygon
            )
            in_altitude = self._check_altitude_in_range(drone, fence)
            inside_3d = inside_2d and in_altitude

            approaching_alt = self._check_altitude_approaching(drone, fence)

            dist, _, _, edge_idx = self._point_to_polygon_distance(
                drone.latitude, drone.longitude, fence.polygon
            )

            intersections = self._check_trajectory_intersection(drone, fence.polygon)
            min_collision_time = intersections[0][0] if intersections else 999.0

            fence_triggered = False
            repulsion_x = 0.0
            repulsion_y = 0.0
            repulsion_z = 0.0
            severity = "warning"
            violation_type = "safe"

            if fence.fence_type == FenceType.EXCLUSION:
                altitude_relevant = in_altitude or (approaching_alt is not None and inside_2d)

                if inside_3d:
                    fence_triggered = True
                    severity = "critical"
                    violation_type = "inside_exclusion_zone"
                    violating_fences.append(fence.fence_id)
                elif inside_2d and approaching_alt is not None:
                    fence_triggered = True
                    severity = "emergency"
                    violation_type = "entering_altitude_range"
                elif altitude_relevant and min_collision_time <= 1.5:
                    fence_triggered = True
                    severity = "emergency"
                    violation_type = "imminent_collision"
                elif altitude_relevant and dist < fence.emergency_distance_meters:
                    fence_triggered = True
                    severity = "emergency"
                    violation_type = "too_close_to_boundary"
                elif altitude_relevant and min_collision_time <= self._prediction_seconds:
                    fence_triggered = True
                    severity = "warning"
                    violation_type = "approaching_boundary"
                elif altitude_relevant and dist < fence.warning_distance_meters:
                    fence_triggered = True
                    severity = "warning"
                    violation_type = "near_boundary"

            elif fence.fence_type == FenceType.INCLUSION:
                if not inside_2d or not in_altitude:
                    fence_triggered = True
                    severity = "critical"
                    violation_type = "outside_inclusion_zone"
                    violating_fences.append(fence.fence_id)
                elif min_collision_time <= self._prediction_seconds:
                    fence_triggered = True
                    severity = "warning"
                    violation_type = "approaching_inclusion_boundary"

            if fence_triggered:
                is_active = True
                active_fences.append(fence.fence_id)

                warnings.append(
                    FenceViolation(
                        drone_id=drone.drone_id,
                        fence_id=fence.fence_id,
                        fence_name=fence.name,
                        violation_type=violation_type,
                        latitude=drone.latitude,
                        longitude=drone.longitude,
                        altitude=drone.altitude,
                        distance_to_boundary=dist,
                        predicted_collision_seconds=intersections[0][0] if intersections else 999.0,
                        timestamp=now_ms,
                        severity=severity,
                    )
                )

                force_multiplier = 1.0
                if severity == "critical":
                    force_multiplier = 3.0
                elif severity == "emergency":
                    force_multiplier = 2.0
                elif severity == "warning":
                    force_multiplier = 0.8

                if edge_idx >= 0:
                    nx, ny = self._compute_edge_normal(
                        fence.polygon, edge_idx, drone.latitude, drone.longitude
                    )

                    distance_ratio = 1.0
                    if dist < fence.emergency_distance_meters:
                        distance_ratio = (fence.emergency_distance_meters - dist + 1.0) / fence.emergency_distance_meters
                    elif dist < fence.warning_distance_meters:
                        distance_ratio = (fence.warning_distance_meters - dist + 1.0) / fence.warning_distance_meters

                    force_magnitude = min(
                        fence.repulsion_force * force_multiplier * distance_ratio,
                        self._max_repulsion_force,
                    )

                    repulsion_x = nx * force_magnitude
                    repulsion_y = ny * force_magnitude

                if fence.fence_type == FenceType.EXCLUSION:
                    if inside_3d or (inside_2d and in_altitude):
                        alt_midpoint = (fence.min_altitude + fence.max_altitude) / 2
                        if drone.altitude >= alt_midpoint:
                            repulsion_z = fence.repulsion_force * force_multiplier
                        else:
                            repulsion_z = -fence.repulsion_force * force_multiplier
                    elif approaching_alt is not None and inside_2d:
                        if drone.altitude < fence.min_altitude:
                            repulsion_z = -fence.repulsion_force * force_multiplier * 0.5
                        elif drone.altitude > fence.max_altitude:
                            repulsion_z = fence.repulsion_force * force_multiplier * 0.5
                elif fence.fence_type == FenceType.INCLUSION:
                    if drone.altitude < fence.min_altitude:
                        repulsion_z = fence.repulsion_force * force_multiplier * 0.8
                    elif drone.altitude > fence.max_altitude:
                        repulsion_z = -fence.repulsion_force * force_multiplier * 0.8

                priority_scale = fence.priority.value / FencePriority.MEDIUM.value

                total_repulsion.x += repulsion_x * priority_scale
                total_repulsion.y += repulsion_y * priority_scale
                total_repulsion.z += repulsion_z * priority_scale

        warnings.sort(key=lambda w: (
            0 if w.severity == "critical" else 1 if w.severity == "emergency" else 2,
            w.distance_to_boundary,
        ))

        return FenceRepulsionResult(
            drone_id=drone.drone_id,
            repulsion_force=total_repulsion,
            is_active=is_active,
            active_fences=active_fences,
            violating_fences=violating_fences,
            warnings=warnings,
        )
