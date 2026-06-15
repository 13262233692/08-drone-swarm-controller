import math
import time
import uuid
from typing import List, Tuple
from config import settings
from models.drone import DroneTelemetry, DroneCommand, Vector3


class BoidsAlgorithm:
    def __init__(self):
        self.separation_distance = settings.boids_separation_distance
        self.alignment_distance = settings.boids_alignment_distance
        self.cohesion_distance = settings.boids_cohesion_distance
        self.separation_weight = settings.boids_separation_weight
        self.alignment_weight = settings.boids_alignment_weight
        self.cohesion_weight = settings.boids_cohesion_weight
        self.max_speed = settings.boids_max_speed
        self.max_force = settings.boids_max_force

    def _latlon_to_meters(self, lat: float, lon: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
        R = 6371000.0

        lat_rad = math.radians(lat)
        ref_lat_rad = math.radians(ref_lat)
        dlat = math.radians(lat - ref_lat)
        dlon = math.radians(lon - ref_lon)

        x = dlon * math.cos(ref_lat_rad) * R
        y = dlat * R

        return x, y

    def _meters_to_latlon(self, x: float, y: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
        R = 6371000.0

        ref_lat_rad = math.radians(ref_lat)
        dlat = y / R
        dlon = x / (R * math.cos(ref_lat_rad))

        lat = ref_lat + math.degrees(dlat)
        lon = ref_lon + math.degrees(dlon)

        return lat, lon

    def _distance_3d_meters(self, drone1: DroneTelemetry, drone2: DroneTelemetry) -> float:
        x1, y1 = self._latlon_to_meters(
            drone1.latitude, drone1.longitude, drone1.latitude, drone1.longitude
        )
        x2, y2 = self._latlon_to_meters(
            drone2.latitude, drone2.longitude, drone1.latitude, drone1.longitude
        )

        dx = x2 - x1
        dy = y2 - y1
        dz = drone2.altitude - drone1.altitude

        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def compute_separation(self, drone: DroneTelemetry, neighbors: List[DroneTelemetry]) -> Vector3:
        separation = Vector3()
        count = 0

        for neighbor in neighbors:
            distance = self._distance_3d_meters(drone, neighbor)

            if 0 < distance < self.separation_distance:
                dx, dy = self._latlon_to_meters(
                    neighbor.latitude, neighbor.longitude,
                    drone.latitude, drone.longitude
                )
                dz = neighbor.altitude - drone.altitude

                magnitude = max(distance, 0.1)
                separation.x -= dx / magnitude * (self.separation_distance - distance)
                separation.y -= dy / magnitude * (self.separation_distance - distance)
                separation.z -= dz / magnitude * (self.separation_distance - distance)
                count += 1

        if count > 0:
            separation.x /= count
            separation.y /= count
            separation.z /= count

        return separation

    def compute_alignment(self, drone: DroneTelemetry, neighbors: List[DroneTelemetry]) -> Vector3:
        alignment = Vector3()
        count = 0

        for neighbor in neighbors:
            distance = self._distance_3d_meters(drone, neighbor)

            if 0 < distance < self.alignment_distance:
                alignment.x += neighbor.velocity.x
                alignment.y += neighbor.velocity.y
                alignment.z += neighbor.velocity.z
                count += 1

        if count > 0:
            alignment.x /= count
            alignment.y /= count
            alignment.z /= count

            alignment.x -= drone.velocity.x
            alignment.y -= drone.velocity.y
            alignment.z -= drone.velocity.z

            alignment = self._limit_vector(alignment, self.max_force)

        return alignment

    def compute_cohesion(self, drone: DroneTelemetry, neighbors: List[DroneTelemetry]) -> Vector3:
        center = Vector3()
        count = 0

        for neighbor in neighbors:
            distance = self._distance_3d_meters(drone, neighbor)

            if 0 < distance < self.cohesion_distance:
                dx, dy = self._latlon_to_meters(
                    neighbor.latitude, neighbor.longitude,
                    drone.latitude, drone.longitude
                )
                center.x += dx
                center.y += dy
                center.z += neighbor.altitude
                count += 1

        if count > 0:
            center.x /= count
            center.y /= count
            center.z /= count

            desired = Vector3(
                x=center.x,
                y=center.y,
                z=center.z - drone.altitude
            )

            desired = self._set_magnitude(desired, self.max_speed)

            steer = Vector3(
                x=desired.x - drone.velocity.x,
                y=desired.y - drone.velocity.y,
                z=desired.z - drone.velocity.z
            )

            steer = self._limit_vector(steer, self.max_force)
            return steer

        return Vector3()

    def _limit_vector(self, v: Vector3, max_magnitude: float) -> Vector3:
        mag = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
        if mag > max_magnitude and mag > 0:
            ratio = max_magnitude / mag
            return Vector3(x=v.x * ratio, y=v.y * ratio, z=v.z * ratio)
        return v

    def _set_magnitude(self, v: Vector3, magnitude: float) -> Vector3:
        mag = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
        if mag > 0:
            ratio = magnitude / mag
            return Vector3(x=v.x * ratio, y=v.y * ratio, z=v.z * ratio)
        return Vector3(x=magnitude, y=0, z=0)

    def compute_command(self, drone: DroneTelemetry, all_drones: List[DroneTelemetry]) -> DroneCommand:
        neighbors = [d for d in all_drones if d.drone_id != drone.drone_id]

        separation = self.compute_separation(drone, neighbors)
        alignment = self.compute_alignment(drone, neighbors)
        cohesion = self.compute_cohesion(drone, neighbors)

        separation.x *= self.separation_weight
        separation.y *= self.separation_weight
        separation.z *= self.separation_weight

        alignment.x *= self.alignment_weight
        alignment.y *= self.alignment_weight
        alignment.z *= self.alignment_weight

        cohesion.x *= self.cohesion_weight
        cohesion.y *= self.cohesion_weight
        cohesion.z *= self.cohesion_weight

        total_force = Vector3(
            x=separation.x + alignment.x + cohesion.x,
            y=separation.y + alignment.y + cohesion.y,
            z=separation.z + alignment.z + cohesion.z
        )

        target_velocity = Vector3(
            x=drone.velocity.x + total_force.x,
            y=drone.velocity.y + total_force.y,
            z=drone.velocity.z + total_force.z
        )

        target_velocity = self._limit_vector(target_velocity, self.max_speed)

        lat_offset = target_velocity.y * 0.00001
        lon_offset = target_velocity.x * 0.00001

        target_position = Vector3(
            x=drone.longitude + lon_offset,
            y=drone.latitude + lat_offset,
            z=drone.altitude + target_velocity.z
        )

        separation_mag = math.sqrt(separation.x ** 2 + separation.y ** 2 + separation.z ** 2)
        alignment_mag = math.sqrt(alignment.x ** 2 + alignment.y ** 2 + alignment.z ** 2)
        cohesion_mag = math.sqrt(cohesion.x ** 2 + cohesion.y ** 2 + cohesion.z ** 2)

        return DroneCommand(
            drone_id=drone.drone_id,
            target_velocity=target_velocity,
            target_position=target_position,
            separation_force=separation_mag,
            alignment_force=alignment_mag,
            cohesion_force=cohesion_mag,
            command_id=str(uuid.uuid4())
        )

    def compute_all_commands(self, drones: List[DroneTelemetry]) -> List[DroneCommand]:
        commands = []
        for drone in drones:
            command = self.compute_command(drone, drones)
            commands.append(command)
        return commands
