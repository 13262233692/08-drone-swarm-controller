from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime
from enum import Enum


class Vector3(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class EulerAngles(BaseModel):
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


class DroneTelemetry(BaseModel):
    drone_id: str
    latitude: float
    longitude: float
    altitude: float = 0.0
    attitude: EulerAngles = EulerAngles()
    battery_level: float = 100.0
    velocity: Vector3 = Vector3()
    timestamp: int = 0
    geohash: Optional[str] = None


class DroneCommand(BaseModel):
    drone_id: str
    target_velocity: Vector3 = Vector3()
    target_position: Vector3 = Vector3()
    separation_force: float = 0.0
    alignment_force: float = 0.0
    cohesion_force: float = 0.0
    fence_force: float = 0.0
    fence_active: bool = False
    violating_fences: List[str] = []
    timestamp: int = 0
    command_id: str = ""


class SwarmMission(BaseModel):
    mission_id: str
    name: str
    target_latitude: float
    target_longitude: float
    target_altitude: float = 50.0
    drone_count: int = 10
    formation: str = "circle"
    speed: float = 5.0


class MissionResponse(BaseModel):
    mission_id: str
    status: str
    message: str


class DroneStatus(BaseModel):
    drone_id: str
    latitude: float
    longitude: float
    altitude: float
    battery_level: float
    is_online: bool
    last_update: datetime


class FenceType(str, Enum):
    EXCLUSION = "exclusion"
    INCLUSION = "inclusion"
    TEMPORARY = "temporary"


class FencePriority(int, Enum):
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    CRITICAL = 4
    MILITARY = 5


class GeoPoint(BaseModel):
    latitude: float
    longitude: float


class GeoFence(BaseModel):
    fence_id: str
    name: str
    fence_type: FenceType = FenceType.EXCLUSION
    priority: FencePriority = FencePriority.HIGH
    polygon: List[GeoPoint]
    min_altitude: float = 0.0
    max_altitude: float = 1000.0
    enabled: bool = True
    expire_at: Optional[datetime] = None
    created_at: datetime = Field(default_factory=datetime.now)
    repulsion_force: float = 50.0
    warning_distance_meters: float = 100.0
    emergency_distance_meters: float = 30.0


class FenceViolation(BaseModel):
    drone_id: str
    fence_id: str
    fence_name: str
    violation_type: str
    latitude: float
    longitude: float
    altitude: float
    distance_to_boundary: float
    predicted_collision_seconds: float
    timestamp: int
    severity: str


class FenceRepulsionResult(BaseModel):
    drone_id: str
    repulsion_force: Vector3
    is_active: bool
    active_fences: List[str]
    violating_fences: List[str]
    warnings: List[FenceViolation]
