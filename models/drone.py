from pydantic import BaseModel
from typing import Optional
from datetime import datetime


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
