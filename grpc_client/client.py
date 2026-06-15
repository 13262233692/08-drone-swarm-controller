import grpc
import time
from typing import Optional, Generator

from config import settings
from models.drone import DroneTelemetry, DroneCommand, Vector3, EulerAngles

from grpc_gen import drone_swarm_pb2
from grpc_gen import drone_swarm_pb2_grpc


class GrpcClient:
    def __init__(self, host: str = None, port: int = None):
        self.host = host or settings.grpc_host
        self.port = port or settings.grpc_port
        self.channel = None
        self.telemetry_stub = None
        self.command_stub = None

    def connect(self):
        address = f"{self.host}:{self.port}"
        self.channel = grpc.insecure_channel(address)
        self.telemetry_stub = drone_swarm_pb2_grpc.TelemetryServiceStub(self.channel)
        self.command_stub = drone_swarm_pb2_grpc.CommandServiceStub(self.channel)
        print(f"[gRPC Client] Connected to {address}")

    def close(self):
        if self.channel:
            self.channel.close()
            print("[gRPC Client] Connection closed")

    def send_telemetry(self, telemetry: DroneTelemetry) -> bool:
        try:
            request = drone_swarm_pb2.TelemetryRequest(
                drone_id=telemetry.drone_id,
                latitude=telemetry.latitude,
                longitude=telemetry.longitude,
                altitude=telemetry.altitude,
                attitude=drone_swarm_pb2.EulerAngles(
                    roll=telemetry.attitude.roll,
                    pitch=telemetry.attitude.pitch,
                    yaw=telemetry.attitude.yaw,
                ),
                battery_level=telemetry.battery_level,
                velocity=drone_swarm_pb2.Vector3(
                    x=telemetry.velocity.x,
                    y=telemetry.velocity.y,
                    z=telemetry.velocity.z,
                ),
                timestamp=telemetry.timestamp or int(time.time() * 1000),
            )

            response = self.telemetry_stub.SendTelemetry(request)
            return response.success
        except Exception as e:
            print(f"[gRPC Client] Send telemetry error: {e}")
            return False

    def get_command(self, drone_id: str) -> Optional[DroneCommand]:
        try:
            request = drone_swarm_pb2.CommandRequest(drone_id=drone_id)
            response = self.command_stub.GetCommand(request)

            return DroneCommand(
                drone_id=response.drone_id,
                target_velocity=Vector3(
                    x=response.target_velocity.x,
                    y=response.target_velocity.y,
                    z=response.target_velocity.z,
                ),
                target_position=Vector3(
                    x=response.target_position.x,
                    y=response.target_position.y,
                    z=response.target_position.z,
                ),
                separation_force=response.separation_force,
                alignment_force=response.alignment_force,
                cohesion_force=response.cohesion_force,
                timestamp=response.timestamp,
                command_id=response.command_id,
            )
        except Exception as e:
            print(f"[gRPC Client] Get command error: {e}")
            return None

    def stream_telemetry(self, telemetry_generator: Generator[DroneTelemetry, None, None]):
        def request_generator():
            for telemetry in telemetry_generator:
                yield drone_swarm_pb2.TelemetryRequest(
                    drone_id=telemetry.drone_id,
                    latitude=telemetry.latitude,
                    longitude=telemetry.longitude,
                    altitude=telemetry.altitude,
                    attitude=drone_swarm_pb2.EulerAngles(
                        roll=telemetry.attitude.roll,
                        pitch=telemetry.attitude.pitch,
                        yaw=telemetry.attitude.yaw,
                    ),
                    battery_level=telemetry.battery_level,
                    velocity=drone_swarm_pb2.Vector3(
                        x=telemetry.velocity.x,
                        y=telemetry.velocity.y,
                        z=telemetry.velocity.z,
                    ),
                    timestamp=telemetry.timestamp or int(time.time() * 1000),
                )

        try:
            responses = self.telemetry_stub.StreamTelemetry(request_generator())
            for response in responses:
                yield DroneCommand(
                    drone_id=response.drone_id,
                    target_velocity=Vector3(
                        x=response.target_velocity.x,
                        y=response.target_velocity.y,
                        z=response.target_velocity.z,
                    ),
                    target_position=Vector3(
                        x=response.target_position.x,
                        y=response.target_position.y,
                        z=response.target_position.z,
                    ),
                    separation_force=response.separation_force,
                    alignment_force=response.alignment_force,
                    cohesion_force=response.cohesion_force,
                    timestamp=response.timestamp,
                    command_id=response.command_id,
                )
        except Exception as e:
            print(f"[gRPC Client] Stream telemetry error: {e}")

    def stream_commands(self, drone_id: str) -> Generator[DroneCommand, None, None]:
        try:
            request = drone_swarm_pb2.CommandRequest(drone_id=drone_id)
            responses = self.command_stub.StreamCommands(request)

            for response in responses:
                yield DroneCommand(
                    drone_id=response.drone_id,
                    target_velocity=Vector3(
                        x=response.target_velocity.x,
                        y=response.target_velocity.y,
                        z=response.target_velocity.z,
                    ),
                    target_position=Vector3(
                        x=response.target_position.x,
                        y=response.target_position.y,
                        z=response.target_position.z,
                    ),
                    separation_force=response.separation_force,
                    alignment_force=response.alignment_force,
                    cohesion_force=response.cohesion_force,
                    timestamp=response.timestamp,
                    command_id=response.command_id,
                )
        except Exception as e:
            print(f"[gRPC Client] Stream commands error: {e}")
