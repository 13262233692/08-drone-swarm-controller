import time
import json
import contextlib
from concurrent import futures
import grpc

from config import settings
from redis_manager.state_manager import RedisStateManager, RedisConnectionPool
from models.drone import DroneTelemetry, EulerAngles, Vector3

from grpc_gen import drone_swarm_pb2
from grpc_gen import drone_swarm_pb2_grpc


@contextlib.contextmanager
def get_redis_pubsub():
    pool = RedisConnectionPool()
    redis_client = pool.get_connection()
    pubsub = redis_client.pubsub()
    try:
        yield pubsub, redis_client
    finally:
        try:
            pubsub.unsubscribe()
        except Exception:
            pass
        try:
            pubsub.close()
        except Exception:
            pass
        try:
            redis_client.close()
        except Exception:
            pass


class TelemetryServicer(drone_swarm_pb2_grpc.TelemetryServiceServicer):
    def __init__(self, redis_manager: RedisStateManager):
        self.redis_manager = redis_manager
        self._drone_last_version = {}

    def SendTelemetry(self, request, context):
        try:
            telemetry = DroneTelemetry(
                drone_id=request.drone_id,
                latitude=request.latitude,
                longitude=request.longitude,
                altitude=request.altitude,
                attitude=EulerAngles(
                    roll=request.attitude.roll,
                    pitch=request.attitude.pitch,
                    yaw=request.attitude.yaw,
                ),
                battery_level=request.battery_level,
                velocity=Vector3(
                    x=request.velocity.x,
                    y=request.velocity.y,
                    z=request.velocity.z,
                ),
                timestamp=request.timestamp if request.timestamp else int(time.time() * 1000),
            )

            if request.timestamp > 0:
                last_ts = self._drone_last_version.get(request.drone_id, 0)
                if request.timestamp < last_ts:
                    return drone_swarm_pb2.TelemetryResponse(
                        success=True,
                        message="Stale telemetry ignored",
                    )
                self._drone_last_version[request.drone_id] = request.timestamp

            success = self.redis_manager.update_telemetry(telemetry)

            return drone_swarm_pb2.TelemetryResponse(
                success=success,
                message="Telemetry received" if success else "Failed to process telemetry",
            )
        except Exception as e:
            return drone_swarm_pb2.TelemetryResponse(
                success=False,
                message=str(e),
            )

    def StreamTelemetry(self, request_iterator, context):
        drone_id = None
        try:
            for request in request_iterator:
                try:
                    if drone_id is None:
                        drone_id = request.drone_id

                    telemetry = DroneTelemetry(
                        drone_id=request.drone_id,
                        latitude=request.latitude,
                        longitude=request.longitude,
                        altitude=request.altitude,
                        attitude=EulerAngles(
                            roll=request.attitude.roll,
                            pitch=request.attitude.pitch,
                            yaw=request.attitude.yaw,
                        ),
                        battery_level=request.battery_level,
                        velocity=Vector3(
                            x=request.velocity.x,
                            y=request.velocity.y,
                            z=request.velocity.z,
                        ),
                        timestamp=request.timestamp if request.timestamp else int(time.time() * 1000),
                    )

                    self.redis_manager.update_telemetry(telemetry)

                    command = self.redis_manager.get_command(request.drone_id)
                    if command:
                        yield drone_swarm_pb2.CommandResponse(
                            drone_id=command.drone_id,
                            target_velocity=drone_swarm_pb2.Vector3(
                                x=command.target_velocity.x,
                                y=command.target_velocity.y,
                                z=command.target_velocity.z,
                            ),
                            target_position=drone_swarm_pb2.Vector3(
                                x=command.target_position.x,
                                y=command.target_position.y,
                                z=command.target_position.z,
                            ),
                            separation_force=command.separation_force,
                            alignment_force=command.alignment_force,
                            cohesion_force=command.cohesion_force,
                            timestamp=command.timestamp,
                            command_id=command.command_id,
                        )

                except Exception as e:
                    continue
        except Exception as e:
            pass


class CommandServicer(drone_swarm_pb2_grpc.CommandServiceServicer):
    def __init__(self, redis_manager: RedisStateManager):
        self.redis_manager = redis_manager

    def GetCommand(self, request, context):
        try:
            command = self.redis_manager.get_command(request.drone_id)
            if command:
                return drone_swarm_pb2.CommandResponse(
                    drone_id=command.drone_id,
                    target_velocity=drone_swarm_pb2.Vector3(
                        x=command.target_velocity.x,
                        y=command.target_velocity.y,
                        z=command.target_velocity.z,
                    ),
                    target_position=drone_swarm_pb2.Vector3(
                        x=command.target_position.x,
                        y=command.target_position.y,
                        z=command.target_position.z,
                    ),
                    separation_force=command.separation_force,
                    alignment_force=command.alignment_force,
                    cohesion_force=command.cohesion_force,
                    timestamp=command.timestamp,
                    command_id=command.command_id,
                )
            else:
                return drone_swarm_pb2.CommandResponse(
                    drone_id=request.drone_id,
                    target_velocity=drone_swarm_pb2.Vector3(x=0, y=0, z=0),
                    target_position=drone_swarm_pb2.Vector3(x=0, y=0, z=0),
                    separation_force=0,
                    alignment_force=0,
                    cohesion_force=0,
                    timestamp=int(time.time() * 1000),
                    command_id="",
                )
        except Exception as e:
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(e))
            return drone_swarm_pb2.CommandResponse()

    def StreamCommands(self, request, context):
        drone_id = request.drone_id
        print(f"[gRPC] Command stream started for drone {drone_id}")

        try:
            with get_redis_pubsub() as (pubsub, redis_client):
                channel = f"drone:command:{drone_id}"
                pubsub.subscribe(channel)

                for message in pubsub.listen():
                    if context.is_active():
                        if message["type"] == "message":
                            try:
                                cmd_data = json.loads(message["data"])
                                yield drone_swarm_pb2.CommandResponse(
                                    drone_id=cmd_data["drone_id"],
                                    target_velocity=drone_swarm_pb2.Vector3(
                                        x=cmd_data["target_velocity"]["x"],
                                        y=cmd_data["target_velocity"]["y"],
                                        z=cmd_data["target_velocity"]["z"],
                                    ),
                                    target_position=drone_swarm_pb2.Vector3(
                                        x=cmd_data["target_position"]["x"],
                                        y=cmd_data["target_position"]["y"],
                                        z=cmd_data["target_position"]["z"],
                                    ),
                                    separation_force=cmd_data["separation_force"],
                                    alignment_force=cmd_data["alignment_force"],
                                    cohesion_force=cmd_data["cohesion_force"],
                                    timestamp=cmd_data["timestamp"],
                                    command_id=cmd_data["command_id"],
                                )
                            except Exception as e:
                                continue
                    else:
                        break

        except Exception as e:
            if context.is_active():
                context.set_code(grpc.StatusCode.INTERNAL)
                context.set_details(str(e))
        finally:
            print(f"[gRPC] Command stream ended for drone {drone_id}")


class GrpcServer:
    def __init__(self):
        self.redis_manager = RedisStateManager()
        self.server = None

    def start(self):
        options = [
            ('grpc.max_receive_message_length', 10 * 1024 * 1024),
            ('grpc.max_send_message_length', 10 * 1024 * 1024),
            ('grpc.keepalive_time_ms', 30000),
            ('grpc.keepalive_timeout_ms', 10000),
            ('grpc.http2.max_pings_without_data', 0),
            ('grpc.http2.min_time_between_pings_ms', 10000),
            ('grpc.http2.min_ping_interval_without_data_ms', 300000),
        ]

        self.server = grpc.server(
            futures.ThreadPoolExecutor(max_workers=500, thread_name_prefix="grpc-worker"),
            options=options,
            maximum_concurrent_rpcs=1000,
        )

        telemetry_servicer = TelemetryServicer(self.redis_manager)
        command_servicer = CommandServicer(self.redis_manager)

        drone_swarm_pb2_grpc.add_TelemetryServiceServicer_to_server(
            telemetry_servicer, self.server
        )
        drone_swarm_pb2_grpc.add_CommandServiceServicer_to_server(
            command_servicer, self.server
        )

        address = f"{settings.grpc_host}:{settings.grpc_port}"
        self.server.add_insecure_port(address)
        self.server.start()

        print(f"[gRPC] Server started on {address}")
        print(f"[gRPC] Max workers: 500, Max concurrent RPCs: 1000")

    def wait_for_termination(self):
        if self.server:
            self.server.wait_for_termination()

    def stop(self):
        if self.server:
            self.server.stop(5)
            print("[gRPC] Server stopped")
