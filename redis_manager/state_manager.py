import json
import time
import redis
import geohash2
import threading
from typing import Dict, List, Optional, Tuple
from config import settings
from models.drone import DroneTelemetry, DroneCommand


class RedisConnectionPool:
    _instance = None
    _pool = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._init_pool()
        return cls._instance

    def _init_pool(self):
        self._pool = redis.ConnectionPool(
            host=settings.redis_host,
            port=settings.redis_port,
            db=settings.redis_db,
            password=settings.redis_password,
            decode_responses=True,
            max_connections=settings.redis_max_connections,
            socket_timeout=settings.redis_socket_timeout,
            socket_connect_timeout=settings.redis_connect_timeout,
            retry_on_timeout=True,
            health_check_interval=30,
        )

    def get_connection(self) -> redis.Redis:
        return redis.Redis(connection_pool=self._pool)

    def get_pool(self) -> redis.ConnectionPool:
        return self._pool

    def get_pool_stats(self) -> Dict:
        return {
            "max_connections": self._pool.max_connections,
            "current_connections": len(self._pool._available_connections) + self._pool._in_use_connections.qsize() if hasattr(self._pool, '_in_use_connections') else 0,
            "available_connections": len(self._pool._available_connections),
        }


UPDATE_TELEMETRY_LUA = """
local drone_id = ARGV[1]
local telemetry_data = ARGV[2]
local geohash = ARGV[3]
local lat = ARGV[4]
local lon = ARGV[5]
local alt = ARGV[6]
local timestamp = ARGV[7]
local version = ARGV[8]
local ttl = tonumber(ARGV[9])

local telemetry_key = "drone:telemetry:" .. drone_id
local geohash_key = "drone:geohashes"
local online_key = "drone:online"
local version_key = telemetry_key .. ":version"

local current_version = redis.call("GET", version_key)
if current_version ~= false and tonumber(current_version) >= tonumber(version) then
    return 0
end

redis.call("SET", telemetry_key, telemetry_data, "EX", ttl)
redis.call("SET", version_key, version, "EX", ttl)
redis.call("HSET", telemetry_key .. ":geohash", "geohash", geohash, "lat", lat, "lon", lon, "alt", alt, "ts", timestamp, "v", version)
redis.call("EXPIRE", telemetry_key .. ":geohash", ttl)
redis.call("ZADD", geohash_key, 0, drone_id)
redis.call("SADD", online_key, drone_id)

local publish_data = cjson.encode({
    drone_id = drone_id,
    geohash = geohash,
    timestamp = tonumber(timestamp),
    version = tonumber(version)
})
redis.call("PUBLISH", "drone:telemetry:updates", publish_data)

return 1
"""

STORE_COMMAND_LUA = """
local drone_id = ARGV[1]
local command_data = ARGV[2]
local timestamp = ARGV[3]
local version = ARGV[4]
local ttl = tonumber(ARGV[5])

local command_key = "drone:command:" .. drone_id
local version_key = command_key .. ":version"

local current_version = redis.call("GET", version_key)
if current_version ~= false and tonumber(current_version) >= tonumber(version) then
    return 0
end

redis.call("SET", command_key, command_data, "EX", ttl)
redis.call("SET", version_key, version, "EX", ttl)
redis.call("PUBLISH", "drone:command:" .. drone_id, command_data)

return 1
"""

GET_ALL_DRONES_LUA = """
local online_key = "drone:online"
local telemetry_prefix = "drone:telemetry:"
local drone_ids = redis.call("SMEMBERS", online_key)
local result = {}
local to_remove = {}

for i, drone_id in ipairs(drone_ids) do
    local telemetry_key = telemetry_prefix .. drone_id
    local data = redis.call("GET", telemetry_key)
    if data then
        table.insert(result, data)
    else
        table.insert(to_remove, drone_id)
    end
end

if #to_remove > 0 then
    for i, drone_id in ipairs(to_remove) do
        redis.call("SREM", online_key, drone_id)
    end
end

return result
"""

GET_ALL_GEOHASHES_LUA = """
local online_key = "drone:online"
local telemetry_prefix = "drone:telemetry:"
local drone_ids = redis.call("SMEMBERS", online_key)
local result = {}

for i, drone_id in ipairs(drone_ids) do
    local geohash_key = telemetry_prefix .. drone_id .. ":geohash"
    local data = redis.call("HMGET", geohash_key, "geohash", "lat", "lon", "alt", "ts", "v")
    if data[1] ~= false then
        table.insert(result, {
            drone_id,
            data[1],
            data[2],
            data[3],
            data[4],
            data[5],
            data[6]
        })
    end
end

return result
"""


class RedisStateManager:
    def __init__(self):
        self._pool = RedisConnectionPool()
        self._redis = self._pool.get_connection()

        self._update_telemetry_script = self._redis.register_script(UPDATE_TELEMETRY_LUA)
        self._store_command_script = self._redis.register_script(STORE_COMMAND_LUA)
        self._get_all_drones_script = self._redis.register_script(GET_ALL_DRONES_LUA)
        self._get_all_geohashes_script = self._redis.register_script(GET_ALL_GEOHASHES_LUA)

        self._version_counter = 0
        self._version_lock = threading.Lock()

        self.telemetry_key_prefix = "drone:telemetry:"
        self.geohash_key = "drone:geohashes"
        self.command_key_prefix = "drone:command:"
        self.telemetry_channel = "drone:telemetry:updates"
        self.command_channel_prefix = "drone:command:"
        self.mission_key = "swarm:mission"
        self.online_drones_key = "drone:online"

    def _get_next_version(self) -> int:
        with self._version_lock:
            self._version_counter += 1
            return self._version_counter

    def _get_telemetry_key(self, drone_id: str) -> str:
        return f"{self.telemetry_key_prefix}{drone_id}"

    def _get_command_key(self, drone_id: str) -> str:
        return f"{self.command_key_prefix}{drone_id}"

    def _get_command_channel(self, drone_id: str) -> str:
        return f"{self.command_channel_prefix}{drone_id}"

    def _compute_geohash(self, lat: float, lon: float) -> str:
        return geohash2.encode(lat, lon, precision=settings.geohash_precision)

    def get_redis_client(self) -> redis.Redis:
        return self._pool.get_connection()

    def update_telemetry(self, telemetry: DroneTelemetry) -> bool:
        try:
            telemetry.geohash = self._compute_geohash(
                telemetry.latitude, telemetry.longitude
            )
            telemetry.timestamp = int(time.time() * 1000)
            version = self._get_next_version()

            data = telemetry.model_dump_json()

            result = self._update_telemetry_script(
                args=[
                    telemetry.drone_id,
                    data,
                    telemetry.geohash,
                    str(telemetry.latitude),
                    str(telemetry.longitude),
                    str(telemetry.altitude),
                    str(telemetry.timestamp),
                    str(version),
                    str(settings.telemetry_ttl),
                ]
            )

            return result == 1
        except Exception as e:
            print(f"[Redis] Update telemetry error: {e}")
            return False

    def update_telemetry_batch(self, telemetry_list: List[DroneTelemetry]) -> int:
        if not telemetry_list:
            return 0

        success_count = 0
        pipe = self._redis.pipeline(transaction=False)

        for telemetry in telemetry_list:
            telemetry.geohash = self._compute_geohash(
                telemetry.latitude, telemetry.longitude
            )
            telemetry.timestamp = int(time.time() * 1000)
            version = self._get_next_version()

            data = telemetry.model_dump_json()

            self._update_telemetry_script(
                args=[
                    telemetry.drone_id,
                    data,
                    telemetry.geohash,
                    str(telemetry.latitude),
                    str(telemetry.longitude),
                    str(telemetry.altitude),
                    str(telemetry.timestamp),
                    str(version),
                    str(settings.telemetry_ttl),
                ],
                client=pipe,
            )
            success_count += 1

        try:
            results = pipe.execute()
            return sum(1 for r in results if r == 1)
        except Exception as e:
            print(f"[Redis] Batch update telemetry error: {e}")
            return 0

    def get_telemetry(self, drone_id: str, min_timestamp: int = 0) -> Optional[DroneTelemetry]:
        try:
            key = self._get_telemetry_key(drone_id)
            data = self._redis.get(key)
            if data:
                telemetry = DroneTelemetry.model_validate_json(data)
                if telemetry.timestamp >= min_timestamp:
                    return telemetry
            return None
        except Exception as e:
            print(f"[Redis] Get telemetry error: {e}")
            return None

    def get_telemetry_batch(self, drone_ids: List[str], min_timestamp: int = 0) -> List[DroneTelemetry]:
        try:
            if not drone_ids:
                return []

            keys = [self._get_telemetry_key(d) for d in drone_ids]
            results = self._redis.mget(keys)

            drones = []
            for data in results:
                if data:
                    try:
                        telemetry = DroneTelemetry.model_validate_json(data)
                        if telemetry.timestamp >= min_timestamp:
                            drones.append(telemetry)
                    except Exception:
                        continue
            return drones
        except Exception as e:
            print(f"[Redis] Get telemetry batch error: {e}")
            return []

    def get_all_drones(self, min_timestamp: int = 0) -> List[DroneTelemetry]:
        try:
            results = self._get_all_drones_script()
            drones = []
            for data in results:
                try:
                    telemetry = DroneTelemetry.model_validate_json(data)
                    if telemetry.timestamp >= min_timestamp:
                        drones.append(telemetry)
                except Exception:
                    continue
            return drones
        except Exception as e:
            print(f"[Redis] Get all drones error: {e}")
            return []

    def get_all_geohashes(self) -> List[Dict]:
        try:
            results = self._get_all_geohashes_script()
            geohashes = []
            for row in results:
                if len(row) >= 7:
                    geohashes.append({
                        "drone_id": row[0],
                        "geohash": row[1],
                        "latitude": float(row[2]),
                        "longitude": float(row[3]),
                        "altitude": float(row[4]),
                        "timestamp": int(row[5]),
                        "version": int(row[6]),
                    })
            return geohashes
        except Exception as e:
            print(f"[Redis] Get all geohashes error: {e}")
            return []

    def get_drones_by_geohash(self, geohash_prefix: str, min_timestamp: int = 0) -> List[DroneTelemetry]:
        try:
            all_drones = self.get_all_drones(min_timestamp)
            return [
                d
                for d in all_drones
                if d.geohash and d.geohash.startswith(geohash_prefix)
            ]
        except Exception as e:
            print(f"[Redis] Get drones by geohash error: {e}")
            return []

    def get_neighbors(self, drone_id: str, radius_km: float = 1.0, min_timestamp: int = 0) -> List[DroneTelemetry]:
        try:
            telemetry = self.get_telemetry(drone_id, min_timestamp)
            if not telemetry:
                return []

            all_drones = self.get_all_drones(min_timestamp)
            neighbors = []

            for other in all_drones:
                if other.drone_id == drone_id:
                    continue

                distance = self._haversine_distance(
                    telemetry.latitude,
                    telemetry.longitude,
                    other.latitude,
                    other.longitude,
                )

                if distance <= radius_km:
                    neighbors.append(other)

            return neighbors
        except Exception as e:
            print(f"[Redis] Get neighbors error: {e}")
            return []

    def _haversine_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        import math

        R = 6371.0

        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    def store_command(self, command: DroneCommand) -> bool:
        try:
            command.timestamp = int(time.time() * 1000)
            version = self._get_next_version()

            data = command.model_dump_json()

            result = self._store_command_script(
                args=[
                    command.drone_id,
                    data,
                    str(command.timestamp),
                    str(version),
                    str(settings.command_ttl),
                ]
            )

            return result == 1
        except Exception as e:
            print(f"[Redis] Store command error: {e}")
            return False

    def store_command_batch(self, commands: List[DroneCommand]) -> int:
        if not commands:
            return 0

        pipe = self._redis.pipeline(transaction=False)

        for command in commands:
            command.timestamp = int(time.time() * 1000)
            version = self._get_next_version()

            data = command.model_dump_json()

            self._store_command_script(
                args=[
                    command.drone_id,
                    data,
                    str(command.timestamp),
                    str(version),
                    str(settings.command_ttl),
                ],
                client=pipe,
            )

        try:
            results = pipe.execute()
            return sum(1 for r in results if r == 1)
        except Exception as e:
            print(f"[Redis] Batch store command error: {e}")
            return 0

    def get_command(self, drone_id: str, min_timestamp: int = 0) -> Optional[DroneCommand]:
        try:
            key = self._get_command_key(drone_id)
            data = self._redis.get(key)
            if data:
                command = DroneCommand.model_validate_json(data)
                if command.timestamp >= min_timestamp:
                    return command
            return None
        except Exception as e:
            print(f"[Redis] Get command error: {e}")
            return None

    def get_command_batch(self, drone_ids: List[str], min_timestamp: int = 0) -> List[Optional[DroneCommand]]:
        try:
            if not drone_ids:
                return []

            keys = [self._get_command_key(d) for d in drone_ids]
            results = self._redis.mget(keys)

            commands = []
            for data in results:
                if data:
                    try:
                        command = DroneCommand.model_validate_json(data)
                        if command.timestamp >= min_timestamp:
                            commands.append(command)
                        else:
                            commands.append(None)
                    except Exception:
                        commands.append(None)
                else:
                    commands.append(None)
            return commands
        except Exception as e:
            print(f"[Redis] Get command batch error: {e}")
            return [None] * len(drone_ids)

    def subscribe_telemetry(self, callback):
        pubsub = self._redis.pubsub()
        pubsub.subscribe(self.telemetry_channel)

        try:
            for message in pubsub.listen():
                if message["type"] == "message":
                    try:
                        data = json.loads(message["data"])
                        callback(data)
                    except Exception as e:
                        print(f"[Redis] Telemetry subscribe error: {e}")
        finally:
            pubsub.unsubscribe()
            pubsub.close()

    def subscribe_commands(self, drone_id: str, callback):
        pubsub = self._redis.pubsub()
        channel = self._get_command_channel(drone_id)
        pubsub.subscribe(channel)

        try:
            for message in pubsub.listen():
                if message["type"] == "message":
                    try:
                        data = json.loads(message["data"])
                        callback(data)
                    except Exception as e:
                        print(f"[Redis] Command subscribe error: {e}")
        finally:
            pubsub.unsubscribe()
            pubsub.close()

    def set_mission(self, mission_data: Dict) -> bool:
        try:
            self._redis.set(self.mission_key, json.dumps(mission_data))
            self._redis.publish("swarm:mission:updates", json.dumps(mission_data))
            return True
        except Exception as e:
            print(f"[Redis] Set mission error: {e}")
            return False

    def get_mission(self) -> Optional[Dict]:
        try:
            data = self._redis.get(self.mission_key)
            if data:
                return json.loads(data)
            return None
        except Exception as e:
            print(f"[Redis] Get mission error: {e}")
            return None

    def get_online_count(self) -> int:
        try:
            return self._redis.scard(self.online_drones_key)
        except Exception:
            return 0

    def cleanup_offline_drones(self):
        try:
            drone_ids = self._redis.smembers(self.online_drones_key)
            if not drone_ids:
                return

            keys = [self._get_telemetry_key(d) for d in drone_ids]
            existing = self._redis.mget(keys)

            pipe = self._redis.pipeline(transaction=False)
            for i, data in enumerate(existing):
                if data is None:
                    pipe.srem(self.online_drones_key, drone_ids[i])

            if len(pipe) > 0:
                pipe.execute()
        except Exception as e:
            print(f"[Redis] Cleanup offline drones error: {e}")

    def get_pool_stats(self) -> Dict:
        return self._pool.get_pool_stats()
