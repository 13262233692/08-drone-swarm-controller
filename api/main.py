from fastapi import FastAPI, HTTPException
from typing import List
from datetime import datetime

from config import settings
from redis_manager.state_manager import RedisStateManager
from models.drone import (
    SwarmMission,
    MissionResponse,
    DroneStatus,
    DroneTelemetry,
    DroneCommand,
)

app = FastAPI(
    title=settings.app_name,
    version=settings.app_version,
    description="百架级自动驾驶无人机编队协同控制中枢服务",
)

redis_manager = RedisStateManager()


@app.get("/")
async def root():
    return {
        "service": settings.app_name,
        "version": settings.app_version,
        "status": "running",
    }


@app.get("/health")
async def health_check():
    try:
        online_count = redis_manager.get_online_count()
        return {
            "status": "healthy",
            "online_drones": online_count,
            "timestamp": datetime.now().isoformat(),
        }
    except Exception as e:
        raise HTTPException(status_code=503, detail=f"Service unhealthy: {str(e)}")


@app.post("/missions", response_model=MissionResponse)
async def create_mission(mission: SwarmMission):
    try:
        mission_data = mission.model_dump()
        success = redis_manager.set_mission(mission_data)

        if success:
            return MissionResponse(
                mission_id=mission.mission_id,
                status="created",
                message=f"Mission {mission.name} created successfully",
            )
        else:
            raise HTTPException(status_code=500, detail="Failed to create mission")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/missions/current")
async def get_current_mission():
    try:
        mission = redis_manager.get_mission()
        if mission:
            return mission
        else:
            return {"mission_id": None, "status": "no_active_mission"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/drones", response_model=List[DroneStatus])
async def list_drones():
    try:
        drones = redis_manager.get_all_drones()
        status_list = []
        for drone in drones:
            status_list.append(
                DroneStatus(
                    drone_id=drone.drone_id,
                    latitude=drone.latitude,
                    longitude=drone.longitude,
                    altitude=drone.altitude,
                    battery_level=drone.battery_level,
                    is_online=True,
                    last_update=datetime.fromtimestamp(drone.timestamp / 1000),
                )
            )
        return status_list
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/drones/{drone_id}", response_model=DroneTelemetry)
async def get_drone(drone_id: str):
    try:
        telemetry = redis_manager.get_telemetry(drone_id)
        if telemetry:
            return telemetry
        else:
            raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/drones/{drone_id}/command", response_model=DroneCommand)
async def get_drone_command(drone_id: str):
    try:
        command = redis_manager.get_command(drone_id)
        if command:
            return command
        else:
            raise HTTPException(
                status_code=404, detail=f"No command found for drone {drone_id}"
            )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/drones/{drone_id}/neighbors")
async def get_drone_neighbors(drone_id: str, radius_km: float = 1.0):
    try:
        neighbors = redis_manager.get_neighbors(drone_id, radius_km)
        return {
            "drone_id": drone_id,
            "radius_km": radius_km,
            "neighbor_count": len(neighbors),
            "neighbors": [
                {
                    "drone_id": n.drone_id,
                    "latitude": n.latitude,
                    "longitude": n.longitude,
                    "altitude": n.altitude,
                    "geohash": n.geohash,
                }
                for n in neighbors
            ],
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/swarm/stats")
async def get_swarm_stats():
    try:
        drones = redis_manager.get_all_drones()
        count = len(drones)

        if count == 0:
            return {
                "total_drones": 0,
                "avg_battery": 0,
                "avg_altitude": 0,
                "center_latitude": 0,
                "center_longitude": 0,
            }

        total_battery = sum(d.battery_level for d in drones)
        total_altitude = sum(d.altitude for d in drones)
        avg_lat = sum(d.latitude for d in drones) / count
        avg_lon = sum(d.longitude for d in drones) / count

        return {
            "total_drones": count,
            "avg_battery": total_battery / count,
            "avg_altitude": total_altitude / count,
            "center_latitude": avg_lat,
            "center_longitude": avg_lon,
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/drones/{drone_id}/telemetry")
async def receive_telemetry(drone_id: str, telemetry: DroneTelemetry):
    try:
        telemetry.drone_id = drone_id
        success = redis_manager.update_telemetry(telemetry)
        if success:
            return {"status": "success", "message": "Telemetry received"}
        else:
            raise HTTPException(status_code=500, detail="Failed to process telemetry")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
