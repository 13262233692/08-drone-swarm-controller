from fastapi import FastAPI, HTTPException
from typing import List
from datetime import datetime

from config import settings
from redis_manager.state_manager import RedisStateManager
from geofencing.manager import GeoFenceManager
from models.drone import (
    SwarmMission,
    MissionResponse,
    DroneStatus,
    DroneTelemetry,
    DroneCommand,
    GeoFence,
    FenceRepulsionResult,
    FenceViolation,
)

app = FastAPI(
    title=settings.app_name,
    version=settings.app_version,
    description="百架级自动驾驶无人机编队协同控制中枢服务",
)

redis_manager = RedisStateManager()
fence_manager = GeoFenceManager()


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


@app.post("/geofences", response_model=dict)
async def create_geofence(fence: GeoFence):
    try:
        if len(fence.polygon) < 3:
            raise HTTPException(
                status_code=400,
                detail="Polygon must have at least 3 points",
            )
        success = fence_manager.add_fence(fence)
        if success:
            return {
                "status": "success",
                "fence_id": fence.fence_id,
                "message": f"Geo-fence '{fence.name}' created successfully",
            }
        else:
            raise HTTPException(status_code=500, detail="Failed to create geo-fence")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/geofences", response_model=List[GeoFence])
async def list_geofences():
    try:
        return fence_manager.get_all_fences()
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/geofences/{fence_id}", response_model=GeoFence)
async def get_geofence(fence_id: str):
    try:
        fence = fence_manager.get_fence(fence_id)
        if fence:
            return fence
        else:
            raise HTTPException(status_code=404, detail=f"Geo-fence {fence_id} not found")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.put("/geofences/{fence_id}", response_model=dict)
async def update_geofence(fence_id: str, fence: GeoFence):
    try:
        if fence_id != fence.fence_id:
            raise HTTPException(
                status_code=400,
                detail="Fence ID in path does not match body",
            )
        if len(fence.polygon) < 3:
            raise HTTPException(
                status_code=400,
                detail="Polygon must have at least 3 points",
            )
        success = fence_manager.update_fence(fence)
        if success:
            return {
                "status": "success",
                "fence_id": fence.fence_id,
                "message": f"Geo-fence '{fence.name}' updated successfully",
            }
        else:
            raise HTTPException(status_code=500, detail="Failed to update geo-fence")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.delete("/geofences/{fence_id}", response_model=dict)
async def delete_geofence(fence_id: str):
    try:
        success = fence_manager.remove_fence(fence_id)
        if success:
            return {
                "status": "success",
                "message": f"Geo-fence {fence_id} deleted successfully",
            }
        else:
            raise HTTPException(status_code=500, detail="Failed to delete geo-fence")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.delete("/geofences", response_model=dict)
async def clear_all_geofences():
    try:
        success = fence_manager.clear_all_fences()
        if success:
            return {
                "status": "success",
                "message": "All geo-fences cleared successfully",
            }
        else:
            raise HTTPException(status_code=500, detail="Failed to clear geo-fences")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/geofences/violations/recent", response_model=List[dict])
async def get_recent_violations(limit: int = 100):
    try:
        return fence_manager.get_violations(limit=limit)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/drones/{drone_id}/fence-status", response_model=FenceRepulsionResult)
async def get_drone_fence_status(drone_id: str):
    try:
        telemetry = redis_manager.get_telemetry(drone_id)
        if not telemetry:
            raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
        result = fence_manager.evaluate_drone(telemetry)
        return result
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/geofences/reload", response_model=dict)
async def reload_geofences_from_redis():
    try:
        fence_manager.reload_from_redis()
        count = len(fence_manager.get_all_fences())
        return {
            "status": "success",
            "message": f"Reloaded {count} geo-fences from Redis",
            "active_fences": count,
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
