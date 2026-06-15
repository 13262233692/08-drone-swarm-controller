from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    app_name: str = "drone-swarm-controller"
    app_version: str = "1.0.0"

    rest_host: str = "0.0.0.0"
    rest_port: int = 8000

    grpc_host: str = "0.0.0.0"
    grpc_port: int = 50051

    redis_host: str = "localhost"
    redis_port: int = 6379
    redis_db: int = 0
    redis_password: Optional[str] = None

    geohash_precision: int = 8
    telemetry_ttl: int = 30
    command_ttl: int = 10

    boids_separation_distance: float = 50.0
    boids_alignment_distance: float = 100.0
    boids_cohesion_distance: float = 150.0
    boids_separation_weight: float = 1.5
    boids_alignment_weight: float = 1.0
    boids_cohesion_weight: float = 1.0
    boids_max_speed: float = 10.0
    boids_max_force: float = 0.1

    boids_process_interval: float = 0.05

    class Config:
        env_file = ".env"


settings = Settings()
