import time
import multiprocessing
from typing import List
from config import settings
from redis_manager.state_manager import RedisStateManager
from boids.algorithm import BoidsAlgorithm
from models.drone import DroneCommand


class BoidsProcessor:
    def __init__(self):
        self.redis_manager = RedisStateManager()
        self.boids = BoidsAlgorithm()
        self.running = False
        self.process = None

    def start(self):
        if self.running:
            return

        self.running = True
        self.process = multiprocessing.Process(target=self._run, daemon=True)
        self.process.start()
        print(f"[Boids] Process started, interval: {settings.boids_process_interval}s")

    def stop(self):
        self.running = False
        if self.process:
            self.process.terminate()
            self.process.join()
            print("[Boids] Process stopped")

    def _run(self):
        print("[Boids] Processing loop started")
        while self.running:
            try:
                start_time = time.time()

                self.redis_manager.cleanup_offline_drones()

                drones = self.redis_manager.get_all_drones()

                if len(drones) > 0:
                    commands = self.boids.compute_all_commands(drones)
                    self._publish_commands(commands)

                elapsed = time.time() - start_time
                sleep_time = max(0, settings.boids_process_interval - elapsed)
                time.sleep(sleep_time)

            except Exception as e:
                print(f"[Boids] Processing error: {e}")
                time.sleep(settings.boids_process_interval)

    def _publish_commands(self, commands: List[DroneCommand]):
        for command in commands:
            self.redis_manager.store_command(command)

        if len(commands) > 0:
            print(f"[Boids] Published {len(commands)} commands")
