import sys
import os
import threading
import signal

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import settings
from generate_proto import generate_proto


def main():
    print("=" * 60)
    print(f"  {settings.app_name} v{settings.app_version}")
    print("  百架级自动驾驶无人机编队协同控制中枢服务")
    print("=" * 60)
    print()

    print("[Init] Generating protobuf files...")
    if not generate_proto():
        print("[Warning] Failed to generate proto files, attempting to continue...")
    print()

    from grpc_server import GrpcServer
    from boids import BoidsProcessor
    import uvicorn
    from api import app

    grpc_server = GrpcServer()
    boids_processor = BoidsProcessor()

    def start_grpc_server():
        grpc_server.start()
        grpc_server.wait_for_termination()

    grpc_thread = threading.Thread(target=start_grpc_server, daemon=True)
    grpc_thread.start()

    boids_processor.start()

    def shutdown_handler(signum, frame):
        print("\n[Shutdown] Received shutdown signal...")
        boids_processor.stop()
        grpc_server.stop()
        print("[Shutdown] Goodbye!")
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    print()
    print(f"[REST] API server starting on http://{settings.rest_host}:{settings.rest_port}")
    print(f"[REST] API docs: http://{settings.rest_host}:{settings.rest_port}/docs")
    print()

    uvicorn.run(
        app,
        host=settings.rest_host,
        port=settings.rest_port,
        log_level="info",
    )


if __name__ == "__main__":
    main()
