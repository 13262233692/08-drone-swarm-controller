import os
import sys
import subprocess


def generate_proto():
    proto_dir = os.path.join(os.path.dirname(__file__), "proto")
    output_dir = os.path.join(os.path.dirname(__file__), "grpc_gen")

    os.makedirs(output_dir, exist_ok=True)

    proto_file = os.path.join(proto_dir, "drone_swarm.proto")

    if not os.path.exists(proto_file):
        print(f"Error: Proto file not found at {proto_file}")
        return False

    try:
        subprocess.check_call(
            [
                sys.executable,
                "-m",
                "grpc_tools.protoc",
                f"--proto_path={proto_dir}",
                f"--python_out={output_dir}",
                f"--grpc_python_out={output_dir}",
                proto_file,
            ]
        )
        print(f"Proto files generated successfully in {output_dir}")

        init_file = os.path.join(output_dir, "__init__.py")
        with open(init_file, "w") as f:
            f.write("import sys\n")
            f.write("import os\n")
            f.write("sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))\n")
            f.write("\n")
            f.write("from .drone_swarm_pb2 import *\n")
            f.write("from .drone_swarm_pb2_grpc import *\n")

        grpc_file = os.path.join(output_dir, "drone_swarm_pb2_grpc.py")
        if os.path.exists(grpc_file):
            with open(grpc_file, "r") as f:
                content = f.read()
            content = content.replace(
                "import drone_swarm_pb2 as drone__swarm__pb2",
                "from . import drone_swarm_pb2 as drone__swarm__pb2"
            )
            with open(grpc_file, "w") as f:
                f.write(content)

        return True
    except subprocess.CalledProcessError as e:
        print(f"Error generating proto files: {e}")
        return False
    except FileNotFoundError:
        print("Error: grpc_tools not found. Install it with: pip install grpcio-tools")
        return False


if __name__ == "__main__":
    generate_proto()
