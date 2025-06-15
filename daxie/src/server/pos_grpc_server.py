import threading
import time
from collections import deque
from concurrent import futures

import grpc

# Generated gRPC code (you'll need to generate these)
from . import pose_telemetry_pb2, pose_telemetry_pb2_grpc


class PoseTelemetryService(pose_telemetry_pb2_grpc.PoseTelemetryServicer):
    def __init__(self):
        self._latest_pose = None  # single shared slot
        self._pose_lock = threading.Lock()  # protect access to it
        self._new_pose_evt = threading.Event()  # signal consumer

        self.debug = False
        self.start_time = time.perf_counter()
        self.pose_count = 0

    def set_debug(self, debug):
        self.debug = debug

    def StreamPoses(self, request_iterator, context):
        """Handle streaming poses from client"""
        print("Device connected to gRPC streaming")
        try:
            for pose_data in request_iterator:
                self.pose_count += 1

                # Extract data
                translation = list(pose_data.translation)
                rotation = list(pose_data.rotation)
                is_open = pose_data.gripper_open
                stream_on = pose_data.start_stream
                precision_mode = pose_data.precision_mode

                # Reorder quaternion from ARCore [qx,qy,qz,qw] to [qw,qx,qy,qz]
                if len(rotation) == 4:
                    wxyz = [rotation[3], rotation[1], rotation[2], rotation[0]]
                else:
                    wxyz = rotation

                # Reorder position if needed (keeping your original logic)
                if len(translation) == 3:
                    pos = [translation[1], translation[2], translation[0]]
                else:
                    pos = translation

                # Store in queue for processing
                pose_data_processed = {
                    "position": pos,
                    "rotation": wxyz,
                    "gripper_open": is_open,
                    "switch": stream_on,
                    "precision": precision_mode,
                }

                # atomically overwrite the shared slot
                with self._pose_lock:
                    self._latest_pose = pose_data_processed
                    # wake up any waiting consumer
                    self._new_pose_evt.set()

            return pose_telemetry_pb2.PoseResponse(
                success=True, message="Poses received successfully"
            )

        except Exception as e:
            print(f"Error in StreamPoses: {e}")
            return pose_telemetry_pb2.PoseResponse(
                success=False, message=f"Error: {str(e)}"
            )

    def get_latest_pose(self, block: bool = True, timeout: float = None):
        """
        Wait for a new pose (optionally), then return the most recent one.
        If block=False, returns immediately (or None if none seen yet).
        """
        if block:
            got = self._new_pose_evt.wait(timeout)
            if not got:
                return None
            # clear the flag so next wait blocks until another set()
            self._new_pose_evt.clear()

        with self._pose_lock:
            return self._latest_pose


def start_grpc_server(host="0.0.0.0", port=8765, debug=False):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    pose_service = PoseTelemetryService()
    pose_telemetry_pb2_grpc.add_PoseTelemetryServicer_to_server(pose_service, server)
    server.add_insecure_port(f"{host}:{port}")
    server.start()

    print("#### gRPC server started ####")
    return server, pose_service


def main():
    # Start server
    server, pose_service = start_grpc_server()
    hz = 0
    hz_queue = deque(maxlen=350 * 50)

    try:
        # Process poses in main thread
        # pose_generator = pose_service.get_latest_pose(block=True)

        while True:
            try:
                ts = time.perf_counter()
                pose_data = pose_service.get_latest_pose(block=True)
                hz_grpc = time.perf_counter() - ts
                hz_queue.append(hz_grpc)
                # Process your pose data here
                hz = 0.99 * hz + (1 - 0.99) * (hz_grpc)
                # print(1 / hz_grpc)
                print(
                    f"Processing pose at {1 / hz:.2f} hz\tmin: {(1 / min(hz_queue)):.2f} hz"
                )

            except StopIteration:
                break
            except KeyboardInterrupt:
                break

    except KeyboardInterrupt:
        print("Stopping server...")
    finally:
        server.stop(0)


if __name__ == "__main__":
    main()
