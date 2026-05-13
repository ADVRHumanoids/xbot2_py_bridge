import os
import socket
import json, yaml
import numpy as np
from dataclasses import dataclass, fields, is_dataclass
from typing import get_type_hints, get_origin, get_args

@dataclass
class JointCommand:
    q: np.ndarray
    dq: np.ndarray
    tau: np.ndarray
    k: np.ndarray
    d: np.ndarray

@dataclass
class RobotCommand:
    joint_command: JointCommand

@dataclass
class JointState:
    q: np.ndarray
    dq: np.ndarray
    tau: np.ndarray
    k: np.ndarray
    d: np.ndarray
    qref: np.ndarray
    vref: np.ndarray
    tauref: np.ndarray

@dataclass
class ImuState:
    quat_w: np.ndarray
    lin_acc_b: np.ndarray
    ang_vel_b: np.ndarray

@dataclass
class RobotState:
    time: float
    joints: JointState
    imus: dict[str,ImuState] 


def _to_serializable(obj):
    """Recursively convert a dataclass tree to a plain dict suitable for yaml.dump."""
    if isinstance(obj, np.ndarray):
        return obj.flatten().tolist()
    if is_dataclass(obj) and not isinstance(obj, type):
        return {f.name: _to_serializable(getattr(obj, f.name)) for f in fields(obj)}
    if isinstance(obj, dict):
        return {k: _to_serializable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return type(obj)(_to_serializable(v) for v in obj)
    return obj


def state_to_dict(state: RobotState) -> str:
    return _to_serializable(state)

def _from_dict(data, cls):
    """Recursively reconstruct a dataclass tree from a plain dict."""
    if is_dataclass(cls) and not isinstance(cls, type(None)):
        hints = get_type_hints(cls)
        return cls(**{f.name: _from_dict(data[f.name], hints[f.name]) for f in fields(cls)})
    if cls is np.ndarray:
        return np.array(data)
    if cls in (float, int, str, bool):
        return cls(data)
    origin = get_origin(cls)
    if origin is dict:
        _, val_type = get_args(cls)
        return {k: _from_dict(v, val_type) for k, v in data.items()}
    if origin in (list, tuple):
        (item_type,) = get_args(cls)
        return origin(_from_dict(v, item_type) for v in data)
    return data


def command_from_dict(data: dict) -> RobotCommand:
    return _from_dict(data, RobotCommand)

class BridgeServer:

    def __init__(self, 
                 name: str, 
                 socket_path: str = None,
                 joint_names: list[str] = [],
                 imu_names: list[str] = [],
                 recv_buffer_size: int = 4096
                 ):
        
        # Initialize the server socket path
        self.server_socket_path = socket_path if socket_path else f'/tmp/{name}_server.sock'
        self.recv_buffer_size = recv_buffer_size

        # Ensure the directory for the socket exists and remove any existing socket file
        os.makedirs(os.path.dirname(self.server_socket_path), exist_ok=True)
        if os.path.exists(self.server_socket_path):
            os.unlink(self.server_socket_path)

        # Create a UNIX domain socket and bind it to the specified path
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.bind(self.server_socket_path)
        self.sock.setblocking(False)

        # Set permissions for the socket file to allow read/write access for all users
        # Useful when working with docker containers or multiple users
        os.chmod(self.server_socket_path, 0o777)

        # Store the joint and IMU names for later use
        self.joint_names = joint_names
        self.imu_names = imu_names

        # Print info
        print(f"[BridgeServer] Initialized on socket {self.server_socket_path}")
        print(f"[BridgeServer] Joint names: {self.joint_names}")
        print(f"[BridgeServer] IMU names: {self.imu_names}")

        # Connected clients
        self.clients = set()

        # Useful stats
        self.max_bytes_received = 0
        self.max_bytes_sent = 0


    def receive(self) -> RobotCommand | None:
        """Receive a command from any client."""
        try:
            # Receive data from any client (non-blocking)
            data, addr = self.sock.recvfrom(self.recv_buffer_size)
            # Update stats
            self.max_bytes_received = max(self.max_bytes_received, len(data))
             # Track the client address
            self.clients.add(addr) 
            # Parse the JSON data into a dict
            data = json.loads(data)
        except BlockingIOError:
            return None
        
        # Handle the received data based on its type        
        data_type = data['type']

        if data_type == 'discovery':
            print(f"[BridgeServer] Discovery request received from {addr}. Sending joint and IMU info.")
            response = {'type': 'discovery'}
            response['joint_names'] = self.joint_names
            response['imu_sensors'] = list(self.imu_names)
            try:
                self.sock.sendto(json.dumps(response).encode('utf-8'), addr)
            except Exception as e:
                print(f"[BridgeServer] Error sending discovery response to {addr}: {e}")
            self.clients.add(addr)
            print(f"[BridgeServer] Client at {addr} connected. Sent {len(self.joint_names)} joints, {len(self.imu_names)} IMUs.")

        elif data_type == 'control':
            return command_from_dict(data)
        else:
            print(f"[BridgeServer] Unknown data type received: {data_type}")

    
    def send_state(self, state: RobotState):
        """Send the current state to all connected clients."""
        state_dict = state_to_dict(state)
        state_dict['type'] = 'state'
        state_message = json.dumps(state_dict, separators=(',', ':')).encode('utf-8')
        # Update stats
        self.max_bytes_sent = max(self.max_bytes_sent, len(state_message))
        sockets_to_remove = []
        for client in self.clients:
            try:
                self.sock.sendto(state_message, client)
            except ConnectionRefusedError:
                print(f"[BridgeServer] Client at {client} disconnected.")
                sockets_to_remove.append(client)
            except Exception as e:
                print(f"[BridgeServer] Error sending state to {client}: {e}")
        for socket in sockets_to_remove:
            self.clients.remove(socket)

