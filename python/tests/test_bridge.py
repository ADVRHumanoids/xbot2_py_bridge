"""Integration test: BridgeServer + a dummy client thread."""

import os
import socket
import tempfile
import threading
import time

import numpy as np
import pytest
import yaml

from xbot2_py_bridge.bridge_server import (
    BridgeServer,
    ImuState,
    JointState,
    RobotState,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

JOINT_NAMES = ["j1", "j2", "j3"]
IMU_NAMES = ["imu_link"]
N = len(JOINT_NAMES)

POLL_INTERVAL = 0.005   # seconds between non-blocking receive() calls
TIMEOUT = 5.0           # seconds before a wait loop gives up


def make_state() -> RobotState:
    return RobotState(
        time=1.23,
        joints=JointState(
            q=np.array([0.1, 0.2, 0.3]),
            dq=np.zeros(N),
            tau=np.ones(N) * 0.5,
            k=np.zeros(N),
            d=np.zeros(N),
            qref=np.array([0.1, 0.2, 0.3]),
            vref=np.zeros(N),
            tauref=np.zeros(N),
        ),
        imus={
            "imu_link": ImuState(
                quat_w=np.array([0.0, 0.0, 0.0, 1.0]),
                lin_acc_b=np.array([0.0, 0.0, 9.81]),
                ang_vel_b=np.zeros(3),
            )
        },
    )


# ---------------------------------------------------------------------------
# Dummy client
# ---------------------------------------------------------------------------

class DummyClient:
    """
    Runs in a background thread.  Protocol:
      1. Wait for a 'state' broadcast from the server.
      2. Send a 'discovery' request; wait for the discovery response.
      3. Send N_CONTROL 'control' messages.
    """

    N_CONTROL = 3

    def __init__(self, server_path: str, client_path: str):
        self.server_path = server_path
        self.client_path = client_path

        self.received_state: dict | None = None
        self.discovery_response: dict | None = None
        self.error: Exception | None = None

        # Events used to synchronise with the test thread
        self.state_received = threading.Event()
        self.discovery_done = threading.Event()
        self.control_sent = threading.Event()

        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._sock.bind(client_path)
        self._sock.settimeout(TIMEOUT)

    def run(self):
        try:
            # 1. Wait for a state message (server broadcasts to all known clients;
            #    we must register first by appearing in self.clients via a prior
            #    recvfrom on the server side — so we send a tiny "ping" to get
            #    added to clients, then listen for the state).
            #
            #    Actually the server only adds a client after receiving from it,
            #    so we send discovery first (step 2) which also registers us, then
            #    wait for the state.  The test loop keeps calling send_state() so
            #    we will receive one shortly after discovery.

            # 2. Send discovery
            discovery_msg = yaml.dump({"type": "discovery"}).encode()
            self._sock.sendto(discovery_msg, self.server_path)

            # Wait for discovery response
            data, _ = self._sock.recvfrom(4096)
            self.discovery_response = yaml.safe_load(data.decode())
            self.discovery_done.set()

            # 3. Wait for a state broadcast
            data, _ = self._sock.recvfrom(65536)
            self.received_state = yaml.safe_load(data.decode())
            self.state_received.set()

            # 4. Send N_CONTROL control messages
            for i in range(self.N_CONTROL):
                ctrl = {
                    "type": "control",
                    "joint_command": {
                        "q":   [float(i)] * N,
                        "dq":  [0.0] * N,
                        "tau": [0.1 * i] * N,
                        "k":   [0.0] * N,
                        "d":   [0.0] * N,
                    },
                }
                self._sock.sendto(yaml.dump(ctrl).encode(), self.server_path)

            self.control_sent.set()

        except Exception as exc:
            self.error = exc
            # Set all events so the test doesn't hang
            self.state_received.set()
            self.discovery_done.set()
            self.control_sent.set()
        finally:
            self._sock.close()
            if os.path.exists(self.client_path):
                os.unlink(self.client_path)

    def start(self):
        self._thread = threading.Thread(target=self.run, daemon=True)
        self._thread.start()

    def join(self, timeout=TIMEOUT):
        self._thread.join(timeout=timeout)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture()
def tmp_sockets(tmp_path):
    server_path = str(tmp_path / "server.sock")
    client_path = str(tmp_path / "client.sock")
    yield server_path, client_path


@pytest.fixture()
def server(tmp_sockets):
    server_path, _ = tmp_sockets
    srv = BridgeServer(
        name="test",
        socket_path=server_path,   # already formatted — no {name} placeholder
        joint_names=JOINT_NAMES,
        imu_names=IMU_NAMES,
    )
    yield srv
    srv.sock.close()
    if os.path.exists(server_path):
        os.unlink(server_path)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_discovery_and_state_and_control(tmp_sockets, server):
    """Full round-trip: discovery → state broadcast → control commands."""

    server_path, client_path = tmp_sockets

    client = DummyClient(server_path=server_path, client_path=client_path)
    client.start()

    # Drive the server: poll receive() until discovery is handled, then keep
    # sending state until the client has received one, then collect control msgs.

    deadline = time.monotonic() + TIMEOUT
    collected_commands = []

    while time.monotonic() < deadline:
        cmd = server.receive()
        if cmd is not None:
            collected_commands.append(cmd)

        # Once the client has registered (discovery), broadcast state
        if server.clients:
            server.send_state(make_state())

        # Stop driving once the client is done
        if client.control_sent.is_set():
            # Drain any remaining control messages
            for _ in range(DummyClient.N_CONTROL * 2):
                cmd = server.receive()
                if cmd is not None:
                    collected_commands.append(cmd)
            break

        time.sleep(POLL_INTERVAL)

    client.join()

    assert client.error is None, f"Client raised: {client.error}"

    # --- discovery response ---
    assert client.discovery_done.is_set(), "Client never received discovery response"
    resp = client.discovery_response
    assert resp["type"] == "discovery"
    assert resp["joint_names"] == JOINT_NAMES
    assert resp["imu_sensors"] == IMU_NAMES

    # --- state broadcast ---
    assert client.state_received.is_set(), "Client never received a state"
    state = client.received_state
    assert state["type"] == "state"
    assert pytest.approx(state["time"], abs=1e-9) == 1.23
    assert pytest.approx(state["joints"]["q"], abs=1e-6) == [0.1, 0.2, 0.3]
    assert "imu_link" in state["imus"]

    # --- control commands ---
    assert client.control_sent.is_set(), "Client never finished sending control"
    assert len(collected_commands) == DummyClient.N_CONTROL, (
        f"Expected {DummyClient.N_CONTROL} commands, got {len(collected_commands)}"
    )
    for i, cmd in enumerate(collected_commands):
        np.testing.assert_array_almost_equal(cmd.joint_command.q, [float(i)] * N)
        np.testing.assert_array_almost_equal(cmd.joint_command.tau, [0.1 * i] * N)
