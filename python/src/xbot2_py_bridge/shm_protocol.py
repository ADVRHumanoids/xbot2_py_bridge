import os
import secrets
import struct
import time
from dataclasses import dataclass
from multiprocessing import shared_memory
from types import SimpleNamespace

import numpy as np


MAGIC = 0x5842504253484D31  # XBPBSHM1
VERSION = 1
HEADER_FORMAT = "<16Q"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

# Keep this header ABI in sync with cpp/src/py_bridge_hal.h. All words are
# uint64_t so C++ can treat the header as a simple fixed-size array.
IDX_MAGIC = 0
IDX_VERSION = 1
IDX_HEADER_SIZE = 2
IDX_TOTAL_SIZE = 3
IDX_NJOINTS = 4
IDX_NIMUS = 5
IDX_STATE_SEQ = 6
IDX_COMMAND_SEQ = 7
IDX_SERVER_SESSION_ID = 8
IDX_CLIENT_SESSION_ID = 9
IDX_COMMAND_VALID = 10
IDX_STATE_OFFSET = 11
IDX_COMMAND_OFFSET = 12
IDX_COMMAND_STAMP_NS = 13
IDX_RESERVED0 = 14
IDX_RESERVED1 = 15

STATE_FIELDS = ("q", "dq", "tau", "k", "d", "qref", "vref", "tauref")
COMMAND_FIELDS = ("q", "dq", "tau", "k", "d")
IMU_FIELDS = (("quat_w", 4), ("lin_acc_b", 3), ("ang_vel_b", 3))


def _align(value: int, alignment: int = 64) -> int:
    return ((value + alignment - 1) // alignment) * alignment


@dataclass(frozen=True)
class ShmLayout:
    njoints: int
    nimus: int
    state_offset: int
    command_offset: int
    total_size: int

    @property
    def state_doubles(self) -> int:
        return 1 + len(STATE_FIELDS) * self.njoints + sum(n for _, n in IMU_FIELDS) * self.nimus

    @property
    def command_doubles(self) -> int:
        return len(COMMAND_FIELDS) * self.njoints


def make_layout(njoints: int, nimus: int) -> ShmLayout:
    # The state frame and command frame are separated by cache-line alignment so
    # the two writers do not touch adjacent memory unnecessarily.
    state_offset = HEADER_SIZE
    state_bytes = (1 + len(STATE_FIELDS) * njoints + sum(n for _, n in IMU_FIELDS) * nimus) * 8
    command_offset = _align(state_offset + state_bytes)
    command_bytes = len(COMMAND_FIELDS) * njoints * 8
    total_size = _align(command_offset + command_bytes)
    return ShmLayout(njoints, nimus, state_offset, command_offset, total_size)


class AttrDict(dict):
    def __getattr__(self, key):
        try:
            return self[key]
        except KeyError as exc:
            raise AttributeError(key) from exc


def _array(buffer, offset: int, count: int) -> np.ndarray:
    return np.ndarray((count,), dtype=np.float64, buffer=buffer, offset=offset)


def _make_joint_namespace(buffer, offset: int, names: tuple[str, ...], njoints: int):
    arrays = {}
    pos = offset
    for field in names:
        arrays[field] = _array(buffer, pos, njoints)
        pos += njoints * 8
    return SimpleNamespace(**arrays), pos


class ShmBridgeMemory:
    def __init__(self, name: str, joint_names: list[str], imu_names: list[str]):
        self.joint_names = list(joint_names)
        self.imu_names = list(imu_names)
        self.layout = make_layout(len(self.joint_names), len(self.imu_names))
        self.server_session_id = secrets.randbits(64) or 1
        self.last_command_seq = 0
        self.active_client_session_id = 0

        # The shm name includes pid and entropy so a new simulator session never
        # attaches to stale command bytes from a previous process.
        shm_name = f"xbot2-pybridge-{name}-{os.getpid()}-{secrets.token_hex(8)}"
        self.shm = shared_memory.SharedMemory(name=shm_name, create=True, size=self.layout.total_size)
        self.shm.buf[:] = b"\x00" * self.layout.total_size

        self._set_header(
            MAGIC,
            VERSION,
            HEADER_SIZE,
            self.layout.total_size,
            self.layout.njoints,
            self.layout.nimus,
            0,
            0,
            self.server_session_id,
            0,
            0,
            self.layout.state_offset,
            self.layout.command_offset,
            0,
            0,
            0,
        )
        self._make_views()

    @property
    def name(self) -> str:
        return self.shm.name

    def _header(self) -> tuple[int, ...]:
        return struct.unpack_from(HEADER_FORMAT, self.shm.buf, 0)

    def _set_header(self, *values: int):
        struct.pack_into(HEADER_FORMAT, self.shm.buf, 0, *values)

    def _set_header_field(self, index: int, value: int):
        struct.pack_into("<Q", self.shm.buf, index * 8, int(value) & 0xFFFFFFFFFFFFFFFF)

    def _get_header_field(self, index: int) -> int:
        return struct.unpack_from("<Q", self.shm.buf, index * 8)[0]

    def _make_views(self):
        # Expose NumPy views directly over the shared-memory buffer. Simulators
        # can write state arrays without serialization or intermediate copies.
        state_pos = self.layout.state_offset
        self.state_time = _array(self.shm.buf, state_pos, 1)
        state_pos += 8
        state_joints, state_pos = _make_joint_namespace(
            self.shm.buf, state_pos, STATE_FIELDS, self.layout.njoints
        )
        imus = AttrDict()
        for imu_name in self.imu_names:
            imu_arrays = {}
            for field, count in IMU_FIELDS:
                imu_arrays[field] = _array(self.shm.buf, state_pos, count)
                state_pos += count * 8
            imus[imu_name] = SimpleNamespace(**imu_arrays)
        self.state = SimpleNamespace(time=self.state_time, joints=state_joints, imus=imus)

        command_joints, _ = _make_joint_namespace(
            self.shm.buf, self.layout.command_offset, COMMAND_FIELDS, self.layout.njoints
        )
        self.command = SimpleNamespace(joints=command_joints)

    def discovery_payload(self) -> dict:
        return {
            "protocol": "xbot2-shm-v1",
            "shm_name": self.name,
            "server_session_id": self.server_session_id,
            "layout": {
                "magic": MAGIC,
                "version": VERSION,
                "header_size": HEADER_SIZE,
                "total_size": self.layout.total_size,
                "state_offset": self.layout.state_offset,
                "command_offset": self.layout.command_offset,
                "njoints": self.layout.njoints,
                "nimus": self.layout.nimus,
            },
        }

    def invalidate_command(self):
        # A new discovery/reconnect starts with no accepted command; the C++
        # side must publish a fresh full frame with its current client session.
        self.last_command_seq = 0
        self.active_client_session_id = 0
        self._set_header_field(IDX_CLIENT_SESSION_ID, 0)
        self._set_header_field(IDX_COMMAND_VALID, 0)
        self._set_header_field(IDX_COMMAND_SEQ, 0)
        self._set_header_field(IDX_COMMAND_STAMP_NS, 0)

    def commit_state(self, sim_time: float):
        seq = self._get_header_field(IDX_STATE_SEQ)
        if seq & 1:
            seq += 1

        # Seqlock publish: odd marks an in-progress write; even marks a stable
        # state frame for C++ readers.
        self._set_header_field(IDX_STATE_SEQ, seq + 1)
        self.state_time[0] = sim_time
        self._set_header_field(IDX_STATE_SEQ, seq + 2)

    def has_new_command(self, freshness_sec: float) -> bool:
        seq1 = self._get_header_field(IDX_COMMAND_SEQ)

        # Reject empty, in-progress, or already-consumed command frames before
        # looking at any payload values.
        if seq1 == 0 or (seq1 & 1) or seq1 <= self.last_command_seq:
            return False

        header = self._header()
        seq2 = header[IDX_COMMAND_SEQ]
        if seq1 != seq2 or (seq2 & 1):
            return False

        if header[IDX_MAGIC] != MAGIC or header[IDX_VERSION] != VERSION:
            return False
        if header[IDX_SERVER_SESSION_ID] != self.server_session_id:
            return False
        if header[IDX_COMMAND_VALID] == 0:
            return False

        # Lock onto the first nonzero client session for this discovery cycle;
        # later commands from any other client/session are stale or unrelated.
        client_session = header[IDX_CLIENT_SESSION_ID]
        if client_session == 0:
            return False
        if self.active_client_session_id == 0:
            self.active_client_session_id = client_session
        elif client_session != self.active_client_session_id:
            return False

        stamp_ns = header[IDX_COMMAND_STAMP_NS]
        now_ns = time.monotonic_ns()

        # Message-like freshness for shm: even if bytes remain mapped, old
        # command frames expire and are not replayed into a new simulator tick.
        if stamp_ns == 0 or now_ns - stamp_ns > int(freshness_sec * 1e9):
            return False

        self.last_command_seq = seq2
        return True

    def close(self):
        self.state = None
        self.command = None
        self.state_time = None
        self.state_joints = None
        try:
            self.shm.close()
        except BufferError:
            pass
        try:
            self.shm.unlink()
        except FileNotFoundError:
            pass
