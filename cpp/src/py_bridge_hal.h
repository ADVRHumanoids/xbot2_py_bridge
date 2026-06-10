#ifndef XBOT2_PyBridge_HAL_H
#define XBOT2_PyBridge_HAL_H

#include <xbot2/hal/device.h>
#include <xbot2/hal/dev_joint.h>
#include <xbot2/hal/dev_imu.h>
#include <xbot2/ipc/pipe.h>
#include <sys/un.h>
#include <array>
#include <cstddef>
#include <cstdint>


namespace XBot {
namespace Hal {

namespace PyBridgeShm {

// Keep this ABI in sync with python/src/xbot2_py_bridge/shm_protocol.py.
// The hot path is intentionally just a fixed header followed by packed
// float64 arrays, so both C++ and Python can index the same memory directly.
constexpr uint64_t MAGIC = 0x5842504253484D31ULL; // XBPBSHM1
constexpr uint64_t VERSION = 1;
constexpr size_t HEADER_WORDS = 16;
constexpr size_t HEADER_SIZE = HEADER_WORDS * sizeof(uint64_t);
constexpr size_t STATE_FIELD_COUNT = 8;
constexpr size_t COMMAND_FIELD_COUNT = 5;

// Header words are uint64_t to keep the layout simple and naturally aligned.
// STATE_SEQ and COMMAND_SEQ implement a seqlock: odd means "writer active",
// even means "stable snapshot".
enum HeaderIndex : size_t
{
    IDX_MAGIC = 0,
    IDX_VERSION,
    IDX_HEADER_SIZE,
    IDX_TOTAL_SIZE,
    IDX_NJOINTS,
    IDX_NIMUS,
    IDX_STATE_SEQ,
    IDX_COMMAND_SEQ,
    IDX_SERVER_SESSION_ID,
    IDX_CLIENT_SESSION_ID,
    IDX_COMMAND_VALID,
    IDX_STATE_OFFSET,
    IDX_COMMAND_OFFSET,
    IDX_COMMAND_STAMP_NS,
    IDX_RESERVED0,
    IDX_RESERVED1,
};

struct Header
{
    uint64_t words[HEADER_WORDS];
};

static_assert(sizeof(Header) == HEADER_SIZE);

}

// 1. define driver classes for each device
// they are responsible for registering resources, implementing safety reactions, 
// and providing sense/move implementations that stream rx/tx data via internal topics

class JointDriver : public DeviceDriverTpl<joint_rx, joint_tx>,
                    private Journal
{

public:

    XBOT2_DECLARE_SMART_PTR(JointDriver)

    JointDriver(DeviceInfo dinfo, const Device::CommonParams& params);

private:

    JointSafety _safety;

    // DeviceDriverTpl interface
private:

    bool sense_impl() override;
    bool move_impl() override;
    void on_tx_recv(const TxType &msg) override;

    bool _init_done = false;
    TxType _tx_tmp;
    double _safe_kp, _safe_kd;
};

struct imu_packet {
    struct rx {
        double orientation[4] = {0,0,0,1};
        double angular_velocity[3] = {0,0,0};
        double linear_acceleration[3] = {0,0,0};
    };
    struct tx {
        // empty for now
    };
};

class ImuDriver : public DeviceDriverTpl<imu_packet::rx, imu_packet::tx>
{
public:
    XBOT2_DECLARE_SMART_PTR(ImuDriver)
    using DeviceDriverTpl::DeviceDriverTpl;
};

// 2. define client classes for each device, which will be eventually loaded by the user
// inside plugins (usually via robotinterface)

class ImuClient : public DeviceClientTpl<imu_packet::rx, imu_packet::tx>,
                  public virtual Imu
{
public:
    using DeviceClientTpl::DeviceClientTpl;

    Eigen::Vector3d getAngularVelocity() const override;
    Eigen::Vector3d getLinearAcceleration() const override;
    Eigen::Quaterniond getOrientation() const override;
};


// 3. define container class, responsible for loading the drivers and communicating
// with the simulator
class PyBridgeDeviceContainer : public DeviceContainerBase,
                                 private Journal
{

public:

    PyBridgeDeviceContainer(std::vector<DeviceInfo> devinfo,
                       const Device::CommonParams& params);


    bool sense_all() override;
    void run_all() override;
    bool move_all() override;

    bool send_string(const std::string& msg);
    bool recv_string(std::string& msg, bool blocking = true);

    ~PyBridgeDeviceContainer();

private:
    bool read_state_from_shm();
    void map_shared_memory(const std::string& shm_name,
                           uint64_t expected_server_session_id);
    void init_shm_views();

    // Sim time is updated outside sense_all(); the control thread may use the
    // simulated clock for synchronization while sense_all() is running.
    void sim_time_thread_main();
    uint64_t generate_client_session_id() const;

    // socket
    int _socket_fd;
    sockaddr_un _socket_local_addr;
    sockaddr_un _socket_remote_addr;

    // devs
    std::vector<JointDriver::Ptr> _joints;
    std::vector<ImuDriver::Ptr> _imus;

    // sim-time update thread
    std::unique_ptr<thread> _recv_thread;
    std::atomic_bool _recv_thread_run{true};

    // Shared-memory ownership remains on the Python server. The HAL only maps
    // the segment advertised during discovery and validates the session id.
    int _shm_fd = -1;
    void * _shm_addr = nullptr;
    size_t _shm_size = 0;
    PyBridgeShm::Header * _shm_header = nullptr;
    uint64_t _server_session_id = 0;
    uint64_t _client_session_id = 0;
    uint64_t _last_state_seq = 0;

    const double * _state_time = nullptr;

    // Direct views into the state and command frames. Field order is fixed by
    // STATE_FIELD_COUNT/COMMAND_FIELD_COUNT and mirrored in shm_protocol.py.
    std::array<const double *, PyBridgeShm::STATE_FIELD_COUNT> _state_joint{};
    struct ImuStateView
    {
        const double * quat_w = nullptr;
        const double * lin_acc_b = nullptr;
        const double * ang_vel_b = nullptr;
    };
    std::vector<ImuStateView> _state_imus;
    std::array<double *, PyBridgeShm::COMMAND_FIELD_COUNT> _command_joint{};

    // Scratch buffers let read_state_from_shm() validate the seqlock before
    // touching HAL device rx fields; torn reads are discarded cleanly.
    std::array<std::vector<double>, PyBridgeShm::STATE_FIELD_COUNT> _state_joint_tmp;
    struct ImuStateBuffer
    {
        double quat_w[4];
        double lin_acc_b[3];
        double ang_vel_b[3];
    };
    std::vector<ImuStateBuffer> _state_imu_tmp;

    bool _enable_simtime = true;
    bool _time_initialized = false;


};

class PyBridgeClientContainer : public DeviceContainerBase
{

public:

    PyBridgeClientContainer(std::vector<DeviceInfo> devinfo,
                       const Device::CommonParams& params);

};

}
}


#endif // PyBridge_HAL_H
