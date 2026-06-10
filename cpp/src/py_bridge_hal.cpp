#include "py_bridge_hal.h"
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <chrono>
#include <random>
#include <thread>

#include <xbot2_interface/xbotinterface2.h>

XBot::Hal::PyBridgeDeviceContainer::PyBridgeDeviceContainer(std::vector<DeviceInfo> devinfo,
                                                  const Device::CommonParams &params)
    : DeviceContainerBase(), 
      Journal(Journal::no_publish, "PyBridge_hal")
{
    auto& j = *this;

    auto& pm = Context().paramManager();

    _socket_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if(_socket_fd == -1)
    {
        throw DeviceUnavailable("Error opening socket: " + std::string(strerror(errno)));
    }

    // setup remote address
    memset(&_socket_remote_addr, 0, sizeof (_socket_remote_addr));
    _socket_remote_addr.sun_family = AF_UNIX;
    std::string remote_name = "/tmp/.xbot2_isaac/xbot2_isaac_server.sock";
    pm.getParam("/xbot/hal/py_bridge_hal/remote_sock_addr", remote_name);
    strncpy(_socket_remote_addr.sun_path,
            remote_name.c_str(),
            remote_name.size());

    // setup local address
    memset(&_socket_local_addr, 0, sizeof (_socket_local_addr));
    _socket_local_addr.sun_family = AF_UNIX;
    std::string local_name = remote_name + ".client." + std::to_string(getpid());
    strncpy(_socket_local_addr.sun_path,
            local_name.c_str(),
            sizeof(_socket_local_addr.sun_path));

    if(local_name.size() > sizeof(_socket_local_addr.sun_path))
    {
        throw std::out_of_range("Local socket name too long ('" + local_name + "')");
    }

    // bind local address
    unlink(local_name.c_str());
    int bind_ret = bind(_socket_fd,
                        (struct sockaddr*)&_socket_local_addr,
                        local_name.size());
    if(bind_ret == -1)
    {
        throw std::runtime_error("Error binding local socket '" + local_name + "': " + std::string(strerror(errno)));
    }


    // Discovery is the only JSON/control-plane exchange in shm mode. It gives
    // us the device list plus the unique shm name and server session id.
    nlohmann::json discovery_msg;
    discovery_msg["type"] = "discovery";
    const std::string discovery_str = discovery_msg.dump();
    while(true)
    {
        try
        {
            if(send_string(discovery_str))
            {
                break;
            }
        }
        catch(std::runtime_error& e)
        {
            j.jwarn("discovery send failed: {}", e.what());
        }

        j.jinfo("waiting for xbot2 isaac server on socket '{}'", remote_name);
        usleep(666000);
    }

    j.jinfo("sent discovery message to server at {}, waiting for reply...", remote_name );

    // wait for response
    std::string response_str(40960, '\0');
    recv_string(response_str);
    auto response = nlohmann::json::parse(response_str);
    j.jinfo("...got response {}", response_str);

    const auto protocol = response.value("protocol", std::string{});
    if(protocol != "xbot2-shm-v1")
    {
        throw std::runtime_error("PyBridge server does not support xbot2-shm-v1");
    }

    const std::string shm_name = response.at("shm_name").get<std::string>();
    const uint64_t server_session_id = response.at("server_session_id").get<uint64_t>();


    // get urdf
    if(response.contains("urdf"))
    {

        auto urdf_str = response["urdf"].get<std::string>();
        XBot::ConfigOptions xb_ifc_cfg;
        xb_ifc_cfg.set_urdf(urdf_str);
        xb_ifc_cfg.set_srdf("<robot name=\"robot\"/>");


        // change framework to xbot2rt
        xb_ifc_cfg.set_parameter<std::string>("robot_type", "xbot2rt");

        // load robot interface cfg object to param manager
        pm.setParam("/xbot/hal/robot_ifc_cfg", xb_ifc_cfg);

        // make an xbi to get info about hal
        auto xbi = ModelInterface::getModel(xb_ifc_cfg);
        xbi->print(std::cout);

        // upload urdf to internal params (used for safety limits)
        pm.setParam("/xbot/robot_description", xbi->getUrdfString());
        pm.setParam<urdf::ModelInterface>("/xbot/urdf_model", *xbi->getUrdf());

    }

    // construct joints
    auto joint_names = response["joint_names"].get<std::vector<std::string>>();

    for(auto jname : joint_names)
    {
        DeviceInfo dinfo;
        dinfo.name = jname;
        dinfo.type = "joint_PyBridge";
        dinfo.id = -1;
        auto dev = std::make_shared<JointDriver>(dinfo, params);
        addDevice(dev);
        _joints.push_back(dev);
        j.jinfo("added joint '{}'", jname);
    }

    // imu
    auto imu_names = response["imu_sensors"].get<std::vector<std::string>>();
    for(auto iname : imu_names)
    {
        DeviceInfo dinfo;
        dinfo.name = iname;
        dinfo.type = "imu_PyBridge";
        dinfo.id = -1;
        auto dev = std::make_shared<ImuDriver>(dinfo, params);
        addDevice(dev);
        _imus.push_back(dev);
        j.jinfo("added imu '{}'", iname);
    }

    // Devices are known now, so validate the shm header counts against the HAL
    // device vectors before creating raw array views into the segment.
    map_shared_memory(shm_name, server_session_id);
    init_shm_views();

    // 
    pm.getParam("/xbot/hal/py_bridge_hal/enable_sim_time", _enable_simtime);
    j.jinfo("simulated time is {}", _enable_simtime ? "enabled" : "disabled");

    if(!_enable_simtime)
    {
        return;
    }

    // enable sim time
    chrono::simulated_clock::enable_sim_time(true);

    // Keep sim-time updates outside sense_all(); the control thread may use the
    // same simulated clock for synchronization while sense_all() is running.
    _recv_thread = std::make_unique<thread>(&PyBridgeDeviceContainer::sim_time_thread_main, this);

}

bool XBot::Hal::PyBridgeDeviceContainer::sense_all()
{

    if(!read_state_from_shm())
    {
        return false;
    }

    DeviceContainerBase::sense_all();

    return true;
}

void XBot::Hal::PyBridgeDeviceContainer::run_all()
{

}

bool XBot::Hal::PyBridgeDeviceContainer::move_all()
{
    DeviceContainerBase::move_all();

    if(!_shm_header)
    {
        return false;
    }

    using namespace PyBridgeShm;

    // Publish command with a seqlock:
    // 1. make COMMAND_SEQ odd so Python sees the frame as unstable,
    // 2. write payload and session metadata,
    // 3. make COMMAND_SEQ even to publish a complete snapshot.
    uint64_t seq = _shm_header->words[IDX_COMMAND_SEQ];
    if(seq & 1)
    {
        seq++;
    }

    _shm_header->words[IDX_COMMAND_SEQ] = seq + 1;

    // Fixed command frame order: q, dq, tau, k, d. We always publish full
    // commands, so reconnects never depend on stale masked/partial values.
    for(size_t i = 0; i < _joints.size(); i++)
    {
        auto& tx = _joints[i]->tx();
        _command_joint[0][i] = tx.pos_ref;
        _command_joint[1][i] = tx.vel_ref;
        _command_joint[2][i] = tx.tor_ref;
        _command_joint[3][i] = tx.gain_kp;
        _command_joint[4][i] = tx.gain_kd;
    }

    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

    // Session ids and monotonic timestamp are stale-command defense. The server
    // rejects commands from old mappings, old clients, or old ticks.
    _shm_header->words[IDX_SERVER_SESSION_ID] = _server_session_id;
    _shm_header->words[IDX_CLIENT_SESSION_ID] = _client_session_id;
    _shm_header->words[IDX_COMMAND_STAMP_NS] = static_cast<uint64_t>(now_ns);
    _shm_header->words[IDX_COMMAND_VALID] = 1;
    _shm_header->words[IDX_COMMAND_SEQ] = seq + 2;

    return true;
}

bool XBot::Hal::PyBridgeDeviceContainer::send_string(const std::string &msg)
{
    int ret = sendto(_socket_fd,
               msg.data(), msg.size(), // do not send null termination
               0,
               reinterpret_cast<const sockaddr*>(&_socket_remote_addr),
               sizeof(_socket_remote_addr));

    if(ret < 0)
    {
        throw std::runtime_error("Error sendto: " + std::string(strerror(errno)));
    }

    return ret == msg.size();
}

bool XBot::Hal::PyBridgeDeviceContainer::recv_string(std::string &msg, bool blocking)
{
    // msg.resize(4096);
    int ret = recvfrom(_socket_fd,
                       msg.data(), msg.size(),
                       blocking ? 0 : MSG_DONTWAIT,
                       nullptr, nullptr);
    if(ret > 0)
    {
        msg.resize(ret);
        return true;
    }
    else
    {
        return false;
    }
}

uint64_t XBot::Hal::PyBridgeDeviceContainer::generate_client_session_id() const
{
    std::random_device rd;
    uint64_t hi = static_cast<uint64_t>(rd()) << 32;
    uint64_t lo = static_cast<uint64_t>(rd());
    uint64_t ret = hi ^ lo ^ static_cast<uint64_t>(getpid());
    return ret == 0 ? 1 : ret;
}

void XBot::Hal::PyBridgeDeviceContainer::map_shared_memory(const std::string& shm_name,
                                                           uint64_t expected_server_session_id)
{
    using namespace PyBridgeShm;

    // Python's SharedMemory name has no leading slash, while shm_open() wants
    // the POSIX form. Accept either to keep discovery payloads simple.
    std::string posix_name = shm_name;
    if(posix_name.empty())
    {
        throw std::runtime_error("empty py_bridge shm name");
    }
    if(posix_name.front() != '/')
    {
        posix_name.insert(posix_name.begin(), '/');
    }

    _shm_fd = shm_open(posix_name.c_str(), O_RDWR, 0);
    if(_shm_fd == -1)
    {
        throw std::runtime_error("unable to open py_bridge shm '" + posix_name + "': " + std::string(strerror(errno)));
    }

    struct stat fd_stat;
    if(fstat(_shm_fd, &fd_stat) == -1)
    {
        throw std::runtime_error("unable to stat py_bridge shm '" + posix_name + "': " + std::string(strerror(errno)));
    }
    _shm_size = static_cast<size_t>(fd_stat.st_size);

    _shm_addr = mmap(nullptr, _shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, _shm_fd, 0);
    if(_shm_addr == MAP_FAILED)
    {
        _shm_addr = nullptr;
        throw std::runtime_error("unable to map py_bridge shm '" + posix_name + "': " + std::string(strerror(errno)));
    }

    if(_shm_size < HEADER_SIZE)
    {
        throw std::runtime_error("py_bridge shm is smaller than header");
    }

    _shm_header = reinterpret_cast<Header *>(_shm_addr);

    // Validate the ABI before storing any persistent pointers into the segment.
    // A mismatch here means the C++ and Python packages are out of sync.
    if(_shm_header->words[IDX_MAGIC] != MAGIC ||
       _shm_header->words[IDX_VERSION] != VERSION ||
       _shm_header->words[IDX_HEADER_SIZE] != HEADER_SIZE)
    {
        throw std::runtime_error("py_bridge shm header/version mismatch");
    }
    if(_shm_header->words[IDX_TOTAL_SIZE] > _shm_size)
    {
        throw std::runtime_error("py_bridge shm size mismatch");
    }
    if(_shm_header->words[IDX_NJOINTS] != _joints.size() ||
       _shm_header->words[IDX_NIMUS] != _imus.size())
    {
        throw std::runtime_error("py_bridge shm device count mismatch");
    }
    if(_shm_header->words[IDX_SERVER_SESSION_ID] != expected_server_session_id)
    {
        throw std::runtime_error("py_bridge shm server session mismatch");
    }

    _server_session_id = expected_server_session_id;
    _client_session_id = generate_client_session_id();

    // Announce this client session immediately, but leave the command frame
    // invalid until move_all() publishes the first complete command.
    _shm_header->words[IDX_CLIENT_SESSION_ID] = _client_session_id;
    _shm_header->words[IDX_COMMAND_VALID] = 0;
}

void XBot::Hal::PyBridgeDeviceContainer::init_shm_views()
{
    using namespace PyBridgeShm;
    auto * base = reinterpret_cast<uint8_t *>(_shm_addr);
    auto * state = reinterpret_cast<double *>(base + _shm_header->words[IDX_STATE_OFFSET]);

    // State layout: time, then joint arrays q/dq/tau/k/d/qref/vref/tauref,
    // then per-IMU quat_w/lin_acc_b/ang_vel_b arrays.
    _state_time = state;
    state += 1;
    for(size_t field = 0; field < STATE_FIELD_COUNT; field++)
    {
        _state_joint[field] = state;
        state += _joints.size();
    }

    _state_imus.resize(_imus.size());
    for(auto& imu : _state_imus)
    {
        imu.quat_w = state;
        state += 4;
        imu.lin_acc_b = state;
        state += 3;
        imu.ang_vel_b = state;
        state += 3;
    }

    // Command layout mirrors Python's COMMAND_FIELDS: q/dq/tau/k/d.
    auto * command = reinterpret_cast<double *>(base + _shm_header->words[IDX_COMMAND_OFFSET]);
    for(size_t field = 0; field < COMMAND_FIELD_COUNT; field++)
    {
        _command_joint[field] = command;
        command += _joints.size();
    }

    for(auto& field_tmp : _state_joint_tmp)
    {
        field_tmp.resize(_joints.size());
    }
    _state_imu_tmp.resize(_imus.size());
}

bool XBot::Hal::PyBridgeDeviceContainer::read_state_from_shm()
{
    using namespace PyBridgeShm;
    if(!_shm_header)
    {
        return false;
    }

    const uint64_t seq1 = _shm_header->words[IDX_STATE_SEQ];

    // No state has been published yet, or Python is currently writing one.
    if(seq1 == 0 || (seq1 & 1))
    {
        return false;
    }

    // Copy into scratch first. If the seqlock changes below, these bytes are
    // discarded and existing device rx values remain untouched.
    for(size_t field = 0; field < STATE_FIELD_COUNT; field++)
    {
        std::memcpy(_state_joint_tmp[field].data(),
                    _state_joint[field],
                    _joints.size()*sizeof(double));
    }

    for(size_t i = 0; i < _state_imus.size(); i++)
    {
        std::memcpy(_state_imu_tmp[i].quat_w, _state_imus[i].quat_w, 4*sizeof(double));
        std::memcpy(_state_imu_tmp[i].lin_acc_b, _state_imus[i].lin_acc_b, 3*sizeof(double));
        std::memcpy(_state_imu_tmp[i].ang_vel_b, _state_imus[i].ang_vel_b, 3*sizeof(double));
    }

    const uint64_t seq2 = _shm_header->words[IDX_STATE_SEQ];

    // Python published a newer frame while we were copying; try again next tick.
    if(seq1 != seq2 || (seq2 & 1))
    {
        return false;
    }

    // Snapshot is stable, so it is now safe to update HAL rx structures.
    for(size_t i = 0; i < _joints.size(); i++)
    {
        auto& rx = _joints[i]->rx();
        rx.link_pos = rx.motor_pos = _state_joint_tmp[0][i];
        rx.link_vel = rx.motor_vel = _state_joint_tmp[1][i];
        rx.torque = _state_joint_tmp[2][i];
        rx.gain_kp = _state_joint_tmp[3][i];
        rx.gain_kd = _state_joint_tmp[4][i];
        rx.pos_ref = _state_joint_tmp[5][i];
        rx.vel_ref = _state_joint_tmp[6][i];
        rx.tor_ref = _state_joint_tmp[7][i];
    }

    for(size_t i = 0; i < _imus.size(); i++)
    {
        auto& rx = _imus[i]->rx();
        const auto& imu = _state_imu_tmp[i];
        std::memcpy(rx.orientation, imu.quat_w, 4*sizeof(double));
        std::memcpy(rx.linear_acceleration, imu.lin_acc_b, 3*sizeof(double));
        std::memcpy(rx.angular_velocity, imu.ang_vel_b, 3*sizeof(double));
    }

    _last_state_seq = seq2;
    return true;
}

void XBot::Hal::PyBridgeDeviceContainer::sim_time_thread_main()
{
    this_thread::set_name("py_bridge");

    uint64_t last_time_seq = 0;

    while(_recv_thread_run.load())
    {
        if(!_shm_header)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }

        // Use the same seqlock idea as sense_all(), but only read time. This
        // thread exists because the control thread may sync on simulated_clock.
        const uint64_t seq1 = _shm_header->words[PyBridgeShm::IDX_STATE_SEQ];
        if(seq1 == 0 || (seq1 & 1) || seq1 == last_time_seq)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }

        const double time = *_state_time;
        const uint64_t seq2 = _shm_header->words[PyBridgeShm::IDX_STATE_SEQ];
        if(seq1 != seq2 || (seq2 & 1))
        {
            continue;
        }

        if(!_time_initialized)
        {
            _time_initialized = true;
            chrono::simulated_clock::initialize(std::chrono::nanoseconds(int64_t(time*1e9)));
        }

        chrono::simulated_clock::set_time(std::chrono::nanoseconds(int64_t(time*1e9)));
        last_time_seq = seq2;
    }

    chrono::simulated_clock::enable_sim_time(false);
}


XBot::Hal::PyBridgeDeviceContainer::~PyBridgeDeviceContainer()
{
    _recv_thread_run = false;
    
    if(_recv_thread)
    {
        _recv_thread->join();       
    }

    if(_shm_addr)
    {
        munmap(_shm_addr, _shm_size);
    }

    if(_shm_fd >= 0)
    {
        close(_shm_fd);
    }

    if(_socket_fd >= 0)
    {
        close(_socket_fd);
    }
}



XBot::Hal::PyBridgeClientContainer::PyBridgeClientContainer(std::vector<DeviceInfo> devinfo,
                                                  const Device::CommonParams &params):
    DeviceContainerBase()
{
    // loop over devinfo and create clients

    for(auto& dinfo : devinfo)
    {
        if(dinfo.type == "joint_PyBridge")
        {
            auto dev = std::make_shared<JointClient>(dinfo, params);
            addDevice(dev);
        }
        else if(dinfo.type == "imu_PyBridge")
        {
            auto dev = std::make_shared<ImuClient>(dinfo, params);
            addDevice(dev);
        }
    }
}


XBot::Hal::JointDriver::JointDriver(DeviceInfo dinfo, const CommonParams &params)
    :
    DeviceDriverTpl<joint_rx, joint_tx>(dinfo, params),
    Journal(Journal::no_publish, dinfo.name),
    _safety(dinfo, get_period_sec(), JointSafety::safety_not_required)
{
    // declare available resources
    uint32_t mask = ~0;
    uint8_t _pos_mask = JointBase::Resource::Mask::Position;
    uint8_t _vel_mask = JointBase::Resource::Mask::Velocity;
    uint8_t _tor_mask = JointBase::Resource::Mask::Effort;
    uint8_t _imp_mask = JointBase::Resource::Mask::Impedance;


    if(mask & _pos_mask)
    {
        XBOT2_ASSERT_THROW(
            register_resource(JointBase::Resource::Position, _pos_mask)
            );
    }

    if(mask & _vel_mask)
    {
        XBOT2_ASSERT_THROW(
            register_resource(JointBase::Resource::Velocity, _vel_mask)
            );
    }

    if(mask & _tor_mask)
    {
        XBOT2_ASSERT_THROW(
            register_resource(JointBase::Resource::Effort, _tor_mask)
            );
    }

    if(mask & _imp_mask)
    {
        XBOT2_ASSERT_THROW(
            register_resource(JointBase::Resource::Impedance, _imp_mask)
            );

        XBOT2_ASSERT_THROW(
            register_resource(JointBase::Resource::Stiffness,
                              JointBase::Resource::Mask::Stiffness)
            );

        XBOT2_ASSERT_THROW(
            register_resource(JointBase::Resource::Damping,
                              JointBase::Resource::Mask::Damping)
            );
    }

    // customize safety reactions
    _safety.set_on_safety_triggered([this](joint_tx& tx)
                                    {
                                        if(_rx.gain_kp < _safe_kp)
                                        {
                                            tx.gain_kp = _safe_kp;
                                            tx.gain_kd = _safe_kd;
                                            tx.pos_ref = _rx.motor_pos;
                                            jwarn("setting safe impedance: kp = {}  kd = {}",
                                                     _safe_kp, _safe_kd);
                                        }
                                    });
}

bool XBot::Hal::JointDriver::sense_impl()
{
    if(!_init_done)
    {
        // initialize tx and safety from received rx
        _safety.initialize(_rx);
        _tx_tmp.reset(_rx);

        // safe gains are defined as the first received gains
        // this is reasonable at least in simulation
        _safe_kp = _rx.gain_kp;
        _safe_kd = _rx.gain_kd;

        _init_done = true;
    }

    return true;
}

bool XBot::Hal::JointDriver::move_impl()
{
    // turn _tmp_tx into a safe tx from safety filter,
    // and save it to _tx

    if(!_safety.enforce(_tx_tmp, _tx))
    {
        // note: returning false means "unable to
        // communicate with the robot"
        // so, we don't

        // reset _tx_tmp (which is unsafe) to be safe
        _tx_tmp = _tx;
    }

    // note: reset mask before next tx msg received
    _tx_tmp.mask = 0;

    return true;
}

void XBot::Hal::JointDriver::on_tx_recv(const TxType &msg)
{
    _tx_tmp.apply(msg);
}

Eigen::Vector3d XBot::Hal::ImuClient::getAngularVelocity() const
{
    return Eigen::Vector3d(_rx.angular_velocity[0],
                           _rx.angular_velocity[1],
                           _rx.angular_velocity[2]);
}

Eigen::Vector3d XBot::Hal::ImuClient::getLinearAcceleration() const
{
    return Eigen::Vector3d(_rx.linear_acceleration[0],
                           _rx.linear_acceleration[1],
                           _rx.linear_acceleration[2]);
}

Eigen::Quaterniond XBot::Hal::ImuClient::getOrientation() const
{
    return Eigen::Quaterniond(_rx.orientation[3], _rx.orientation[0], _rx.orientation[1], _rx.orientation[2]);
}

XBOT2_REGISTER_DEVICE(XBot::Hal::PyBridgeDeviceContainer, XBot::Hal::PyBridgeClientContainer, py_bridge_hal)
