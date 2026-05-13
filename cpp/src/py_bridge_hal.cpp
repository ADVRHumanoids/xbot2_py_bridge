#include "py_bridge_hal.h"
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <xbot2_interface/xbotinterface2.h>

XBot::Hal::PyBridgeDeviceContainer::PyBridgeDeviceContainer(std::vector<DeviceInfo> devinfo,
                                                  const Device::CommonParams &params)
    : DeviceContainerBase(), _recv_json("__PyBridge_hal_recv_json")
{
    Journal j("PyBridge_hal");

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


    // discovery
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
    j.jinfo("...got response");


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

    // TBD imu and other devices

    // enable sim time
    chrono::simulated_clock::enable_sim_time(true);

    // thread for reading sim state and setting sim time
    _recv_thread = std::make_unique<thread>([this]()
    {
        this_thread::set_name("py_bridge");

        bool time_initialized = false;

        while(_recv_thread_run.load())
        {
            // wait for message
            std::string response_str(40960, '\0');

            if(!recv_string(response_str, true))
            {
                continue;
            }

            // keep only most recent
            while(true)
            {
                response_str.resize(40960);
                if(!recv_string(response_str, false))
                {
                    break;
                }
            }

            // parse json
            nlohmann::json response = nlohmann::json::parse(response_str);

            // handle simulation time
            double time = response["time"].get<double>();

            if(!time_initialized)
            {
                time_initialized = true;
                chrono::simulated_clock::initialize(std::chrono::nanoseconds(int64_t(time*1e9)));
            }

            chrono::simulated_clock::set_time(std::chrono::nanoseconds(int64_t(time*1e9)));

            // set response
            _recv_json.set_value(response);
        }

        chrono::simulated_clock::enable_sim_time(false);
    });

}

bool XBot::Hal::PyBridgeDeviceContainer::sense_all()
{

    if(!_recv_json.isValid())
    {
        return false;
    }

    nlohmann::json response;
    _recv_json.get_value(response);
    _recv_json.clear();

    auto type = response["type"].get<std::string>();

    if(type == "state")
    {
        // joints
        auto& js = response["joints"];

        auto q    = js["q"].get<std::vector<double>>();
        auto dq   = js["dq"].get<std::vector<double>>();
        auto tau  = js["tau"].get<std::vector<double>>();
        auto k    = js["k"].get<std::vector<double>>();
        auto d    = js["d"].get<std::vector<double>>();
        auto qref = js["qref"].get<std::vector<double>>();
        auto vref = js["vref"].get<std::vector<double>>();
        auto tauref = js["tauref"].get<std::vector<double>>();

        for(int i = 0; i < _joints.size(); i++)
        {
            auto& rx = _joints[i]->rx();
            rx.link_pos = rx.motor_pos = q[i];
            rx.link_vel = rx.motor_vel = dq[i];
            rx.torque = tau[i];
            rx.gain_kp = k[i];
            rx.gain_kd = d[i];
            rx.pos_ref = qref[i];
            rx.vel_ref = vref[i];
            rx.tor_ref = tauref[i];
        }

        // TBD imu
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

    // prepare command message as JSON
    nlohmann::json command_msg;
    command_msg["type"] = "control";

    auto& jc = command_msg["joint_command"];
    auto& q   = jc["q"]   = nlohmann::json::array();
    auto& dq  = jc["dq"]  = nlohmann::json::array();
    auto& tau = jc["tau"] = nlohmann::json::array();
    auto& k   = jc["k"]   = nlohmann::json::array();
    auto& d   = jc["d"]   = nlohmann::json::array();

    for(auto& j : _joints)
    {
        auto& tx = j->tx();
        q.push_back(tx.pos_ref);
        dq.push_back(tx.vel_ref);
        tau.push_back(tx.tor_ref);
        k.push_back(tx.gain_kp);
        d.push_back(tx.gain_kd);
    }

    if(!send_string(command_msg.dump()))
    {
        return false;
    }

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

XBot::Hal::PyBridgeDeviceContainer::~PyBridgeDeviceContainer()
{
    _recv_thread_run = false;
    _recv_thread->join();
}



XBot::Hal::PyBridgeClientContainer::PyBridgeClientContainer(std::vector<DeviceInfo> devinfo,
                                                  const Device::CommonParams &params):
    DeviceContainer(devinfo, params)
{

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


XBOT2_REGISTER_DEVICE(XBot::Hal::PyBridgeDeviceContainer, XBot::Hal::PyBridgeClientContainer, py_bridge_hal)
