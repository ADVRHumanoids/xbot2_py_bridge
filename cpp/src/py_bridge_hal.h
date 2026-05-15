#ifndef XBOT2_PyBridge_HAL_H
#define XBOT2_PyBridge_HAL_H

#include <xbot2/hal/device.h>
#include <xbot2/hal/dev_joint.h>
#include <xbot2/hal/dev_imu.h>
#include <xbot2/ipc/pipe.h>
#include <sys/un.h>
#include <nlohmann/json.hpp>


namespace XBot {
namespace Hal {

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

    struct JsonParameter : public Parameter<nlohmann::json>
    {
        JsonParameter(const std::string& name):
            Parameter(name)
        {}

        void clear()
        {
            _valid = false;
        }
    };

    bool receive_from_server();

    // socket
    int _socket_fd;
    sockaddr_un _socket_local_addr;
    sockaddr_un _socket_remote_addr;

    // devs
    std::vector<JointDriver::Ptr> _joints;
    std::vector<ImuDriver::Ptr> _imus;

    // recv thread
    std::unique_ptr<thread> _recv_thread;
    std::atomic_bool _recv_thread_run{true};

    // thread-safe json
    JsonParameter _recv_json;


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
