#pragma once

#include "gct/pin_model.hpp"

namespace gct {

struct CompensatorConfig{
    VectorXd kp;
    VectorXd kd;
    bool clip_torque{true};
};

class GravityCompensator{
public:
    /*
    TODO
    构造函数
    查询model config
    三种控制律，gravity,pd+gravity,position_servo
    力矩限制
    */

    explicit GravityCompensator(const PinModel& model, const CompensatorConfig& config = {});

    //const只读，&引用效率更高
    const PinModel& model() const { return model_; }
    const CompensatorConfig& config() const { return config_; }

    VectorXd gravity_torque(const VectorXd& q) const;

    VectorXd pd_plus_gravity(const VectorXd& q, const VectorXd& v, const VectorXd& q_des, const VectorXd& v_des) const;

    //v_des作为前馈
    VectorXd position_servo_command(const VectorXd& q, const VectorXd& q_des,
        const VectorXd& v_des) const;

    VectorXd clip_torque(const VectorXd& tau) const;

private:
    PinModel model_;
    CompensatorConfig config_;
};
}