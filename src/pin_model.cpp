#include "gct/pin_model.hpp"

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <stdexcept>

namespace gct {

struct PinModel::Impl {
    pinocchio::Model model;
    mutable pinocchio::Data data;//临时缓存，可在const函数修改
};


PinModel::PinModel(const std::string& urdf_path, double armature)
    : impl_(std::make_unique<Impl>()), urdf_path_(urdf_path){
    //urdf->pinocchio model
    pinocchio::urdf::buildModel(urdf_path, impl_->model);
    //buildModel不会给半成品，如果速度维度为0,说明解析出了一个空的模型
    if(impl_->model.nv() == 0){
        throw std::runtime_error("URDF produced an empty Pinocchio model: " +
                             urdf_path);
    }
    //这里是简化写法，实际的电机转子惯量会根据URDF中的惯量信息计算
    impl_->model.armature.setConstant(armature);
    //创建一个model大小的data
    impl_->data = pinocchio::Data(impl_->model);
}

//拷贝构造函数
PinModel::PinModel(const PinModel& other):urdf_path_(other.urdf_path_){
    if(other.impl_.empty()){
        return;
    }
    impl_ = std::make_unique<Impl>();
    impl_->model = other.impl_->model;
    impl_->data = pinocchio::Data(impl_->model);
}

PinModel::PinModel& operator=(const PinModel& other){
    //防止自拷贝
    if(this == &other){
        return *this;
    }

    urdf_path_ = other.urdf_path_;
    //防止复制空对象
    if(other.impl_ == nullptr){
        impl_.reset();
        return *this;
    }

    impl_ = std::make_unique<Impl>();
    impl_->model = other.impl_->model;
    impl_->data = pinocchio::Data(impl_->model);

    return *this;
}

//移动构造函数和移动赋值运算符，效率相比拷贝更高
PinModel(PinModel&& other) noexcept = default;
PinModel& operator=(PinModel&& other) noexcept = default;
~PinModel() = default;

int PinModel::nv() const{
    return impl_!=nullptr ? static_cast<int>impl_->model.nv() : 0;
}

int PinModel::nq() const{
    return impl_!=nullptr ? static_cast<int>impl_->model.nq() : 0;
}

std::vector<std::string> joint_names() const{
    if(impl_ == nullptr){
        return {};
    }

    std::vector<std::string> names;
    names.reserve(static_cast<std::size_t>(impl_->model.njoints()));

    for(pinocchio::JointIndex i = 0; i < static_cast<pinocchio::JointIndex>(impl_->model.njoints()); ++i){
        names.push_back(impl_->model.names[i]);
    }

    return names;
}

VectorXd torque_limits() const{
    if(impl_ == nullptr){
        return {};
    }
    return impl_->model.effortLimit;
}

VectorXd gravity(const VectorXd& q) const{
    //速度和加速度为0的逆动力学
    return rnea(q, VectorXd::Zero(nv()), VectorXd::Zero(nv()));
}

VectorXd rnea(const VectorXd& q, const VectorXd& v, const VectorXd& a) const{
    if(impl_ == nullptr){
        throw std::runtime_error("PinModel is not empty");
    }

    if(q.size()!=nq() || v.size()!=nv() || a.size()!=nv()){
        throw std::invalid_argument("state vector has the wrong size");
    }

    return rnea(impl_->model, impl_->data, q, v, a);
}

const std::string& urdf_path() const { return urdf_path_; }

}