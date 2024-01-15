#include "model/model.hpp"

namespace model
{

Model::Model() : Filter::MotionModel()
{
    std::cout<<"Ctor of plugin simple model \n";
}

autodiff::VectorXreal Model::propagate(const autodiff::VectorXreal &state)
{
    // std::cout<<"PROPAGATE OF PLUGIN \n";
    autodiff::VectorXreal newState(state.size());
    autodiff::real dt = autodiff::real(dt_.seconds());
    const auto index = states_->getStateOrder();
    // std::cout<<"x_dot:"<<(index.find("x") != index.end() )<<"\n";
    newState(0) = state(index.at("x")) + state(index.at("x_dot")) * cos(state(index.at("yaw"))) * dt; // Update x
    newState(1) = state(index.at("y")) + state(index.at("x_dot")) * sin(state(index.at("yaw"))) * dt; // Update y
    newState(2) = state(index.at("yaw")) + state(index.at("yaw_dot")) * dt;                           // Update yaw angle
    newState(3) = state(index.at("x_dot")) + state(index.at("x_ddot")) * dt;
    newState(4) = state(index.at("yaw_dot"));
    newState(5) = state(index.at("x_ddot"));
    return newState;
}

Model::~Model()
{
}

}  // namespace model


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(model::Model, Filter::MotionModel)