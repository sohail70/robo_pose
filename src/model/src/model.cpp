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
    newState(0)  = state(index.at("x")) + state(index.at("x_dot")) * cos(state(index.at("yaw"))) * dt; 
    newState(1)  = state(index.at("y")) + state(index.at("x_dot")) * sin(state(index.at("yaw"))) * dt; 
    newState(2)  = state(index.at("z"));                                                             
    newState(3)  = state(index.at("roll"));
    newState(4)  = state(index.at("pitch"));
    newState(5)  = state(index.at("yaw")) + state(index.at("yaw_dot")) * dt;
    newState(6)  = state(index.at("x_dot")) + state(index.at("x_ddot")) * dt;
    newState(7)  = state(index.at("y_dot"));
    newState(8)  = state(index.at("z_dot")); 
    newState(9)  = state(index.at("roll_dot"));
    newState(10) = state(index.at("pitch_dot"));
    newState(11) = state(index.at("yaw_dot"));
    newState(12) = state(index.at("x_ddot"));
    newState(13) = state(index.at("y_ddot"));
    newState(14) = state(index.at("z_ddot"));
    return newState;

}

Model::~Model()
{
}

}  // namespace model


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(model::Model, Filter::MotionModel)