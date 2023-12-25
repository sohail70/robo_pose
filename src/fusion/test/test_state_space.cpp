#include<gtest/gtest.h>
#include<fusion/ekf.hpp>
#include<fusion/sensor_data.hpp>
#include<fusion/motion_model.hpp>
#include<fusion/motion_model_factory.hpp>
#include<geometry_msgs/msg/pose2_d.hpp>
#include<fusion/visualization.hpp>
#include<vector>
#include<fusion/state_space.hpp>

TEST(StateSpace , test1)
{
    ASSERT_EQ(1,1);
    std::vector<std::string> state_names{"x" , "y" , "yaw"};
    Filter::StateSpace state_space_(state_names);
    state_space_.updateStates({1,2,3}); 
    /////////////////////////////////



}