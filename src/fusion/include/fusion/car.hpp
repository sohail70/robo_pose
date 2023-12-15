#ifndef CAR_HPP
#define CAR_HPP

#include "fusion/motion_model.hpp"

namespace Filter{

    class Car: public MotionModel
    {
        public:
            Car();
            virtual void init() override;
            virtual void update(const rclcpp::Time&) override;
            virtual void setVelocity(const Filter::Velocity& ) override;
            virtual void setAngularVelocity(const Filter::AngularVelocity& ) override;
            virtual void setVelAndAngVelFromTwist(const geometry_msgs::msg::Twist& ) override;
        private:

    };
} //namespace Filter
#endif