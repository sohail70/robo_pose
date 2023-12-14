#ifndef CONSTANT_HEADING_RATE_HPP
#define CONSTANT_HEADING_RATE_HPP

#include "fusion/motion_model.hpp"
namespace Filter{

    class ConstantHeadingRate: public MotionModel
    {
        public:
            ConstantHeadingRate();
            virtual void init() override;
            virtual void update(const rclcpp::Time&) override;
            virtual void setVelocity(const Filter::Velocity& ) override;
            virtual void setAngularVelocity(const Filter::AngularVelocity& ) override;
            Filter::Position& getPosition();
            Filter::Angle& getAngle();
        private:

    };
} //namespace Filter
#endif