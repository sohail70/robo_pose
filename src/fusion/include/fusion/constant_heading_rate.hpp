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
        private:

    };
} //namespace Filter
#endif