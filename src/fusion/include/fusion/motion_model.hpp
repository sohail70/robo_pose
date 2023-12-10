#ifndef MOTION_MODEL_HPP
#define MOTION_MODEL_HPP

#include<iostream>

namespace Filter{
    class MotionModel{
            public:
                MotionModel();
                virtual void init() = 0;
                virtual void update() = 0;
            protected:
                double x, y, z;
                double x_dot, y_dot, z_dot;
        };
} //namespace Filter
#endif