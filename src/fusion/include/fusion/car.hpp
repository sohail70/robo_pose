#ifndef CAR_HPP
#define CAR_HPP

#include "fusion/motion_model.hpp"
namespace Filter{

    class Car: public MotionModel
    {
        public:
            Car();
            virtual void init() override;
            virtual void update() override;
        private:

    };
} //namespace Filter
#endif