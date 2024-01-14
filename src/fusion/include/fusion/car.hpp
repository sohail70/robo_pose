#ifndef CAR_HPP
#define CAR_HPP

#include "fusion/motion_model.hpp"

namespace Filter{

    class Car: public MotionModel
    {
        public:
            Car();
            // virtual Eigen::MatrixXd getJacobian() override;
            virtual autodiff::VectorXreal propagate(const autodiff::VectorXreal&) override;


        private:

    };
} //namespace Filter
#endif