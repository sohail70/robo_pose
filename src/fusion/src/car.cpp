#include "fusion/car.hpp"

namespace Filter{
    Car::Car()
    {
        std::cout<<"Ctor of Car \n";
    }

    autodiff::VectorXreal Car::propagate(const autodiff::VectorXreal&)
    {}

    // Eigen::MatrixXd Car::getJacobian()
    // {}

} //namespace Filter