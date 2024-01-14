#ifndef MODEL__MODEL_HPP_
#define MODEL__MODEL_HPP_

#include "model/visibility_control.h"
#include<fusion/motion_model.hpp>
namespace model
{

class Model : public Filter::MotionModel
{
public:
  Model();
  Model(const rclcpp::NodeOptions & options);
  virtual autodiff::VectorXreal propagate(const autodiff::VectorXreal & state) override;
  virtual ~Model();
};

}  // namespace model

#endif  // MODEL__MODEL_HPP_
