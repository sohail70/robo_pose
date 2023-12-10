#include<gtest/gtest.h>
#include<fusion/motion_model_factory.hpp>
#include<fusion/motion_model.hpp>
#include<typeinfo>
TEST(FactoryTest , creatingObjectTest) //1st is group and 2nd is specific thing you wanna do
{
    ASSERT_EQ(1,1);
    Filter::MotionModelFactory factory_;
    std::unique_ptr<Filter::MotionModel> constant_heading_model_ = factory_.createModel(Filter::ModelType::CONSTANT_HEADING_RATE);
    std::unique_ptr<Filter::MotionModel> car_ = factory_.createModel(Filter::ModelType::CAR);

    ASSERT_NE(constant_heading_model_,nullptr);
    ASSERT_NE(car_,nullptr);


    ASSERT_EQ(typeid(*constant_heading_model_) , typeid(Filter::ConstantHeadingRate));
    ASSERT_EQ(typeid(*car_) , typeid(Filter::Car));

}