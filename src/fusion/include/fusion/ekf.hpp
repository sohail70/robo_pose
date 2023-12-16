#include<fusion/fusion.hpp>


namespace Filter{
    class Ekf: public Fusion{
        private:
        
        public:
            Ekf(std::unique_ptr<MotionModel> );
            virtual void predict(const rclcpp::Time& ) override;
            virtual void update() override;
    };
} //namespace Filter