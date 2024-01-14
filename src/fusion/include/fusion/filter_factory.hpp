#ifndef FILTER_FACTORY_HPP
#define FILTER_FACTORY_HPP

#include<unordered_map>
#include<fusion/fusion.hpp>
#include<fusion/ekf.hpp>
namespace Filter{
    enum class FilterType{EKF,UKF};
    class FilterFactory
    {
        public:
            FilterFactory();
            std::unique_ptr<Fusion> createFilter(FilterType  , std::shared_ptr<MotionModel> , std::shared_ptr<StateSpace> );
        private:
            using Creator = std::function<std::unique_ptr<Fusion>()>;
            std::unordered_map<int,Creator> filters_;

    };


} // namespace Filter


#endif