#include <pluginlib/class_loader.hpp>
#include <fusion/motion_model.hpp>

int main(int argc, char** argv)
{
  // To avoid unused parameter warnings
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<Filter::MotionModel> loader("fusion", "Filter::MotionModel");

  try
  {
    std::shared_ptr<Filter::MotionModel> model = loader.createSharedInstance("model::Model");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}