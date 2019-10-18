#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "dynamic_tutorials/visibility_control.h"

namespace DYNAMIC_TUTORIAL
{
class DynamicTutorialNode : public rclcpp::Node
{
public:
  DynamicTutorialNode()
  : Node("dynamic_tutorial_node")
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto param_change_callback =
      [this](std::vector<rclcpp::Parameter> parameters)
      {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        for (auto parameter : parameters) {
          rclcpp::ParameterType parameter_type = parameter.get_type();
          if (rclcpp::ParameterType::PARAMETER_NOT_SET == parameter_type) {
            RCLCPP_INFO(this->get_logger(),
              "parameter '%s' deleted successfully",
              parameter.get_name().c_str()
            );
            result.successful &= true;
          } else if (rclcpp::ParameterType::PARAMETER_INTEGER == parameter_type) {
            if (parameter.as_int() % 2 != 0) {
              RCLCPP_INFO(this->get_logger(),
                "Requested value '%d' for parameter '%s' is not an even number:"
                " rejecting change...",
                parameter.as_int(),
                parameter.get_name().c_str()
              );
              result.successful = false;
            } else {
              RCLCPP_INFO(this->get_logger(),
                "parameter '%s' has changed and is now: %s",
                parameter.get_name().c_str(),
                parameter.value_to_string().c_str()
              );
              result.successful &= true;
            }
          } else {
            RCLCPP_INFO(this->get_logger(),
              "only integer parameters can be set\n"
              "requested value for parameter '%s' is not an even number, rejecting change...",
              parameter.get_name().c_str()
            );
            result.successful = false;
          }
        }
        return result;
      };

    // callback_handler needs to be alive to keep the callback functional
    this->set_on_parameters_set_callback(param_change_callback);
  }
};

}  // namespace DYNAMIC_TUTORIAL

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = std::make_shared<DYNAMIC_TUTORIAL::DynamicTutorialNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
