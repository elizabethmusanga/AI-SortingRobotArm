#ifndef ROBOTIC_ARM_HWINTERFACE_HPP
#define ROBOTIC_ARM_HWINTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interfaces.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <libserial/SerialPort.h>

#include <vector>
#include <string>

namespace robotic_arm_hw
{
    using callback_return = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::callback_return;

    class RoboticArmHWInterface : public hardware_interface::SystemInterface
    {
        public:
            RoboticArmHWInterface();

            // Destructor
            virtual ~RoboticArmHWInterface();

            virtual callback_return on_active(const rclcpp_lifecycle::State &previous_state) override;
            virtual callback_return on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

            virtual callback_return on_init(const hardware_interface::HardwareInfo &hardware_info) override;
            virtual std::vector<hardwarw_interface::StateInterface> export_state_interfaces() override;
            virtual std::vector<hardwarw_interface::CommandInterface> export_command_interfaces() override;

            virtual hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration & period) override;
            virtual hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        private:
            LibSerial::SerialPort arduino_port;
            std::string port_;

            std::vector<double> position_commands;
            std::vector<double> prev_position_commands;
            std::vector<double> position_states;

    };
}

#endif