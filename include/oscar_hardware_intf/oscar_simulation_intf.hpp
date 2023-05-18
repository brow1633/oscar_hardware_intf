#ifndef OSCAR_HARDWARE_INTF_HPP
#define OSCAR_HARDWARE_INTF_HPP
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "oscar_ros_msgs/msg/wheel_speeds.hpp"

namespace oscar_hardware_intf
{	
    class WheelSpeedPubSub : public rclcpp::Node
    {
        public:
            WheelSpeedPubSub(std::string node_name, std::string pub_name, std::string sub_name);
            void publish(oscar_ros_msgs::msg::WheelSpeeds& msg);
            float get_wheel_speed(int WheelIdx);
        private:
            void sub_callback(const oscar_ros_msgs::msg::WheelSpeeds::SharedPtr msg);
            rclcpp::Publisher<oscar_ros_msgs::msg::WheelSpeeds>::SharedPtr publisher_;
            rclcpp::Subscription<oscar_ros_msgs::msg::WheelSpeeds>::SharedPtr subscription_;
            oscar_ros_msgs::msg::WheelSpeeds currWheelSpeeds;
    };

	class OscarSimulationIntf : public hardware_interface::SystemInterface {
	public:
		RCLCPP_SHARED_PTR_DEFINITIONS(OscarSimulationIntf)

		hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
		std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
		std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
		hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
		hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
		hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
		hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

	private:
		float last_pos;

		std::vector<double> hw_commands_;
		std::vector<double> hw_positions_;
		std::vector<double> hw_velocities_;

        std::shared_ptr<WheelSpeedPubSub> node;

		double base_x_, base_y_, base_theta_;
	};
}

#endif
