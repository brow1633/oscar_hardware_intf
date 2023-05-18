#include "oscar_hardware_intf/oscar_simulation_intf.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <rclcpp/executors.hpp>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "oscar_ros_msgs/msg/wheel_speeds.hpp"

#include <string>

#include <stdio.h>
#include <string.h>
using std::placeholders::_1;

namespace oscar_hardware_intf
{
    WheelSpeedPubSub::WheelSpeedPubSub(std::string node_name, std::string pub_name, std::string sub_name) : Node(node_name)
    {
        publisher_ = this->create_publisher<oscar_ros_msgs::msg::WheelSpeeds>(pub_name, 10);
        subscription_ = this->create_subscription<oscar_ros_msgs::msg::WheelSpeeds>(sub_name, 10, std::bind(&WheelSpeedPubSub::sub_callback, this, _1));
    }

    void WheelSpeedPubSub::publish(oscar_ros_msgs::msg::WheelSpeeds& msg)
    {
        publisher_->publish(msg);
    }

    void WheelSpeedPubSub::sub_callback(const oscar_ros_msgs::msg::WheelSpeeds::SharedPtr msg)
    {
        currWheelSpeeds = *msg;
        //RCLCPP_INFO(rclcpp::get_logger("OscarSimulationIntf"), "GOt wheel speed of %.5f", msg->fl);
    }

    float WheelSpeedPubSub::get_wheel_speed(int WheelIdx)
    {
        switch(WheelIdx)
        {
            case 0:
                return currWheelSpeeds.fl;
            case 1:
                return currWheelSpeeds.fr;
            case 2:
                return currWheelSpeeds.rl;
            case 3:
                return currWheelSpeeds.rr;
            default:
                return 0;
        }
    }

	hardware_interface::CallbackReturn OscarSimulationIntf::on_init(const hardware_interface::HardwareInfo & info)
	{
        node = std::make_shared<WheelSpeedPubSub>("simulation_intf_wheelspeeds", "target_wheel_speeds", "measured_wheel_speeds");

		if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
		{
			return hardware_interface::CallbackReturn::ERROR;
		}

		base_x_ = 0.0;
		base_y_ = 0.0;
		base_theta_ = 0.0;

		hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
		hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
		hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

		for(const hardware_interface::ComponentInfo & joint: info_.joints)
		{

			if(joint.command_interfaces.size() != 1)
			{
				RCLCPP_FATAL(
						rclcpp::get_logger("OscarSimulationIntf"),
						"Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
						joint.command_interfaces.size());
				return hardware_interface::CallbackReturn::ERROR;
			}

			if(joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
			{
				RCLCPP_FATAL(
						rclcpp::get_logger("OscarSimulationIntf"),
						"Joint '%s' has '%s' command interfaces found. '%s' expected.", joint.name.c_str(),
						joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
				return hardware_interface::CallbackReturn::ERROR;
			}

			if(joint.state_interfaces.size() != 2)
			{
				RCLCPP_FATAL(
						rclcpp::get_logger("OscarSimulationIntf"),
						"Joint '%s' has %zu state interfaces found. 1 expected.", joint.name.c_str(),
						joint.state_interfaces.size());
				return hardware_interface::CallbackReturn::ERROR;
			}

			if(joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
			{
				RCLCPP_FATAL(
						rclcpp::get_logger("OscarSimulationIntf"),
						"Joint '%s' has '%s' state interfaces found. '%s' expected.", joint.name.c_str(),
						joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
				return hardware_interface::CallbackReturn::ERROR;
			}
			RCLCPP_INFO(rclcpp::get_logger("OscarSimulationIntf"),
					"4"
					);
			if(joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
			{
				RCLCPP_FATAL(
						rclcpp::get_logger("OscarSimulationIntf"),
						"Joint '%s' has '%s' state interfaces found. '%s' expected.", joint.name.c_str(),
						joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
				return hardware_interface::CallbackReturn::ERROR;
			}
			RCLCPP_INFO(rclcpp::get_logger("OscarSimulationIntf"),
					"5"
					);
		}
		RCLCPP_INFO(rclcpp::get_logger("OscarSimulationIntf"),
				"Init completed successfully"
				);
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	std::vector<hardware_interface::StateInterface> OscarSimulationIntf::export_state_interfaces()
	{
		std::vector<hardware_interface::StateInterface> state_interfaces;
		for(auto i = 0u; i < info_.joints.size(); i++)
		{
			state_interfaces.emplace_back(hardware_interface::StateInterface(
						info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
			state_interfaces.emplace_back(hardware_interface::StateInterface(
						info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
		}
		return state_interfaces;
	}

	std::vector<hardware_interface::CommandInterface> OscarSimulationIntf::export_command_interfaces()
	{
		std::vector<hardware_interface::CommandInterface> command_interfaces;
		for (auto i = 0u; i < info_.joints.size(); i++)
		{
			command_interfaces.emplace_back(hardware_interface::CommandInterface(
						info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
		}

		return command_interfaces;
	}

	hardware_interface::CallbackReturn OscarSimulationIntf::on_activate(const rclcpp_lifecycle::State &)
	{
		RCLCPP_INFO(rclcpp::get_logger("OscarSimulationIntf"), "Activating HW Intf...");
		for(auto i=0u; i < hw_positions_.size(); i++)
		{
			if(std::isnan(hw_positions_[i]))
			{
				hw_positions_[i] = 0;
				hw_velocities_[i] = 0;
				hw_commands_[i] = 0;
			}
		}
		RCLCPP_INFO(rclcpp::get_logger("OscarSimulationIntf"), "success");
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::CallbackReturn OscarSimulationIntf::on_deactivate(
			const rclcpp_lifecycle::State & /*previous_state*/)
	{

		RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::return_type OscarSimulationIntf::read(const rclcpp::Time &, const rclcpp::Duration & period)
	{
		/*for (uint8_t i = 0; i < hw_commands_.size(); i++)
		{
			hw_velocities_[i] = (counts[i] / encoder_cpr) * micro_run_freq * 6.28;
			hw_positions_[i] += hw_velocities_[i] * period.seconds();

			// BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
			RCLCPP_INFO(
					rclcpp::get_logger("DiffBotSystemHardware"),
					"Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
					hw_velocities_[i], info_.joints[i].name.c_str());
			// END: This part here is for exemplary purposes - Please do not copy to your production code
		}*/
        rclcpp::spin_some(node);
        for(int i = 0; i < hw_commands_.size(); i++)
        {
            hw_velocities_[i] = node->get_wheel_speed(i) * 6.28 / 60;
            hw_positions_[i] += hw_velocities_[i] * period.seconds();
        }

		return hardware_interface::return_type::OK;
	}
	hardware_interface::return_type OscarSimulationIntf::write(const rclcpp::Time &, const rclcpp::Duration &)
	{

		/*for(auto i=0u; i < hw_commands_.size(); i++)
		{
			RCLCPP_INFO(rclcpp::get_logger("OscarSimulationIntf"), "Got command %.5f for '%s'!", hw_commands_[i], info_.joints[i].name.c_str());
		}*/
		oscar_ros_msgs::msg::WheelSpeeds msg;

        std::vector<float> cmds;
		for(int i = 0; i < 4; i++) {
			cmds.push_back(hw_commands_[i] / (6.28) * 60); 
		}

        msg.fl = cmds[0];
        msg.fr = cmds[1];
        msg.rl = cmds[2];
        msg.rr = cmds[3];

        node->publish(msg);

		//RCLCPP_INFO(rclcpp::get_logger("OscarSimulationIntf"), "Joint States Written");
		return hardware_interface::return_type::OK;
	}


}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(oscar_hardware_intf::OscarSimulationIntf, hardware_interface::SystemInterface)
