#include "oscar_hardware_intf/oscar_simulation_intf.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

#include <string>

#include <stdio.h>
#include <string.h>

namespace oscar_hardware_intf
{
	hardware_interface::CallbackReturn OscarSimulationIntf::on_init(const hardware_interface::HardwareInfo & info)
	{
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
		close(serial_port);

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

		return hardware_interface::return_type::OK;
	}
	hardware_interface::return_type OscarSimulationIntf::write(const rclcpp::Time &, const rclcpp::Duration &)
	{

		for(auto i=0u; i < hw_commands_.size(); i++)
		{
			RCLCPP_INFO(rclcpp::get_logger("OscarSimulationIntf"), "Got command %.5f for '%s'!", hw_commands_[i], info_.joints[i].name.c_str());
		}
		std::vector<float> cmds;

		for(int i = 0; i < 4; i++) {
			cmds.push_back(hw_commands_[i] / (6.28) * 60); // send cmds in rpm
		}

		std_msgs::msg::Float32MultiArray msg;

		// set up dimensions
		msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
		msg.layout.dim[0].size = cmds.size();
		msg.layout.dim[0].stride = 1;
		msg.layout.dim[0].label = "WheelIdx";

		// copy in the data
		msg.data.clear();
		msg.data.insert(msg.data.end(), cmds.begin(), cmds.end());


		RCLCPP_INFO(rclcpp::get_logger("OscarSimulationIntf"), "Joint States Written");
		return hardware_interface::return_type::OK;
	}


}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(oscar_hardware_intf::OscarSimulationIntf, hardware_interface::SystemInterface)
