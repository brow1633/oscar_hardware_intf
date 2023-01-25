#include "oscar_hardware_intf/oscar_hardware_intf.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>

#include <stdio.h>
#include <string.h>
#include <unistd.h> // write(), read(), close()
					//
int handle_serial_read(int serial_port, uint8_t* read_buf, size_t max_size) {
	RCLCPP_INFO(rclcpp::get_logger("OscarHardwareIntf"), "Reading..");
	return read(serial_port, read_buf, max_size);
}

void handle_serial_write(int serial_port, float* write_buff) {
	oscar_hardware_intf::byteToFloat converter;
	uint8_t startByte = 0xFF;
	write(serial_port, &startByte, sizeof(startByte));
	for(int i = 0; i < 4; i++) {
		converter.num = write_buff[i];
		RCLCPP_INFO(rclcpp::get_logger("OscarHardwareIntf"), "%s", std::to_string(converter.num).c_str());
		write(serial_port, converter.bytes, 4*sizeof(converter.bytes[0]));
	}
}



namespace oscar_hardware_intf
{
	void OscarHardwareIntf::setup_serial()
	{
		serial_port = open(serial_port_name.c_str(), O_RDWR);

		if(tcgetattr(serial_port, &tty) != 0) {
			//printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
			// TODO: THROW ERROR
			RCLCPP_FATAL(rclcpp::get_logger("OscarHardwareIntf"),
					"%s", strerror(errno)
					);
			return;
		}

		tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
		tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
		tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
		tty.c_cflag |= CS8; // 8 bits per byte (most common)
		tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
		tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

		tty.c_lflag &= ~ICANON;
		tty.c_lflag &= ~ECHO; // Disable echo
		tty.c_lflag &= ~ECHOE; // Disable erasure
		tty.c_lflag &= ~ECHONL; // Disable new-line echo
		tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
		tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
		tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

		tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
		tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
							   // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
							   // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

		tty.c_cc[VTIME] = 1;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
		tty.c_cc[VMIN] = 17;

		// Set in/out baud rate to be 9600
		cfsetispeed(&tty, B115200);
		cfsetospeed(&tty, B115200);

		// Save tty settings, also checking for error
		if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
			//printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
			RCLCPP_FATAL(rclcpp::get_logger("OscarHardwareIntf"),
					"%s", strerror(errno)
					);
			//TODO: THROW ERROR
			return;
		}
	}

	hardware_interface::CallbackReturn OscarHardwareIntf::on_init(const hardware_interface::HardwareInfo & info)
	{
		if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
		{
			return hardware_interface::CallbackReturn::ERROR;
		}

		base_x_ = 0.0;
		base_y_ = 0.0;
		base_theta_ = 0.0;

		//hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]); // how to parameter
		//hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]); // how to parameter
		RCLCPP_INFO(
				rclcpp::get_logger("OscarHardwareIntf"),
				"encoder_cpr: %s", info_.hardware_parameters["encoder_cpr"].c_str());
		RCLCPP_INFO(
				rclcpp::get_logger("OscarHardwareIntf"),
				"micro_run_freq: %s", info_.hardware_parameters["micro_run_freq"].c_str());
		RCLCPP_INFO(
				rclcpp::get_logger("OscarHardwareIntf"),
				"serial_port_name: %s", info_.hardware_parameters["serial_port"].c_str());
		encoder_cpr = std::stod(info_.hardware_parameters["encoder_cpr"]);
		micro_run_freq = std::stod(info_.hardware_parameters["micro_run_freq"]);
		serial_port_name = info_.hardware_parameters["serial_port"];
		setup_serial();

		hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
		hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
		hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

		for(const hardware_interface::ComponentInfo & joint: info_.joints)
		{

			if(joint.command_interfaces.size() != 1)
			{
				RCLCPP_FATAL(
						rclcpp::get_logger("OscarHardwareIntf"),
						"Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
						joint.command_interfaces.size());
				return hardware_interface::CallbackReturn::ERROR;
			}

			if(joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
			{
				RCLCPP_FATAL(
						rclcpp::get_logger("OscarHardwareIntf"),
						"Joint '%s' has '%s' command interfaces found. '%s' expected.", joint.name.c_str(),
						joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
				return hardware_interface::CallbackReturn::ERROR;
			}

			if(joint.state_interfaces.size() != 2)
			{
				RCLCPP_FATAL(
						rclcpp::get_logger("OscarHardwareIntf"),
						"Joint '%s' has %zu state interfaces found. 1 expected.", joint.name.c_str(),
						joint.state_interfaces.size());
				return hardware_interface::CallbackReturn::ERROR;
			}

			if(joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
			{
				RCLCPP_FATAL(
						rclcpp::get_logger("OscarHardwareIntf"),
						"Joint '%s' has '%s' state interfaces found. '%s' expected.", joint.name.c_str(),
						joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
				return hardware_interface::CallbackReturn::ERROR;
			}
			RCLCPP_INFO(rclcpp::get_logger("OscarHardwareIntf"),
					"4"
					);
			if(joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
			{
				RCLCPP_FATAL(
						rclcpp::get_logger("OscarHardwareIntf"),
						"Joint '%s' has '%s' state interfaces found. '%s' expected.", joint.name.c_str(),
						joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
				return hardware_interface::CallbackReturn::ERROR;
			}
			RCLCPP_INFO(rclcpp::get_logger("OscarHardwareIntf"),
					"5"
					);
		}
		RCLCPP_INFO(rclcpp::get_logger("OscarHardwareIntf"),
				"Init completed successfully"
				);
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	std::vector<hardware_interface::StateInterface> OscarHardwareIntf::export_state_interfaces()
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

	std::vector<hardware_interface::CommandInterface> OscarHardwareIntf::export_command_interfaces()
	{
		std::vector<hardware_interface::CommandInterface> command_interfaces;
		for (auto i = 0u; i < info_.joints.size(); i++)
		{
			command_interfaces.emplace_back(hardware_interface::CommandInterface(
						info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
		}

		return command_interfaces;
	}

	hardware_interface::CallbackReturn OscarHardwareIntf::on_activate(const rclcpp_lifecycle::State &)
	{
		RCLCPP_INFO(rclcpp::get_logger("OscarHardwareIntf"), "Activating HW Intf...");
		for(auto i=0u; i < hw_positions_.size(); i++)
		{
			if(std::isnan(hw_positions_[i]))
			{
				hw_positions_[i] = 0;
				hw_velocities_[i] = 0;
				hw_commands_[i] = 0;
			}
		}
		RCLCPP_INFO(rclcpp::get_logger("OscarHardwareIntf"), "success");
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::CallbackReturn OscarHardwareIntf::on_deactivate(
			const rclcpp_lifecycle::State & /*previous_state*/)
	{

		RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");
		close(serial_port);

		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::return_type OscarHardwareIntf::read(const rclcpp::Time &, const rclcpp::Duration & period)
	{
		uint8_t read_buf[20];
		float counts[4] = {0};

		int num_bytes = handle_serial_read(serial_port, read_buf, sizeof(read_buf));

		//TODO: make these like timeouts
		if(read_buf[0] != 0xff) { return hardware_interface::return_type::OK; } // sanity check
		if(num_bytes != 17) { return hardware_interface::return_type::OK; }

		byteToFloat converter;
		for(int i = 0; i < 4; i++) {
			for(int j = 0; j < 4; j++) {
				converter.bytes[j] = read_buf[j+1+4*i];
			}
			counts[i] = converter.num;
		}
		for (uint8_t i = 0; i < hw_commands_.size(); i++)
		{
			hw_velocities_[i] = (counts[i] / encoder_cpr) * micro_run_freq * 6.28;
			hw_positions_[i] += hw_velocities_[i] * period.seconds();

			// BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
			RCLCPP_INFO(
					rclcpp::get_logger("DiffBotSystemHardware"),
					"Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
					hw_velocities_[i], info_.joints[i].name.c_str());
			// END: This part here is for exemplary purposes - Please do not copy to your production code
		}

		return hardware_interface::return_type::OK;
	}
	hardware_interface::return_type OscarHardwareIntf::write(const rclcpp::Time &, const rclcpp::Duration &)
	{
		RCLCPP_INFO(rclcpp::get_logger("OscarHardwareIntf"), "Writing...");

		for(auto i=0u; i < hw_commands_.size(); i++)
		{
			RCLCPP_INFO(rclcpp::get_logger("OscarHardwareIntf"), "Got command %.5f for '%s'!", hw_commands_[i], info_.joints[i].name.c_str());
		}

		float cmds[4];
		for(int i = 0; i < 4; i++) {
			cmds[i] = hw_commands_[i] * encoder_cpr / (micro_run_freq) / (6.28); // send cmds in ticks/cycle
		}

		handle_serial_write(serial_port, cmds);
		RCLCPP_INFO(rclcpp::get_logger("OscarHardwareIntf"), "Joint States Written");
		return hardware_interface::return_type::OK;
	}


}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(oscar_hardware_intf::OscarHardwareIntf, hardware_interface::SystemInterface)
