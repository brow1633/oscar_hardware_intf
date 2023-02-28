#ifndef OSCAR_HARDWARE_INTF_HPP
#define OSCAR_HARDWARE_INTF_HPP
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <stdint.h>

//int handle_serial_read(int serial_port, uint8_t* read_buf, size_t max_size);

namespace oscar_hardware_intf
{	
	typedef union {
	  float num;
	  uint8_t bytes[4];
	} byteToFloat;

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
		void setup_serial();
		int serial_port;
		struct termios tty;

		float encoder_cpr;
		int micro_run_freq;
		std::string serial_port_name;
		float last_pos;

		std::vector<double> hw_commands_;
		std::vector<double> hw_positions_;
		std::vector<double> hw_velocities_;

		double base_x_, base_y_, base_theta_;
	};
}

#endif
