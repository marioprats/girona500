/*
 *  ocean_server_batteries.cpp
 *
 *  Created : 08/07/2010
 *  Author Narcis Palomeras
 */

#include "ros/ros.h"
#include <cola2_lib/cola2_io/SerialPort.h>
#include "safety_g500/BatteryLevel.h"
#include <string>

class OceanServerBatteries {

private:

	struct DeviceConfig {
		int device_timeout;

		// ------- Serial Port Config -------
		std::string sp_path ;
		int sp_baud_rate ;
		int sp_char_size ;
		int sp_stop_bits ;
		std::string sp_parity ;
		std::string sp_flow_control ;
		int sp_timeout ;
	};

	safety_g500::BatteryLevel _battery_level_data;
	cola2::io::SerialPort _serial_port;
	DeviceConfig _config;
	ros::NodeHandle _n;
	ros::Publisher _pub_data;
	int _controller;
	
public:
	OceanServerBatteries() { }
	~OceanServerBatteries() { }

	void init() {
		//get configuration
		getConfig();
		_controller = -1;
		//cola2::io::serialPort _serial_port ;
		setUpSerialPort();
		_pub_data = _n.advertise<safety_g500::BatteryLevel> ("/safety_g500/ocean_server_batteries", 1);
	}

	/**
	 * 	Take serial port parameters from ROS param server
	 */

	void
	getConfig( )
	{
		if(!ros::param::getCached("ocean_server_batteries/device_timeout", _config.device_timeout)) {ROS_FATAL("Invalid parameters for ocean_server_batteries/device_timeout in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ocean_server_batteries/sp_path", _config.sp_path)) {ROS_FATAL("Invalid parameters for ocean_server_batteries/sp_path in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ocean_server_batteries/sp_baud_rate", _config.sp_baud_rate)) {ROS_FATAL("Invalid parameters for ocean_server_batteries/sp_baud_rate in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ocean_server_batteries/sp_char_size", _config.sp_char_size)) {ROS_FATAL("Invalid parameters for ocean_server_batteries/sp_char_size in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ocean_server_batteries/sp_stop_bits", _config.sp_stop_bits)) {ROS_FATAL("Invalid parameters for ocean_server_batteries/sp_stop_bits in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ocean_server_batteries/sp_parity", _config.sp_parity)) {ROS_FATAL("Invalid parameters for ocean_server_batteries/sp_parity in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ocean_server_batteries/sp_flow_control", _config.sp_flow_control)) {ROS_FATAL("Invalid parameters for ocean_server_batteries/sp_flow_control in param server!"); ros::shutdown();}
		if(!ros::param::getCached("ocean_server_batteries/sp_timeout", _config.sp_timeout)) {ROS_FATAL("Invalid parameters for ocean_server_batteries/sp_timeout in param server!"); ros::shutdown();}
	}


	/**
		 * Set up the serial port with the configuration loaded from the configuration
		 *
		 */
	void
	setUpSerialPort( )
	{
			try {
					_serial_port.open(_config.sp_path);
					_serial_port.setBaudRate(cola2::io::SerialPort::baudRateFromInteger(_config.sp_baud_rate));
					_serial_port.setCharSize(cola2::io::SerialPort::charSizeFromInteger(_config.sp_char_size));
					_serial_port.setNumOfStopBits(cola2::io::SerialPort::numOfStopBitsFromInteger(_config.sp_stop_bits));
					_serial_port.setParity(cola2::io::SerialPort::parityFromString(_config.sp_parity));
					_serial_port.setFlowControl(cola2::io::SerialPort::flowControlFromString(_config.sp_flow_control));
			}
			catch( std::exception& e) {
					ROS_ERROR("Error setting up the serial port!");
					ros::shutdown() ;
			}
	}


	/**
		*  Used to interpret the raw information received from the device and store it to the
		*  according data manipulators.
		*
		*  @param line		Raw line read from the device.
		*  @return true if successful, false otherwise.
		*  @date 08/07/2010
		*/
	bool
	tryLoadReading( const std::string& line )
	{
		try {
			//Identify the correct string
			size_t found = line.find("*");
			if (found != std::string::npos) {
				readStatus(line);
			    _battery_level_data.header.stamp = ros::Time::now();
			    _battery_level_data.header.frame_id = "no_frame";
			    _pub_data.publish(_battery_level_data);
			}
			else {
				size_t found = line.find("Cluster:");
				if (found != std::string::npos) {
					readCluster(line);
				}
				else {
					size_t found = line.find("Controller:");
					if (found != std::string::npos) {
						std::istringstream iss(line);
						//Parse it
						std::string temp; 			    
						iss >> temp;
						iss >> temp;
						_controller = atoi(temp.c_str());
						//std::cout << "Controller: " << _controller << std::endl;
					}
					else{
						size_t found = line.find("B(");
						if (found != std::string::npos) {
							readBattery(line);
						}					
					}
				}
			}
//			else ROS_INFO("--> %s", line.c_str());

			return true;
		}
		catch ( std::exception& e ) { ROS_WARN_STREAM(  "Error parsing line: " << e.what() ) ; return false ; }
	}

	void
	readBattery(std::string my_line){
		//Clean it 
		std::replace(my_line.begin(), my_line.end(), '(', ' ');
		std::replace(my_line.begin(), my_line.end(), ')', ' ');
		std::replace(my_line.begin(), my_line.end(), '=', ' ');
									
		//ROS_INFO("(bat)%s", my_line.c_str());
		std::istringstream iss(my_line);
		
		
		//B 1   V 16.58 I -0.13 S: D - -  | B 2   V 16.58 I -0.12 S: D - - 

		//Parse it
		std::string temp; 			    
		std::string bat_1_status; 			    
		std::string bat_2_status; 			    
																
		iss >> temp;
		iss >> temp;
		int bat_1 = atoi(temp.c_str());
		iss >> temp;
		iss >> temp;
		iss >> temp;
		iss >> temp;
		iss >> temp;
		iss >> temp;
		iss >> bat_1_status;
		if (bat_1_status == "-") {
			iss >> temp; //Comprova si s'ha llegit (- - | AC- | NG)
			//std::cout << "llegit - llegirem un altre " << temp << std::endl;
		}
		//std::cout << "Bat " << bat_1 << ": " << bat_1_status << std::endl;
		iss >> temp;
		iss >> temp;
		iss >> temp;
		int bat_2 = atoi(temp.c_str());
		iss >> temp;
		iss >> temp;
		iss >> temp;
		iss >> temp;
		iss >> temp;
		iss >> temp;
		iss >> bat_2_status;
		//std::cout << "Bat " << bat_2 << ": " << bat_2_status << std::endl;
		
		if (_controller == 1){
			_battery_level_data.controller_1[bat_1-1] = bat_1_status;
			_battery_level_data.controller_1[bat_2-1] = bat_2_status;
		}
		else {
			if (_controller == 2){
					_battery_level_data.controller_2[bat_1-1] = bat_1_status;
					_battery_level_data.controller_2[bat_2-1] = bat_2_status;
			}
			else {
				if (_controller == 3){
					_battery_level_data.controller_3[bat_1-1] = bat_1_status;
					_battery_level_data.controller_3[bat_2-1] = bat_2_status;
				}	
			}
		}
	}
	
	void
	readStatus(std::string my_line){
		//Clean it 
		std::replace(my_line.begin(), my_line.end(), '*', ' ');
		std::replace(my_line.begin(), my_line.end(), ',', ' ');
		std::replace(my_line.begin(), my_line.end(), '%', ' ');
		std::replace(my_line.begin(), my_line.end(), ':', ' ');
		//ROS_INFO("(status)%s", my_line.c_str());
						
		std::istringstream iss(my_line);
		
		//Parse it
		std::string temp; 			    
		std::string status; 			    
						
		iss >> status;
		//std::cout << "Status: " << status << std::endl;
		iss >> temp;
		//std::cout << "Mins: " << temp << std::endl;
		int minutes = atoi(temp.c_str());
		iss >> temp;
		iss >> temp;
		iss >> temp;
		float charge = atof(temp.c_str());
		//std::cout << "Capacity: " << charge << std::endl;
		_battery_level_data.charge = charge;
		if(status=="CHARGING"){
			iss >> temp;
			iss >> temp;
			int packs = atoi(temp.c_str());	    
			//std::cout << "Packs: " << packs << std::endl;
			iss >> temp;
			iss >> temp;
			iss >> temp;
			int cycles = atoi(temp.c_str());
			//std::cout << "Cycles: " << cycles << std::endl;
			_battery_level_data.packs = packs;
			_battery_level_data.cycles = cycles;
		} 
		else {
			iss >> temp;
			iss >> temp;
			int full_discharge = atoi(temp.c_str());	    
			//std::cout << "FD: " << full_discharge << std::endl;
			iss >> temp;
			iss >> temp;
			int full_charge = atoi(temp.c_str());
			//std::cout << "FC: " << full_charge << std::endl;
			_battery_level_data.full_discharge = full_discharge;
			_battery_level_data.full_charge = full_charge;
		}			    
		_battery_level_data.status = status;
		_battery_level_data.minutes = minutes;
	}
	
	
	void
	readCluster(std::string my_line){
		//Clean it 
		std::replace(my_line.begin(), my_line.end(), ',', ' ');
		std::replace(my_line.begin(), my_line.end(), '%', ' ');
		std::replace(my_line.begin(), my_line.end(), ':', ' ');
		//ROS_INFO("(cluster)%s", my_line.c_str());
						
		std::istringstream iss(my_line);	
		//Parse it
		std::string temp; 			    
		iss >> temp;
		iss >> temp;
		iss >> temp;
		iss >> temp;
		float amps = atof(temp.c_str());
		//std::cout << "Amps: " << amps << std::endl;
		iss >> temp;
		iss >> temp;
		float watts = atof(temp.c_str());
		//std::cout << "Watts: " << watts << std::endl;
		iss >> temp;
		iss >> temp;
		iss >> temp;
		iss >> temp;
		float volts = atof(temp.c_str());
		//std::cout << "Volts: " << volts << std::endl;
		_battery_level_data.amps = amps;
		_battery_level_data.watts = watts;
		_battery_level_data.volts = volts;
	}
	
	/*
	 * Read a line from the serial port, parse it, and publish the data.
	 *
	 */
	void iterate()
	{
		try {
			std::string line = _serial_port.readLine(_config.device_timeout);
			tryLoadReading(line);
		}
		catch(cola2::io::SerialPort::NotOpen&){ ROS_WARN("Serial port is not open."); }
		catch(cola2::io::SerialPort::ReadTimeout&){ ROS_WARN("I/O device reading timeout"); }
		catch(std::runtime_error&){ ROS_WARN("Serial port I/O error."); }
	}
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "ocean_server_batteries");
	OceanServerBatteries ocean_server_batteries;
	ocean_server_batteries.init();

	while (ros::ok())
	{
		ros::spinOnce();
		ocean_server_batteries.iterate();
	}
}
