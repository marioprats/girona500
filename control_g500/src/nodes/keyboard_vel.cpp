/*
 * keyboard_vel.cpp
 *
 *  Created on: 22/05/2011
 *  updated on: 11/01/2012
 *      Author: arnau & narcis
 */

//TODO: MIRAR EL CODI QUE HI HA A: "teleop_pr2_keyboard.cpp" del paquet standard del PR2 de ROS

#include <boost/thread/mutex.hpp>
#include <termios.h>
#include <stdio.h>
#include <signal.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include <boost/thread.hpp>
#include "auv_msgs/NavSts.h"
#include "sensor_msgs/JointState.h"
#include <math.h> 
#define _USE_MATH_DEFINES

struct JoyConfig {
	double min_x_v;
	double max_x_v;
	double inc_x;
	double min_y_v;
	double max_y_v;
	double inc_y;
	double min_z_v;
	double max_z_v;
	double inc_z;
	double min_yaw_v;
	double max_yaw_v;
	double inc_yaw;
  //joints arm
        double min_arm_v;
	double max_arm_v;
	double inc_arm;
	double min_hand_v;
	double max_hand_v;
	double inc_hand;
	double min_support_v;
	double max_support_v;
	double inc_support;
};


const int _NUMBER_OF_SETPOINTS_ = 6;
const int _NUMBER_OF_JOINTS_ = 3;
JoyConfig _config;

std::vector< int > _pressed_keys ;
std::vector< float > _setpoints ;
std::vector< float > _setjoints ;

boost::mutex _setpoints_mutex;
//boost::mutex _setjoints_mutex;
boost::shared_ptr< boost::thread > _reading_thread ;

bool _reading_on = false ;
bool _shutdown_thread = false;
struct termios oldt, newt ;
ros::Publisher _pub_ack_teleop ;


/**
 * Get the configuration from an xml file. The data is set in xmlMapper.
 * Like in the previous COLA2
 *
 * @author Arnau Carrera
 * @date 22/05/2011
 */
void getConfig() ;


/**
 *This method configure the standard input to read the key pressed
 *and after reading sets the configuration again.
 *
 * @author Arnau Carrera
 * @date 22/05/2011
 */
int getChar() ;


/**
 *This function is executed in a independent thread.
 *It reads the key pressed using gethChar and do the action
 *needed in the setpoints vector
 *
 * @author Arnau Carrera
 * @date 22/05/2011
 */
void readKeyboardHits( ) ;


/**
 *This function is executed in every loop_rate in the main.
 *It generates the message to send to the teleoperation and
 *publish it.
 *
 * @author Arnau Carrera
 * @date 22/05/2011
 */
void publishSetpoints( ros::Publisher& pub ) ;


/**
 *This function is executed in every loop_rate in the main.
 *It generates the message to send to the arm controller.
 *
 * @author Arnau Carrera
 * @date 27/03/2012
 */
void publishSetjoints( ros::Publisher& pub ) ;


/**
 *This function is a callback.
 *This callback is called when teleoperation publish a
 *joystick_ok( this message contains seq_num and ok ).
 *The function read this message and answer with ack message.
 *
 * @param pub The publisher have to be the type JoystickData
 *
 * @author Arnau Carrera
 * @date 22/05/2011
 */
void joystickOkCallback(const std_msgs::String& ok_msg);

void navStsCallback( const auv_msgs::NavSts& msg );

/**
 *This functions was called when you recived a SIGTERM event.
 *When you received this, the method recover the keyboard.
 * @param ok_msg This is a string with num_seq + "ok"
 * @author Arnau Carrera
 * @date 22/05/2011
 */
void quit(int sig) ;


/**
 * Initialize all the class, and have a loop_rate where it calls the publish method.
 * @author Arnau Carrera
 * @date 22/05/2011
 */
int main(int argc, char **argv);


int
main(int argc, char **argv)
{
	ros::init(argc, argv, "keyboard", ros::init_options::NoSigintHandler ) ;

	// BEGINING INITIALIZE
	getConfig();
	ROS_INFO("Configuration Loaded correctly") ;
	_pressed_keys.push_back( 1 ) ;
	_setpoints.resize( _NUMBER_OF_SETPOINTS_ ) ;
	_setjoints.resize( _NUMBER_OF_JOINTS_ ) ;

	//Millor Així o no val la pena tenir dos Threads.
	_reading_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&readKeyboardHits)));
	ROS_INFO("Initialized thread correctly") ;
	
	// END INITIALIZE

	ros::NodeHandle n;

	//Publishers
	ros::Publisher _pub = n.advertise<sensor_msgs::Joy>("/control_g500/joystick_data", 1);
	ros::Publisher _pub_Joint = n.advertise<sensor_msgs::JointState>("/control_g500/joint_state",1);
	_pub_ack_teleop = n.advertise<std_msgs::String >("/control_g500/joystick_ack", 1);

	//Subcribers
	ros::Subscriber sub_ok = n.subscribe( "/control_g500/joystick_ok", 1, joystickOkCallback);

	signal(SIGINT, quit);

	ros::Rate loop_rate( 10.0 ) ;
	_reading_on = true ;

	while ( ros::ok() )
	{
		ros::spinOnce();
		publishSetpoints(_pub);
		publishSetjoints(_pub_Joint);
		loop_rate.sleep() ;
	}
	
	_shutdown_thread = true;
	
	return 0;
}



void
getConfig(){
	if(!ros::param::getCached("joy/min_x_v", _config.min_x_v)) {ROS_FATAL("Invalid parameters for joy/min_x_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/max_x_v", _config.max_x_v)) {ROS_FATAL("Invalid parameters for joy/max_x_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/x_inc", _config.inc_x)) {ROS_FATAL("Invalid parameters for joy/x_inc in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/min_y_v", _config.min_y_v)) {ROS_FATAL("Invalid parameters for joy/min_y_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/max_y_v", _config.max_y_v)) {ROS_FATAL("Invalid parameters for joy/max_y_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/y_inc", _config.inc_y)) {ROS_FATAL("Invalid parameters for joy/y_inc in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/min_z_v", _config.min_z_v)) {ROS_FATAL("Invalid parameters for joy/min_z_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/max_z_v", _config.max_z_v)) {ROS_FATAL("Invalid parameters for joy/max_z_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/z_inc", _config.inc_z)) {ROS_FATAL("Invalid parameters for joy/z_inc in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/min_yaw_v", _config.min_yaw_v)) {ROS_FATAL("Invalid parameters for joy/min_yaw_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/max_yaw_v", _config.max_yaw_v)) {ROS_FATAL("Invalid parameters for joy/max_yaw_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/yaw_inc", _config.inc_yaw)) {ROS_FATAL("Invalid parameters for joy/yaw_inc in param server!"); ros::shutdown();}

	if(!ros::param::getCached("joint/min_arm_v", _config.min_arm_v)) {ROS_FATAL("Invalid parameters for joint/min_arm_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joint/max_arm_v", _config.max_arm_v)) {ROS_FATAL("Invalid parameters for joint/max_arm_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joint/arm_inc", _config.inc_arm)) {ROS_FATAL("Invalid parameters for AKI joint/arm_inc in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joint/min_hand_v", _config.min_hand_v)) {ROS_FATAL("Invalid parameters for joint/min_hand_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joint/max_hand_v", _config.max_hand_v)) {ROS_FATAL("Invalid parameters for joint/max_hand_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joint/hand_inc", _config.inc_hand)) {ROS_FATAL("Invalid parameters for joint/hand_inc in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joint/min_support_v", _config.min_support_v)) {ROS_FATAL("Invalid parameters for joint/min_support_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joint/max_support_v", _config.max_support_v)) {ROS_FATAL("Invalid parameters for joint/max_support_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joint/support_inc", _config.inc_support)) {ROS_FATAL("Invalid parameters for joint/support_inc in param server!"); ros::shutdown();}


	
}



int
getChar()
{
	int ch ;
	tcgetattr( STDIN_FILENO, &oldt ) ;
	//newt = oldt ;
	memcpy(&newt, &oldt, sizeof(struct termios));
	newt.c_lflag &= ~( ICANON | ECHO ) ;
	newt.c_cc[VEOL] = 1;
	newt.c_cc[VEOF] = 2;
	tcsetattr( STDIN_FILENO, TCSANOW, &newt ) ;
	ch = getchar() ;
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt ) ;
	return ch ;
}



void
readKeyboardHits( ) {

	while(!_shutdown_thread)
	{
		const size_t X = 0 ;
		const size_t Y = 1 ;
		const size_t Z = 2 ;
//		const size_t Roll = 3 ;
//		const size_t Pitch = 4 ;
		const size_t YAW = 5 ;

		const size_t ARM = 0 ;
		const size_t HAND = 1 ;
		const size_t SUPPORT = 2 ; 

		//Forward, backward, turn left and turn right
		const int KEY_W = 87 ;
		const int KEY_w = 119 ;
		const int KEY_S = 83 ;
		const int KEY_s = 115 ;
		const int KEY_A = 65 ;
		const int KEY_a = 97 ;
		const int KEY_D = 68 ;
		const int KEY_d = 100 ;

		//Up and down
		const int KEY_UP = 65 ;
		const int KEY_LEFT = 68 ;
		const int KEY_RIGHT = 67 ;
		const int KEY_DOWN = 66 ;

		//Join 0 ARM
		// possitive increment
		const int KEY_O = 79 ;
		const int KEY_o = 111 ;
		// negative increment
		const int KEY_L = 76 ;
		const int KEY_l = 108 ;

		//Join 1 HAND
		// possitive increment
		const int KEY_I = 73 ;
		const int KEY_i = 105 ;
		// negative increment
		const int KEY_K = 75 ;
		const int KEY_k = 107 ;
		
		//Join 2 SUPPORT
		// possitive increment
		const int KEY_U = 85 ;
		const int KEY_u = 117 ;
		// negative increment
		const int KEY_J = 74 ;
		const int KEY_j = 106 ;
		
		
		//Esc and Space control actions
		const int KEY_ESC = 27 ;
		const int KEY_OBRACKET = 91 ;
		const int KEY_SPACE = 32 ;

//		const int KEY_P = 80 ;
//		const int KEY_p = 112 ;

		int key = getChar() ;

		_pressed_keys.push_back( key ) ;

		boost::mutex::scoped_lock lock( _setpoints_mutex ) ;
		//boost::mutex::scoped_lock lock( _setjoints_mutex ) ;

		if ( key == KEY_ESC ) {

			if ( getChar() == KEY_OBRACKET ) {
				int arrow_key = getChar() ;
				if ( arrow_key == KEY_UP ) {
					_setpoints[ Z ] -= _config.inc_z;
				}
				else if ( arrow_key == KEY_LEFT ) {
					_setpoints[ Y ] -= _config.inc_y;
				}
				else if ( arrow_key == KEY_RIGHT ) {
					_setpoints[ Y ] += _config.inc_y;
				}
				else if ( arrow_key == KEY_DOWN ) {
					_setpoints[ Z ] += _config.inc_z;
				}
			}
		}
		else if ( key == KEY_W || key == KEY_w ) {
			_setpoints[ X ] += _config.inc_x;
		}
		else if ( key == KEY_S || key == KEY_s ) {
			_setpoints[ X ] -= _config.inc_x;
		}
		else if ( key == KEY_D || key == KEY_d ) {
			_setpoints[YAW] += _config.inc_yaw;
		}
		else if ( key == KEY_A || key == KEY_a ) {
			_setpoints[YAW] -= _config.inc_yaw;
		}
		else if ( key == KEY_O || key == KEY_o ) {
		        _setjoints[ARM] += _config.inc_arm;
		}
		else if ( key == KEY_L || key == KEY_l ) {
		        _setjoints[ARM] -= _config.inc_arm;
		}
		else if ( key == KEY_I || key == KEY_i ) {
		        _setjoints[HAND] += _config.inc_hand;
		}
		else if ( key == KEY_K || key == KEY_k ) {
		        _setjoints[HAND] -= _config.inc_hand;
		}
		else if ( key == KEY_U || key == KEY_u ) {
		        _setjoints[SUPPORT] += _config.inc_support;
		}
		else if ( key == KEY_J || key == KEY_j ) {
		        _setjoints[SUPPORT] -= _config.inc_support;
		}
		else if ( key == KEY_SPACE ) {
			//std::fill( _setpoints.begin(), _setpoints.end(), 0.0 );
			_setpoints[X] = 0.0;
			_setpoints[Y] = 0.0;
			_setpoints[Z] = 0.0;
			_setpoints[YAW] = 0.0;

			_setjoints[ARM] = 0.0;
			_setjoints[HAND] = 0.0;
			_setjoints[SUPPORT] = 0.0;
		}
		else {
			ROS_INFO( "Keycode is: %d. Ignored Key.", key );
		}

		//Saturate
		if(_setpoints[X] > _config.max_x_v) _setpoints[X] = _config.max_x_v;
		else if(_setpoints[X] < _config.min_x_v) _setpoints[X] = _config.min_x_v;
		 
		if(_setpoints[Y] > _config.max_y_v) _setpoints[Y] = _config.max_y_v;
		else if(_setpoints[Y] < _config.min_y_v) _setpoints[Y] = _config.min_y_v;
				
		if(_setpoints[Z] > _config.max_z_v) _setpoints[Z] = _config.max_z_v;
		else if(_setpoints[Z] < _config.min_z_v) _setpoints[Z] = _config.min_z_v;

		if(_setpoints[YAW] > _config.max_yaw_v) _setpoints[YAW] = _config.max_yaw_v;
		else if(_setpoints[YAW] < _config.min_yaw_v) _setpoints[YAW] = _config.min_yaw_v;

		//Joint Saturate
		
		if(_setjoints[ARM] > _config.max_arm_v) _setjoints[ARM] = _config.max_arm_v;
		else if(_setjoints[ARM] < _config.min_arm_v) _setjoints[ARM] = _config.min_arm_v;
		 
		if(_setjoints[HAND] > _config.max_hand_v) _setjoints[HAND] = _config.max_hand_v;
		else if(_setjoints[HAND] < _config.min_hand_v) _setjoints[HAND] = _config.min_hand_v;
				
		if(_setjoints[SUPPORT] > _config.max_support_v) _setjoints[SUPPORT] = _config.max_support_v;
		else if(_setjoints[SUPPORT] < _config.min_support_v) _setjoints[SUPPORT] = _config.min_support_v;


				
	}
}


void
publishSetpoints( ros::Publisher& pub )
{
	boost::mutex::scoped_lock lock( _setpoints_mutex ) ;

	sensor_msgs::Joy msg ;
	msg.axes.reserve(_setpoints.size());
	copy(_setpoints.begin(), _setpoints.end(), back_inserter(msg.axes));

	msg.buttons.reserve(_pressed_keys.size());
	copy(_pressed_keys.begin(),_pressed_keys.end(),back_inserter(msg.buttons));

	msg.header.stamp = ros::Time::now() ;
	pub.publish( msg ) ;

	_pressed_keys.clear() ;
	_pressed_keys.resize(1) ;
}


void
publishSetjoints( ros::Publisher& pub )
{
	boost::mutex::scoped_lock lock( _setpoints_mutex ) ;

	sensor_msgs::JointState msg ;

	msg.position.reserve(_setjoints.size()) ;
	copy(_setjoints.begin(), _setjoints.end(), back_inserter(msg.position)) ;

	msg.velocity.reserve(_setjoints.size()) ;
	copy(_setjoints.begin(), _setjoints.end(), back_inserter(msg.velocity)) ;

	msg.header.stamp = ros::Time::now() ;
	pub.publish( msg ) ;

	_pressed_keys.clear() ;
	_pressed_keys.resize(1) ;
}


void
joystickOkCallback( const std_msgs::String& ok_msg )
{
	std::istringstream iss( ok_msg.data ) ;
	std::string msg;
	unsigned int seq ;
	iss >> seq ;
	iss >> msg ;
	if ( "ok" == msg ) {
		std_msgs::String msg ;
		msg.data = boost::lexical_cast<std::string>(seq+1) + " ack" ;
		_pub_ack_teleop.publish( msg ) ;
	}
}


void
quit(int sig)
{
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	ros::shutdown();
	exit(0);
}
