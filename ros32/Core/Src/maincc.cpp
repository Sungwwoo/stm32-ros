#include "maincc.h"

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include "motor_driver.h"
#include "imu_driver.h"

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz
#define DEBUG_LOG_FREQUENCY                    10   //hz

#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

MotorDriver motorDriver;
ImuDriver imuDriver;
ros::NodeHandle nh;

std_msgs::String str_msg;
geometry_msgs::Twist twist;
sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom;
sensor_msgs::JointState joint_states;

int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};

unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];
ros::Time current_time;
uint32_t current_offset;


char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];
char mag_frame_id[30];

char joint_state_header_frame_id[30];

const float wheel_radius = 0.1;
const float wheel_separation = 0.125;

float value[2] = {0};
bool isReceived = false;

char msg[] = "Dummy";
char confirm_receive[] = "Received cmd_vel";


void cbCmd(const geometry_msgs::Twist& data){
	value[0] = data.linear.x;
	value[1] = data.angular.z;
	motorDriver.controlMotor(wheel_radius, wheel_separation, value);
	motorDriver.readEncoder(last_diff_tick[LEFT], last_diff_tick[RIGHT]);
	isReceived = true;
}

ros::Subscriber<geometry_msgs::Twist> sub_cmd("cmd_vel", &cbCmd);


ros::Publisher pub_state("from_stm", &str_msg);
ros::Publisher imu_pub("imu", &imu_msg);
ros::Publisher odom_pub("odom", &odom);
ros::Publisher joint_states_pub("joint_states", &joint_states);

geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART3)
		nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART3)
		nh.getHardware()->reset_rbuf();
}

void setup(UART_HandleTypeDef* leftPort, UART_HandleTypeDef* rightPort, SPI_HandleTypeDef* imuPort, GPIO_TypeDef* portx, uint16_t pin)
{
	motorDriver.init(leftPort, rightPort);
	imuDriver.imu_init(imuPort, portx, pin);
	nh.initNode();
	nh.advertise(pub_state);
	nh.subscribe(sub_cmd);
}

void loop(void)
{
	nh.spinOnce();
	if (isReceived)
		str_msg.data = confirm_receive;
	else
		str_msg.data = msg;
	pub_state.publish(&str_msg);
}

bool calcOdom(double diff_time){
	/*
	float* orientation;
	double wheel_l, wheel_r;      // rotation value of wheel [rad]
	double delta_s, theta, delta_theta;
	static double last_theta = 0.0;
	double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
	double step_time;

	wheel_l = wheel_r = 0.0;
	delta_s = delta_theta = theta = 0.0;
	v = w = 0.0;
	step_time = 0.0;

	step_time = diff_time;

	if (step_time == 0)
		return false;

	wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
	wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

	if (isnan(wheel_l))
	wheel_l = 0.0;

	if (isnan(wheel_r))
	wheel_r = 0.0;

	delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
	// theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
	orientation = sensors.getOrientation();
	theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3],
				0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

	delta_theta = theta - last_theta;

	// compute odometric pose
	odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
	odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
	odom_pose[2] += delta_theta;

	// compute odometric instantaneouse velocity

	v = delta_s / step_time;
	w = delta_theta / step_time;

	odom_vel[0] = v;
	odom_vel[1] = 0.0;
	odom_vel[2] = w;

	last_velocity[LEFT]  = wheel_l / step_time;
	last_velocity[RIGHT] = wheel_r / step_time;
	last_theta = theta;
	*/
	return true;

}

