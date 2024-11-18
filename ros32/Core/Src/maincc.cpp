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


// Log
char log_msg[100];

// Timer for controlling publish rate
static uint32_t tTime[3];

// Drivers
MotorDriver motorDriver;
ImuDriver imuDriver;

// Nodehandler
ros::NodeHandle nh;
uint32_t current_offset;
ros::Time current_time;

// ros msgs
std_msgs::String str_msg;
geometry_msgs::Twist twist;
sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom;
sensor_msgs::MagneticField mag_msg;
sensor_msgs::JointState joint_states;
geometry_msgs::TransformStamped odom_tf;

// Encoder data
bool init_encoder = true;
int32_t left_encoder, right_encoder;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double last_rad[WHEEL_NUM] = {0.0, 0.0};

// tick
unsigned long prev_update_time;

// Odometry
float odom_pose[3];
double odom_vel[3];

// Ros Params
char get_prefix[10];
char *get_tf_prefix = get_prefix;
char odom_header_frame_id[30];
char odom_child_frame_id[30];
char imu_frame_id[30];
char mag_frame_id[30];
char joint_state_header_frame_id[30];
double last_velocity[WHEEL_NUM] = {0.0, 0.0};

// Robot Wheel Information
const float wheel_radius = 0.1;
const float wheel_separation = 0.125;

// cmd_vel values
float goal_velocity[2] = {0};

// For cmd_vel receive check
bool isReceived = false;
char msg[] = "Dummy";
char confirm_receive[] = "Received cmd_vel";

// cmd_vel callback
void cbCmd(const geometry_msgs::Twist &data){
    goal_velocity[LINEAR] = data.linear.x;
    goal_velocity[ANGULAR] = data.angular.z;
    // motorDriver.controlMotor(wheel_radius, wheel_separation, value);
    // motorDriver.readEncoder(last_diff_tick[LEFT], last_diff_tick[RIGHT]);
    isReceived = true;
}

// Subscriber
ros::Subscriber<geometry_msgs::Twist> sub_cmd("cmd_vel", &cbCmd);

// Publishers
ros::Publisher pub_state("from_stm", &str_msg);
ros::Publisher imu_pub("imu", &imu_msg);
ros::Publisher odom_pub("odom", &odom);
ros::Publisher joint_states_pub("joint_states", &joint_states);
ros::Publisher mag_pub("magnetic_field", &mag_msg);
tf::TransformBroadcaster tf_broadcaster;

// Prototypes
ros::Time rosNow();
void initOdom(void);
void initJointStates(void);
bool calcOdom(double diff_time);
void updateOdom(void);
void updateIMU(void);
void updateJointStates(void);
void updateTFPrefix();
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateMotorInfo(int32_t left_tick, int32_t right_tick);
void updateTime();
void updateGyroCali();
void pubImu(void);
void pubMag(void);
void pubDriveInfo(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART3)
        nh.getHardware()->flush();
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART3)
        nh.getHardware()->reset_rbuf();
}
ros::Time rosNow(){
    return nh.now();
}

/******************* ROBOT INITIALIZATION **********************/
// Initialize Odometry
void initOdom(void){
    init_encoder = true;
    for (int index = 0; index < 3; index++){
        odom_pose[index] = 0.0;
        odom_vel[index] = 0.0;
    }
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 0.0;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.angular.z = 0.0;
}


// Initialize Joint States
void initJointStates(void){
    static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};
    joint_states.header.frame_id = joint_state_header_frame_id;
    joint_states.name = joint_states_name;
    joint_states.name_length = WHEEL_NUM;
    joint_states.position_length = WHEEL_NUM;
    joint_states.velocity_length = WHEEL_NUM;
    joint_states.effort_length = WHEEL_NUM;
}


void setup(UART_HandleTypeDef *leftPort, UART_HandleTypeDef *rightPort, SPI_HandleTypeDef *imuPort, GPIO_TypeDef *portx, uint16_t pin){
    // ros initialization
    nh.initNode();
    nh.loginfo("Starting Robot Initialization");
    nh.loginfo("Initializing Nodes");
    nh.advertise(pub_state);
    nh.advertise(imu_pub);
    nh.advertise(odom_pub);
    nh.advertise(joint_states_pub);
    nh.advertise(mag_pub);
    nh.subscribe(sub_cmd);
    nh.negotiateTopics();
    nh.loginfo("Nodes Initialized");
    tf_broadcaster.init(nh);

    // hardware initialization
    nh.loginfo("Loading Motor Driver");
    motorDriver.init(leftPort, rightPort);
    nh.loginfo("Loading IMU Driver");
    imuDriver.imu_init(imuPort, portx, pin);

    // initializing slam parameters
    nh.loginfo("Initializing Odometry Message");
    initOdom();
    nh.loginfo("Initializing JointState Message");
    initJointStates();

    // Gyro Calibration
    imuDriver.calibrationGyro();
    prev_update_time = HAL_GetTick();
    nh.loginfo("Initialized Robot");
}


/******************* ROBOT INITIALIZATION END **********************/
bool calcOdom(double diff_time){
    float* orientation;
    double wheel_l, wheel_r; // rotation value of wheel [rad]
    double delta_s, theta, delta_theta;
    static double last_theta = 0.0;
    double v, w; // v = translational velocity [m/s], w = rotational velocity [rad/s]
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

    delta_s = wheel_radius * (wheel_r + wheel_l) / wheel_separation;
    // theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION; -> slip 때문에 이걸로 안됨

    orientation = imuDriver.getOrientation();
    theta = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);
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
    last_velocity[LEFT] = wheel_l / step_time;
    last_velocity[RIGHT] = wheel_r / step_time;
    last_theta = theta;

    return true;
}


/******************* PARAMETER UPDATE FUNCTIONS ************************/
void updateOdom(void){
    odom.header.frame_id = odom_header_frame_id;
    odom.child_frame_id = odom_child_frame_id;
    odom.pose.pose.position.x = odom_pose[0];
    odom.pose.pose.position.y = odom_pose[1];
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);
    odom.twist.twist.linear.x = odom_vel[0];
    odom.twist.twist.angular.z = odom_vel[2];
}

// Unused
void updateIMU(void){
    imuDriver.computeIMU();
}

void updateJointStates(void){
    static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
    static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
    //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

    joint_states_pos[LEFT] = last_rad[LEFT];
    joint_states_pos[RIGHT] = last_rad[RIGHT];
    joint_states_vel[LEFT] = last_velocity[LEFT];
    joint_states_vel[RIGHT] = last_velocity[RIGHT];
    joint_states.position = (double*)joint_states_pos;
    joint_states.velocity = (double*)joint_states_vel;
}

void updateTFPrefix(){
    static bool isChecked = false;
    if (isChecked == false){
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");
        sprintf(imu_frame_id, "imu_link");
        sprintf(mag_frame_id, "mag_link");
        sprintf(joint_state_header_frame_id, "base_link");
        isChecked = true;
    }
}

void updateTF(geometry_msgs::TransformStamped& odom_tf){
    odom_tf.header = odom.header;
    odom_tf.child_frame_id = odom.child_frame_id;
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation = odom.pose.pose.orientation;
}

void updateMotorInfo(int32_t left_tick, int32_t right_tick){
    int32_t current_tick = 0;
    static int32_t last_tick[WHEEL_NUM] = {0, 0};
    if (init_encoder){
        for (int index = 0; index < WHEEL_NUM; index++){
            last_diff_tick[index] = 0;
            last_tick[index] = 0;
            last_rad[index] = 0.0;
            last_velocity[index] = 0.0;
        }
        last_tick[LEFT] = left_tick;
        last_tick[RIGHT] = right_tick;
        init_encoder = false;
        return;
    }
    current_tick = left_tick;

    if (current_tick == last_tick[0])
        nh.loginfo("Left Motor Present Position didn't changed");
    last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
    last_tick[LEFT] = current_tick;
    last_rad[LEFT] += TICK2RAD * (double)last_diff_tick[LEFT];
    current_tick = right_tick;

    if (current_tick == last_tick[1])
        nh.loginfo("Right Motor Present Position didn't changed");
    last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
    last_tick[RIGHT] = current_tick;
    last_rad[RIGHT] += TICK2RAD * (double)last_diff_tick[RIGHT];

}

void updateTime(){
    current_offset = HAL_GetTick();
    current_time = nh.now();
}

void updateGyroCali(){
    static bool isCaliEnded = false;
    char log_msg[50];

    if (!isCaliEnded){
        sprintf(log_msg, "Start Calibration of Gyro");
        nh.loginfo(log_msg);
        imuDriver.calibrationGyro();
        sprintf(log_msg, "Calibration End");
        nh.loginfo(log_msg);
        isCaliEnded = true;
    }
}

/*********************** PUBLISHERS ***************************/
// Publish IMU sensor data
void pubImu(void){
    // Get IMU Data
    imu_msg = imuDriver.getIMU();
    imu_msg.header.stamp = rosNow();
    imu_msg.header.frame_id = imu_frame_id;
    imu_pub.publish(&imu_msg);
}

// Publish Magnetic Field Data
void pubMag(void){
    // Get Mag Data
    mag_msg = imuDriver.getMag();
    mag_msg.header.stamp = rosNow();
    mag_msg.header.frame_id = mag_frame_id;
    mag_pub.publish(&mag_msg);
}

// Publish Odometry, JointState, and tf(tf_broadcaster.sendTransform)
void pubDriveInfo(void){
    unsigned long time_now = HAL_GetTick();
    unsigned long step_time = time_now - prev_update_time;

    // Read Encoder
    //motorDriver.readEncoder(left_encoder, right_encoder);
    //updateMotorInfo(left_encoder, right_encoder);
    prev_update_time = time_now;
    ros::Time stamp_now = rosNow();

    // calculate odometry
    calcOdom((double)(step_time * 0.001));

    // odometry
    updateOdom();
    odom.header.stamp = stamp_now;
    odom_pub.publish(&odom);

    // odometry tf
    updateTF(odom_tf);
    odom_tf.header.stamp = stamp_now;
    tf_broadcaster.sendTransform(odom_tf);

    // joint states
    updateJointStates();
    joint_states.header.stamp = stamp_now;
    joint_states_pub.publish(&joint_states);
}

/*********************** ROBOT OPERATION ***************************/
void loop(void){
    static bool variable_flag = false;
    uint32_t t = HAL_GetTick();

    nh.loginfo("LOOP");
    nh.spinOnce();
    updateTime();
    updateTFPrefix();

    if (!variable_flag){
        initOdom();
        variable_flag = true;
    }
    updateTFPrefix();

    // For checking cmd_vel topic receive
    if (isReceived)
        str_msg.data = confirm_receive;
    else
        str_msg.data = msg;

    isReceived = false;
    pub_state.publish(&str_msg);

    // Control motor
    if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY)){
        motorDriver.controlMotor(wheel_radius, wheel_separation, goal_velocity);
        tTime[0] = t;
    }
    
    // Publish Odom, TF, Joint State
    if ((t-tTime[1]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY)){
        pubDriveInfo();
        tTime[1] = t;
    }

    // Publish IMU, Magnetometer data
    if ((t-tTime[2]) >= (1000 / IMU_PUBLISH_FREQUENCY)){
        imuDriver.computeIMU();
        pubImu();
        pubMag();
        tTime[2] = t;
    }
    updateGyroCali();
}
