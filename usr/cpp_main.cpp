/*
 * cpp_main.c

 */
#include "cpp_main.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "ringbuffer.h"
#include "ros.h"

#include <string>

#include "std_msgs/Byte.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt32.h"
#include "nbt.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

#include "sensor_msgs/Range.h"

extern uint8_t RxBuffer[RxBufferSize];

extern uint8_t nh_connected;

extern float gyroX;
extern float gyroY;
extern float gyroZ;
extern float accelX;
extern float accelY;
extern float accelZ;

extern int8_t sideDataRight;
extern int8_t speedDataRight;
extern int8_t sideDataLeft;
extern int8_t speedDataLeft;
extern int8_t sideRXDataRight;
extern int8_t sideRXDataLeft;

extern uint8_t sensorData1;
extern uint8_t sensorData2;
extern uint8_t sensorData3;
extern uint8_t sensorData4;
extern uint8_t sensorData5;
extern uint8_t sensorData6;

extern uint8_t sensorData7;
extern uint8_t sensorData8;
extern uint8_t sensorData9;
extern uint8_t sensorData10;
extern uint8_t sensorData11;
extern uint8_t sensorData12;

extern uint8_t speedRXDataRight;
extern uint8_t speedRXDataLeft;

extern uint32_t leftCount, rightCount;

extern CAN_RxHeaderTypeDef wheel_RxHeader;

struct ringbuffer rb;

ros::NodeHandle nh;

std_msgs::Int8 uint_msg_right;
std_msgs::Int8 uint_msg_left;

std_msgs::UInt8 sensor_data_msg;

geometry_msgs::Vector3 gyro_msg;
geometry_msgs::Vector3 accel_msg;

sensor_msgs::Range laser_sensor_msg_1;
sensor_msgs::Range laser_sensor_msg_2;
sensor_msgs::Range laser_sensor_msg_3;
sensor_msgs::Range laser_sensor_msg_4;
sensor_msgs::Range laser_sensor_msg_5;
sensor_msgs::Range laser_sensor_msg_6;
sensor_msgs::Range laser_sensor_msg_7;
sensor_msgs::Range laser_sensor_msg_8;
sensor_msgs::Range laser_sensor_msg_9;
sensor_msgs::Range laser_sensor_msg_10;
sensor_msgs::Range laser_sensor_msg_11;
sensor_msgs::Range laser_sensor_msg_12;

extern "C" void rpm_right_subCb(const std_msgs::Int8& msg);
extern "C" void rpm_left_subCb(const std_msgs::Int8& msg);
extern "C" void init_ROS();
ros::Publisher stm("stm", &gyro_msg);
ros::Publisher gyro("gyro", &gyro_msg);
ros::Publisher accel("accel", &accel_msg);
ros::Publisher rpm_right("rpm_right", &uint_msg_right);
ros::Publisher rpm_left("rpm_left", &uint_msg_left);
ros::Subscriber<std_msgs::Int8> rpm_right_sub("rpm_right_sub", rpm_right_subCb);
ros::Subscriber<std_msgs::Int8> rpm_left_sub("rpm_left_sub", rpm_left_subCb);

ros::Publisher sensor_data("sensor_data", &sensor_data_msg);

ros::Publisher laser_sensor_data_1("laser_sensor_msg_1", &laser_sensor_msg_1);
ros::Publisher laser_sensor_data_2("laser_sensor_msg_2", &laser_sensor_msg_2);
ros::Publisher laser_sensor_data_3("laser_sensor_msg_3", &laser_sensor_msg_3);
ros::Publisher laser_sensor_data_4("laser_sensor_msg_4", &laser_sensor_msg_4);
ros::Publisher laser_sensor_data_5("laser_sensor_msg_5", &laser_sensor_msg_5);
ros::Publisher laser_sensor_data_6("laser_sensor_msg_6", &laser_sensor_msg_6);
ros::Publisher laser_sensor_data_7("laser_sensor_msg_7", &laser_sensor_msg_7);
ros::Publisher laser_sensor_data_8("laser_sensor_msg_8", &laser_sensor_msg_8);
ros::Publisher laser_sensor_data_9("laser_sensor_msg_9", &laser_sensor_msg_9);
ros::Publisher laser_sensor_data_10("laser_sensor_msg_10", &laser_sensor_msg_10);
ros::Publisher laser_sensor_data_11("laser_sensor_msg_11", &laser_sensor_msg_11);
ros::Publisher laser_sensor_data_12("laser_sensor_msg_12", &laser_sensor_msg_12);

static nbt_t gyro_nbt;
static nbt_t accel_nbt;
static nbt_t rpm_right_nbt;
static nbt_t rpm_left_nbt;
static nbt_t ros_nbt;

static nbt_t sensor1_data_nbt;
static nbt_t sensor2_data_nbt;
static nbt_t sensor3_data_nbt;
static nbt_t sensor4_data_nbt;
static nbt_t sensor5_data_nbt;
static nbt_t sensor6_data_nbt;
static nbt_t sensor7_data_nbt;
static nbt_t sensor8_data_nbt;
static nbt_t sensor9_data_nbt;
static nbt_t sensor10_data_nbt;
static nbt_t sensor11_data_nbt;
static nbt_t sensor12_data_nbt;

extern "C" void rpm_right_subCb(const std_msgs::Int8& msg)
{
	if (msg.data >= 6) {
		speedDataRight = msg.data;
		sideDataRight = 1;
	}
	else if (msg.data <= -6) {
		speedDataRight = -(msg.data);
		sideDataRight = 0;
	}
	else {
		speedDataRight = 0;
	}
}

extern "C" void rpm_left_subCb(const std_msgs::Int8& msg)
{
	if (msg.data >= 6) {
		speedDataLeft = msg.data;
		sideDataLeft = 0;
	}
	else if (msg.data <= -6) {
		speedDataLeft = -(msg.data);
		sideDataLeft = 1;
	}
	else {
		speedDataLeft = 0;
	}
}

extern "C" void cdc_receive_put(uint8_t value)
	{
		ringbuffer_putchar(&rb, value);
	}
extern "C" void init_ROS(void)
{
	ringbuffer_init(&rb, RxBuffer, RxBufferSize);
	// Initialize ROS
	nh.initNode();
	nh.subscribe(rpm_left_sub);
	nh.subscribe(rpm_right_sub);
	nh.advertise(stm);
	nh.advertise(gyro);
	nh.advertise(accel);
	nh.advertise(rpm_left);
	nh.advertise(rpm_right);

	nh.advertise(laser_sensor_data_1);
//	nh.advertise(laser_sensor_data_2);
//	nh.advertise(laser_sensor_data_3);
//	nh.advertise(laser_sensor_data_4);
//	nh.advertise(laser_sensor_data_5);
//	nh.advertise(laser_sensor_data_6);
//	nh.advertise(laser_sensor_data_7);
//	nh.advertise(laser_sensor_data_8);
//	nh.advertise(laser_sensor_data_9);
//	nh.advertise(laser_sensor_data_10);
//	nh.advertise(laser_sensor_data_11);
//	nh.advertise(laser_sensor_data_12);

	NBT_init(&rpm_left_nbt, 9);
	NBT_init(&rpm_right_nbt, 9);
	NBT_init(&gyro_nbt, 9);
	NBT_init(&accel_nbt, 9);
	NBT_init(&ros_nbt, 1);
	NBT_init(&sensor1_data_nbt, 9);
	NBT_init(&sensor2_data_nbt, 9);
	NBT_init(&sensor3_data_nbt, 9);
	NBT_init(&sensor4_data_nbt, 9);
	NBT_init(&sensor5_data_nbt, 9);
	NBT_init(&sensor6_data_nbt, 9);
}

extern "C" void laser_sensor_handler_1(void)
{
	if (NBT_handler(&sensor1_data_nbt)) {
		char frame_id_1[] = "sensor_frame_fr_0";
		laser_sensor_msg_1.min_range = 0.0;
		laser_sensor_msg_1.max_range = 2.0;
		laser_sensor_msg_1.field_of_view = 0.436332;
		laser_sensor_msg_1.radiation_type = 1;
		laser_sensor_msg_1.range = (float)(sensorData1)/100;
		laser_sensor_msg_1.header.frame_id = frame_id_1;
    	if (nh.connected()) {
    		laser_sensor_data_1.publish(&laser_sensor_msg_1);
    	}
	}
}

extern "C" void laser_sensor_handler_2(void)
{
	if (NBT_handler(&sensor2_data_nbt)) {
		char frame_id_2[] = "sensor_frame_fr_1";
		laser_sensor_msg_2.min_range = 0.0;
		laser_sensor_msg_2.max_range = 2.0;
		laser_sensor_msg_2.field_of_view = 0.436332;
		laser_sensor_msg_2.radiation_type = 1;
		laser_sensor_msg_2.range = (float)(sensorData2)/100;
		laser_sensor_msg_2.header.frame_id = frame_id_2;
    	if (nh.connected()) {
    		laser_sensor_data_2.publish(&laser_sensor_msg_2);
    	}
	}
}

extern "C" void laser_sensor_handler_3(void)
{
	if (NBT_handler(&sensor3_data_nbt)) {
		char frame_id_3[] = "sensor_frame_fr_2";
		laser_sensor_msg_3.min_range = 0.0;
		laser_sensor_msg_3.max_range = 2.0;
		laser_sensor_msg_3.field_of_view = 0.436332;
		laser_sensor_msg_3.radiation_type = 1;
		laser_sensor_msg_3.range = (float)(sensorData3)/100;
		laser_sensor_msg_3.header.frame_id = frame_id_3;
    	if (nh.connected()) {
    		laser_sensor_data_3.publish(&laser_sensor_msg_3);
    	}
	}
}

extern "C" void laser_sensor_handler_4(void)
{
	if (NBT_handler(&sensor4_data_nbt)) {
		char frame_id_4[] = "sensor_frame_fl_0";
		laser_sensor_msg_4.min_range = 0.0;
		laser_sensor_msg_4.max_range = 2.0;
		laser_sensor_msg_4.field_of_view = 0.436332;
		laser_sensor_msg_4.radiation_type = 1;
		laser_sensor_msg_4.range = (float)(sensorData4)/100;
		laser_sensor_msg_4.header.frame_id = frame_id_4;
    	if (nh.connected()) {
    		laser_sensor_data_4.publish(&laser_sensor_msg_4);
    	}
	}
}

extern "C" void laser_sensor_handler_5(void)
{
	if (NBT_handler(&sensor5_data_nbt)) {
		char frame_id_5[] = "sensor_frame_fl_1";
		laser_sensor_msg_5.min_range = 0.0;
		laser_sensor_msg_5.max_range = 2.0;
		laser_sensor_msg_5.field_of_view = 0.436332;
		laser_sensor_msg_5.radiation_type = 1;
		laser_sensor_msg_5.range = (float)(sensorData5)/100;
		laser_sensor_msg_5.header.frame_id = frame_id_5;
		if (nh.connected()) {
			laser_sensor_data_5.publish(&laser_sensor_msg_5);
		}
	}
}

extern "C" void laser_sensor_handler_6(void)
{
	if (NBT_handler(&sensor6_data_nbt)) {
		char frame_id_6[] = "sensor_frame_fl_2";
		laser_sensor_msg_6.min_range = 0.0;
		laser_sensor_msg_6.max_range = 2.0;
		laser_sensor_msg_6.field_of_view = 0.436332;
		laser_sensor_msg_6.radiation_type = 1;
		laser_sensor_msg_6.range = (float)(sensorData6)/100;
		laser_sensor_msg_6.header.frame_id = frame_id_6;
		if (nh.connected()) {
			laser_sensor_data_6.publish(&laser_sensor_msg_6);
		}
	}
}

//extern "C" void laser_sensor_handler_7(void)
//{
//	if (NBT_handler(&sensor7_data_nbt)) {
//		char frame_id_7[] = "sensor_frame_br_0";
//		laser_sensor_msg_7.min_range = 0.0;
//		laser_sensor_msg_7.max_range = 2.0;
//		laser_sensor_msg_7.field_of_view = 0.436332;
//		laser_sensor_msg_7.radiation_type = 1;
//		laser_sensor_msg_7.range = (float)(sensorData7)/100;
//		laser_sensor_msg_7.header.frame_id = frame_id_7;
//		laser_sensor_data_7.publish(&laser_sensor_msg_7);
//	}
//}
//
//extern "C" void laser_sensor_handler_8(void)
//{
//	if (NBT_handler(&sensor6_data_nbt)) {
//		char frame_id_6[] = "sensor_frame_fl_2";
//		laser_sensor_msg_6.min_range = 0.0;
//		laser_sensor_msg_6.max_range = 2.0;
//		laser_sensor_msg_6.field_of_view = 0.436332;
//		laser_sensor_msg_6.radiation_type = 1;
//		laser_sensor_msg_6.range = (float)(sensorData6)/100;
//		laser_sensor_msg_6.header.frame_id = frame_id_6;
//		laser_sensor_data_6.publish(&laser_sensor_msg_6);
//	}
//}
//
//extern "C" void laser_sensor_handler_9(void)
//{
//	if (NBT_handler(&sensor7_data_nbt)) {
//		char frame_id_7[] = "sensor_frame_fl_7";
//		laser_sensor_msg_6.min_range = 0.0;
//		laser_sensor_msg_6.max_range = 2.0;
//		laser_sensor_msg_6.field_of_view = 0.436332;
//		laser_sensor_msg_6.radiation_type = 1;
//		laser_sensor_msg_6.range = (float)(sensorData6)/100;
//		laser_sensor_msg_6.header.frame_id = frame_id_6;
//		laser_sensor_data_6.publish(&laser_sensor_msg_6);
//	}
//}
//
//extern "C" void laser_sensor_handler_10(void)
//{
//	if (NBT_handler(&sensor6_data_nbt)) {
//		char frame_id_6[] = "sensor_frame_fl_2";
//		laser_sensor_msg_6.min_range = 0.0;
//		laser_sensor_msg_6.max_range = 2.0;
//		laser_sensor_msg_6.field_of_view = 0.436332;
//		laser_sensor_msg_6.radiation_type = 1;
//		laser_sensor_msg_6.range = (float)(sensorData6)/100;
//		laser_sensor_msg_6.header.frame_id = frame_id_6;
//		laser_sensor_data_6.publish(&laser_sensor_msg_6);
//	}
//}
//
//extern "C" void laser_sensor_handler_11(void)
//{
//	if (NBT_handler(&sensor6_data_nbt)) {
//		char frame_id_6[] = "sensor_frame_fl_2";
//		laser_sensor_msg_6.min_range = 0.0;
//		laser_sensor_msg_6.max_range = 2.0;
//		laser_sensor_msg_6.field_of_view = 0.436332;
//		laser_sensor_msg_6.radiation_type = 1;
//		laser_sensor_msg_6.range = (float)(sensorData6)/100;
//		laser_sensor_msg_6.header.frame_id = frame_id_6;
//		laser_sensor_data_6.publish(&laser_sensor_msg_6);
//	}
//}
//
//extern "C" void laser_sensor_handler_12(void)
//{
//	if (NBT_handler(&sensor6_data_nbt)) {
//		char frame_id_6[] = "sensor_frame_fl_2";
//		laser_sensor_msg_6.min_range = 0.0;
//		laser_sensor_msg_6.max_range = 2.0;
//		laser_sensor_msg_6.field_of_view = 0.436332;
//		laser_sensor_msg_6.radiation_type = 1;
//		laser_sensor_msg_6.range = (float)(sensorData6)/100;
//		laser_sensor_msg_6.header.frame_id = frame_id_6;
//		laser_sensor_data_6.publish(&laser_sensor_msg_6);
//	}
//}

extern "C" void rpm_right_handler(void)
{
	if (NBT_handler(&rpm_right_nbt))
	  {
		if (sideRXDataRight == 1) {
		    uint_msg_right.data = speedRXDataRight;
		}
		else if (sideRXDataRight == 2) {
		  	uint_msg_right.data = -speedRXDataRight;
		}
		else {
		  	uint_msg_right.data = 0;
		}
    	if (nh.connected()) {
    		rpm_right.publish(&uint_msg_right);
    	}
	  }
}

extern "C" void rpm_left_handler(void)
{
	if (NBT_handler(&rpm_left_nbt))
		{
		  if (sideRXDataLeft == 2) {
		  	uint_msg_left.data = speedRXDataLeft;
		  }
		  else if (sideRXDataLeft == 1) {
		  	uint_msg_left.data = -speedRXDataLeft;
		  }
		  else {
		  	uint_msg_left.data = 0;
		  }
		  if (nh.connected()) {
			  rpm_left.publish(&uint_msg_left);
		  }
		}
}

extern "C" void gyro_handler(void)
{

    	gyro_msg.x = gyroX;
    	gyro_msg.y = gyroY;
    	gyro_msg.z = gyroZ;
    	if (nh.connected()) {
    		if (NBT_handler(&gyro_nbt))
    		{
    			gyro.publish(&gyro_msg);
    		}
    	}
}

extern "C" void accel_handler(void)
{
    	accel_msg.x = accelX;
    	accel_msg.y = accelY;
    	accel_msg.z = accelZ;
    	if (nh.connected()) {
    		if (NBT_handler(&accel_nbt))
    		{
    			accel.publish(&accel_msg);
    		}
    	}
}


extern "C" void spinOnce(void)
{
	if (NBT_handler(&ros_nbt))
	{
		nh_connected = nh.connected();
		nh.spinOnce();
	}
}

