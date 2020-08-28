/*
 * cpp_main.c
 */
#include "cpp_main.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "ringbuffer.h"
#include "ros.h"

#include <vector>

#include <string>

#include "std_msgs/Byte.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt8MultiArray.h"
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

extern int8_t speedDataRightFrontWheel;
extern int8_t speedDataLeftFrontWheel;
extern int8_t speedDataRightBackWheel;
extern int8_t speedDataLeftBackWheel;

extern int8_t sideDataRightFrontWheel;
extern int8_t sideDataLeftFrontWheel;
extern int8_t sideDataRightBackWheel;
extern int8_t sideDataLeftBackWheel;

extern uint8_t speedRXDataRightFrontWheel;
extern uint8_t sideRXDataRightFrontWheel;
extern uint8_t speedRXDataRightBackWheel;
extern uint8_t sideRXDataRightBackWheel;
extern uint8_t speedRXDataLeftFrontWheel;
extern uint8_t sideRXDataLeftFrontWheel;
extern uint8_t speedRXDataLeftBackWheel;
extern uint8_t sideRXDataLeftBackWheel;

extern uint8_t state_can;

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

extern uint8_t sensorData13;
extern uint8_t sensorData14;
extern uint8_t sensorData15;
extern uint8_t sensorData16;

extern uint8_t laser_sensor_data[16];

extern uint32_t leftCount, rightCount;

extern CAN_RxHeaderTypeDef wheel_RxHeader;

struct ringbuffer rb;

ros::NodeHandle nh;

std_msgs::Int8 uint_msg_right_front;
std_msgs::Int8 uint_msg_left_front;
std_msgs::Int8 uint_msg_right_back;
std_msgs::Int8 uint_msg_left_back;

std_msgs::UInt8MultiArray laser_sensors_data_array;

std_msgs::UInt8 state_data_msg;

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
sensor_msgs::Range laser_sensor_msg_13;
sensor_msgs::Range laser_sensor_msg_14;
sensor_msgs::Range laser_sensor_msg_15;
sensor_msgs::Range laser_sensor_msg_16;


extern "C" void rpm_rightFront_subCb(const std_msgs::Int8& msg);
extern "C" void rpm_leftFront_subCb(const std_msgs::Int8& msg);
extern "C" void rpm_rightBack_subCb(const std_msgs::Int8& msg);
extern "C" void rpm_leftBack_subCb(const std_msgs::Int8& msg);
extern "C" void init_ROS();

ros::Publisher stm("stm", &gyro_msg);
ros::Publisher gyro("gyro", &gyro_msg);
ros::Publisher accel("accel", &accel_msg);

ros::Publisher rpm_right_front("rpm_right_front", &uint_msg_right_front);
ros::Publisher rpm_left_front("rpm_left_front", &uint_msg_left_front);
ros::Publisher rpm_right_back("rpm_right_back", &uint_msg_right_back);
ros::Publisher rpm_left_back("rpm_left_back", &uint_msg_left_back);

ros::Subscriber<std_msgs::Int8> rpm_rightFront_sub("rpm_rightFront_sub", rpm_rightFront_subCb);
ros::Subscriber<std_msgs::Int8> rpm_leftFront_sub("rpm_leftFront_sub", rpm_leftFront_subCb);
ros::Subscriber<std_msgs::Int8> rpm_rightBack_sub("rpm_rightBack_sub", rpm_rightBack_subCb);
ros::Subscriber<std_msgs::Int8> rpm_leftBack_sub("rpm_leftBack_sub", rpm_leftBack_subCb);

ros::Publisher laser_sensors_data("laser_sensors_data", &laser_sensors_data_array);
ros::Publisher state_data("state_data", &state_data_msg);

//ros::Publisher laser_sensor_data_1("laser_sensor_msg_1", &laser_sensor_msg_1);
//ros::Publisher laser_sensor_data_2("laser_sensor_msg_2", &laser_sensor_msg_2);
//ros::Publisher laser_sensor_data_3("laser_sensor_msg_3", &laser_sensor_msg_3);
//ros::Publisher laser_sensor_data_4("laser_sensor_msg_4", &laser_sensor_msg_4);
//ros::Publisher laser_sensor_data_5("laser_sensor_msg_5", &laser_sensor_msg_5);
//ros::Publisher laser_sensor_data_6("laser_sensor_msg_6", &laser_sensor_msg_6);
//ros::Publisher laser_sensor_data_7("laser_sensor_msg_7", &laser_sensor_msg_7);
//ros::Publisher laser_sensor_data_8("laser_sensor_msg_8", &laser_sensor_msg_8);
//ros::Publisher laser_sensor_data_9("laser_sensor_msg_9", &laser_sensor_msg_9);
//ros::Publisher laser_sensor_data_10("laser_sensor_msg_10", &laser_sensor_msg_10);
//ros::Publisher laser_sensor_data_11("laser_sensor_msg_11", &laser_sensor_msg_11);
//ros::Publisher laser_sensor_data_12("laser_sensor_msg_12", &laser_sensor_msg_12);
//ros::Publisher laser_sensor_data_13("laser_sensor_msg_13", &laser_sensor_msg_13);
//ros::Publisher laser_sensor_data_14("laser_sensor_msg_14", &laser_sensor_msg_14);
//ros::Publisher laser_sensor_data_15("laser_sensor_msg_15", &laser_sensor_msg_15);
//ros::Publisher laser_sensor_data_16("laser_sensor_msg_16", &laser_sensor_msg_16);

static nbt_t gyro_nbt;
static nbt_t accel_nbt;
static nbt_t rpm_right_front_nbt;
static nbt_t rpm_left_front_nbt;
static nbt_t rpm_right_back_nbt;
static nbt_t rpm_left_back_nbt;
static nbt_t ros_nbt;
static nbt_t state_nbt;
static nbt_t laser_sensors_data_nbt;

//static nbt_t sensor1_data_nbt;
//static nbt_t sensor2_data_nbt;
//static nbt_t sensor3_data_nbt;
//static nbt_t sensor4_data_nbt;
//static nbt_t sensor5_data_nbt;
//static nbt_t sensor6_data_nbt;
//static nbt_t sensor7_data_nbt;
//static nbt_t sensor8_data_nbt;
//static nbt_t sensor9_data_nbt;
//static nbt_t sensor10_data_nbt;
//static nbt_t sensor11_data_nbt;
//static nbt_t sensor12_data_nbt;
//static nbt_t sensor13_data_nbt;
//static nbt_t sensor14_data_nbt;
//static nbt_t sensor15_data_nbt;
//static nbt_t sensor16_data_nbt;

extern "C" void rpm_rightFront_subCb(const std_msgs::Int8& msg)
{
	if (msg.data >= 6) {
		speedDataRightFrontWheel = msg.data;
		sideDataRightFrontWheel = 0; //CW
	}
	else if (msg.data <= -6) {
		speedDataRightFrontWheel = -(msg.data);
		sideDataRightFrontWheel = 1; //CCW
	}
	else if (msg.data == 0) {
		speedDataRightFrontWheel = 0;
		sideDataRightFrontWheel = 2;
	}
}

extern "C" void rpm_leftFront_subCb(const std_msgs::Int8& msg)
{
	if (msg.data >= 6) {
		speedDataLeftFrontWheel = msg.data;
		sideDataLeftFrontWheel = 1; //CCW
	}
	else if (msg.data <= -6) {
		speedDataLeftFrontWheel = -(msg.data);
		sideDataLeftFrontWheel = 0; //CW
	}
	else {
		speedDataLeftFrontWheel = 0;
		sideDataLeftFrontWheel = 2;
	}
}

extern "C" void rpm_rightBack_subCb(const std_msgs::Int8& msg)
{
	if (msg.data >= 6) {
		speedDataRightBackWheel = msg.data;
		sideDataRightBackWheel = 0; //CW
	}
	else if (msg.data <= -6) {
		speedDataRightBackWheel = -(msg.data);
		sideDataRightBackWheel = 1; //CCW
	}
	else {
		speedDataRightBackWheel = 0;
		sideDataRightBackWheel = 2;
	}
}

extern "C" void rpm_leftBack_subCb(const std_msgs::Int8& msg)
{
	if (msg.data >= 6) {
		speedDataLeftBackWheel = msg.data;
		sideDataLeftBackWheel = 1; //CCW
	}
	else if (msg.data <= -6) {
		speedDataLeftBackWheel = -(msg.data);
		sideDataLeftBackWheel = 0; //CW
	}
	else {
		speedDataLeftBackWheel = 0;
		sideDataLeftBackWheel = 2;
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

	nh.subscribe(rpm_rightFront_sub);
	nh.subscribe(rpm_leftFront_sub);
	nh.subscribe(rpm_rightBack_sub);
	nh.subscribe(rpm_leftBack_sub);

	nh.advertise(stm);
	nh.advertise(gyro);
	nh.advertise(accel);
	nh.advertise(rpm_left_front);
	nh.advertise(rpm_right_front);
	nh.advertise(rpm_left_back);
	nh.advertise(rpm_right_back);

	nh.advertise(laser_sensors_data);

//	nh.advertise(state_data);
//	nh.advertise(laser_sensor_data_1);
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
//	nh.advertise(laser_sensor_data_13);
//	nh.advertise(laser_sensor_data_14);
//	nh.advertise(laser_sensor_data_15);
//	nh.advertise(laser_sensor_data_16);

	NBT_init(&rpm_left_front_nbt, 9);
	NBT_init(&rpm_right_front_nbt, 9);
	NBT_init(&rpm_left_back_nbt, 9);
	NBT_init(&rpm_right_back_nbt, 9);

	NBT_init(&laser_sensors_data_nbt, 9);

	//NBT_init(&state_nbt, 9);

	NBT_init(&gyro_nbt, 5);
	NBT_init(&accel_nbt, 5);

	NBT_init(&ros_nbt, 1);

//	NBT_init(&sensor1_data_nbt, 5);
//	NBT_init(&sensor2_data_nbt, 5);
//	NBT_init(&sensor3_data_nbt, 5);
//	NBT_init(&sensor4_data_nbt, 5);
//	NBT_init(&sensor5_data_nbt, 5);
//	NBT_init(&sensor6_data_nbt, 5);
//	NBT_init(&sensor7_data_nbt, 5);
//	NBT_init(&sensor8_data_nbt, 5);
//	NBT_init(&sensor9_data_nbt, 5);
//	NBT_init(&sensor10_data_nbt, 5);
//	NBT_init(&sensor11_data_nbt, 5);
//	NBT_init(&sensor12_data_nbt, 5);
//	NBT_init(&sensor13_data_nbt, 5);
//	NBT_init(&sensor14_data_nbt, 5);
//	NBT_init(&sensor15_data_nbt, 5);
//	NBT_init(&sensor16_data_nbt, 5);
}

extern "C" void laser_sensors_data_handler(void)
{
	if (NBT_handler(&laser_sensors_data_nbt)) {
		laser_sensors_data_array.data_length = 16;
		laser_sensors_data_array.data = laser_sensor_data;
    	if (nh.connected()) {
    		laser_sensors_data.publish(&laser_sensors_data_array);
    	}
	}
}

extern "C" void state_data_handler(void)
{
	if (NBT_handler(&state_nbt)) {
		state_data_msg.data = state_can;
    	if (nh.connected()) {
    		state_data.publish(&state_data_msg);
    	}
	}
}

//extern "C" void laser_sensor_handler_1(void)
//{
//	if (NBT_handler(&sensor1_data_nbt)) {
//		char frame_id_1[] = "sensor_frame_fr_0";
//		laser_sensor_msg_1.min_range = 0.0;
//		laser_sensor_msg_1.max_range = 2.0;
//		laser_sensor_msg_1.field_of_view = 0.436332;
//		laser_sensor_msg_1.radiation_type = 1;
//		laser_sensor_msg_1.range = (float)(sensorData1)/100;
//		laser_sensor_msg_1.header.frame_id = frame_id_1;
//    	if (nh.connected()) {
//    		laser_sensor_data_1.publish(&laser_sensor_msg_1);
//    	}
//	}
//}
//
//extern "C" void laser_sensor_handler_2(void)
//{
//	if (NBT_handler(&sensor2_data_nbt)) {
//		char frame_id_2[] = "sensor_frame_fr_1";
//		laser_sensor_msg_2.min_range = 0.0;
//		laser_sensor_msg_2.max_range = 2.0;
//		laser_sensor_msg_2.field_of_view = 0.436332;
//		laser_sensor_msg_2.radiation_type = 1;
//		laser_sensor_msg_2.range = (float)(sensorData2)/100;
//		laser_sensor_msg_2.header.frame_id = frame_id_2;
//    	if (nh.connected()) {
//    		laser_sensor_data_2.publish(&laser_sensor_msg_2);
//    	}
//	}
//}
//
//extern "C" void laser_sensor_handler_3(void)
//{
//	if (NBT_handler(&sensor3_data_nbt)) {
//		char frame_id_3[] = "sensor_frame_fr_2";
//		laser_sensor_msg_3.min_range = 0.0;
//		laser_sensor_msg_3.max_range = 2.0;
//		laser_sensor_msg_3.field_of_view = 0.436332;
//		laser_sensor_msg_3.radiation_type = 1;
//		laser_sensor_msg_3.range = (float)(sensorData3)/100;
//		laser_sensor_msg_3.header.frame_id = frame_id_3;
//    	if (nh.connected()) {
//    		laser_sensor_data_3.publish(&laser_sensor_msg_3);
//    	}
//	}
//}
//
//extern "C" void laser_sensor_handler_4(void)
//{
//	if (NBT_handler(&sensor4_data_nbt)) {
//		char frame_id_4[] = "sensor_frame_fl_3";
//		laser_sensor_msg_4.min_range = 0.0;
//		laser_sensor_msg_4.max_range = 2.0;
//		laser_sensor_msg_4.field_of_view = 0.436332;
//		laser_sensor_msg_4.radiation_type = 1;
//		laser_sensor_msg_4.range = (float)(sensorData4)/100;
//		laser_sensor_msg_4.header.frame_id = frame_id_4;
//    	if (nh.connected()) {
//    		laser_sensor_data_4.publish(&laser_sensor_msg_4);
//    	}
//	}
//}
//
//extern "C" void laser_sensor_handler_5(void)
//{
//	if (NBT_handler(&sensor5_data_nbt)) {
//		char frame_id_5[] = "sensor_frame_fl_0";
//		laser_sensor_msg_5.min_range = 0.0;
//		laser_sensor_msg_5.max_range = 2.0;
//		laser_sensor_msg_5.field_of_view = 0.436332;
//		laser_sensor_msg_5.radiation_type = 1;
//		laser_sensor_msg_5.range = (float)(sensorData5)/100;
//		laser_sensor_msg_5.header.frame_id = frame_id_5;
//		if (nh.connected()) {
//			laser_sensor_data_5.publish(&laser_sensor_msg_5);
//		}
//	}
//}
//
//extern "C" void laser_sensor_handler_6(void)
//{
//	if (NBT_handler(&sensor6_data_nbt)) {
//		char frame_id_6[] = "sensor_frame_fl_1";
//		laser_sensor_msg_6.min_range = 0.0;
//		laser_sensor_msg_6.max_range = 2.0;
//		laser_sensor_msg_6.field_of_view = 0.436332;
//		laser_sensor_msg_6.radiation_type = 1;
//		laser_sensor_msg_6.range = (float)(sensorData6)/100;
//		laser_sensor_msg_6.header.frame_id = frame_id_6;
//		if (nh.connected()) {
//			laser_sensor_data_6.publish(&laser_sensor_msg_6);
//		}
//	}
//}
//
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
//	if (NBT_handler(&sensor8_data_nbt)) {
//		char frame_id_8[] = "sensor_frame_fl_2";
//		laser_sensor_msg_8.min_range = 0.0;
//		laser_sensor_msg_8.max_range = 2.0;
//		laser_sensor_msg_8.field_of_view = 0.436332;
//		laser_sensor_msg_8.radiation_type = 1;
//		laser_sensor_msg_8.range = (float)(sensorData8)/100;
//		laser_sensor_msg_8.header.frame_id = frame_id_8;
//		laser_sensor_data_8.publish(&laser_sensor_msg_8);
//	}
//}
//
//extern "C" void laser_sensor_handler_9(void)
//{
//	if (NBT_handler(&sensor9_data_nbt)) {
//		char frame_id_9[] = "sensor_frame_fl_7";
//		laser_sensor_msg_9.min_range = 0.0;
//		laser_sensor_msg_9.max_range = 2.0;
//		laser_sensor_msg_9.field_of_view = 0.436332;
//		laser_sensor_msg_9.radiation_type = 1;
//		laser_sensor_msg_9.range = (float)(sensorData9)/100;
//		laser_sensor_msg_9.header.frame_id = frame_id_9;
//		laser_sensor_data_9.publish(&laser_sensor_msg_9);
//	}
//}
//
//extern "C" void laser_sensor_handler_10(void)
//{
//	if (NBT_handler(&sensor10_data_nbt)) {
//		char frame_id_10[] = "sensor_frame_fl_2";
//		laser_sensor_msg_10.min_range = 0.0;
//		laser_sensor_msg_10.max_range = 2.0;
//		laser_sensor_msg_10.field_of_view = 0.436332;
//		laser_sensor_msg_10.radiation_type = 1;
//		laser_sensor_msg_10.range = (float)(sensorData10)/100;
//		laser_sensor_msg_10.header.frame_id = frame_id_10;
//		laser_sensor_data_10.publish(&laser_sensor_msg_10);
//	}
//}
//
//extern "C" void laser_sensor_handler_11(void)
//{
//	if (NBT_handler(&sensor11_data_nbt)) {
//		char frame_id_11[] = "sensor_frame_fl_2";
//		laser_sensor_msg_11.min_range = 0.0;
//		laser_sensor_msg_11.max_range = 2.0;
//		laser_sensor_msg_11.field_of_view = 0.436332;
//		laser_sensor_msg_11.radiation_type = 1;
//		laser_sensor_msg_11.range = (float)(sensorData11)/100;
//		laser_sensor_msg_11.header.frame_id = frame_id_11;
//		laser_sensor_data_11.publish(&laser_sensor_msg_11);
//	}
//}
//
//extern "C" void laser_sensor_handler_12(void)
//{
//	if (NBT_handler(&sensor12_data_nbt)) {
//		char frame_id_12[] = "sensor_frame_fl_2";
//		laser_sensor_msg_12.min_range = 0.0;
//		laser_sensor_msg_12.max_range = 2.0;
//		laser_sensor_msg_12.field_of_view = 0.436332;
//		laser_sensor_msg_12.radiation_type = 1;
//		laser_sensor_msg_12.range = (float)(sensorData12)/100;
//		laser_sensor_msg_12.header.frame_id = frame_id_12;
//		laser_sensor_data_12.publish(&laser_sensor_msg_12);
//	}
//}
//
//extern "C" void laser_sensor_handler_13(void)
//{
//	if (NBT_handler(&sensor13_data_nbt)) {
//		char frame_id_13[] = "sensor_frame_fl_2";
//		laser_sensor_msg_13.min_range = 0.0;
//		laser_sensor_msg_13.max_range = 2.0;
//		laser_sensor_msg_13.field_of_view = 0.436332;
//		laser_sensor_msg_13.radiation_type = 1;
//		laser_sensor_msg_13.range = (float)(sensorData13)/100;
//		laser_sensor_msg_13.header.frame_id = frame_id_13;
//		laser_sensor_data_13.publish(&laser_sensor_msg_13);
//	}
//}
//
//extern "C" void laser_sensor_handler_14(void)
//{
//	if (NBT_handler(&sensor14_data_nbt)) {
//		char frame_id_14[] = "sensor_frame_fl_2";
//		laser_sensor_msg_14.min_range = 0.0;
//		laser_sensor_msg_14.max_range = 2.0;
//		laser_sensor_msg_14.field_of_view = 0.436332;
//		laser_sensor_msg_14.radiation_type = 1;
//		laser_sensor_msg_14.range = (float)(sensorData14)/100;
//		laser_sensor_msg_14.header.frame_id = frame_id_14;
//		laser_sensor_data_14.publish(&laser_sensor_msg_14);
//	}
//}
//
//extern "C" void laser_sensor_handler_15(void)
//{
//	if (NBT_handler(&sensor15_data_nbt)) {
//		char frame_id_15[] = "sensor_frame_fl_2";
//		laser_sensor_msg_15.min_range = 0.0;
//		laser_sensor_msg_15.max_range = 2.0;
//		laser_sensor_msg_15.field_of_view = 0.436332;
//		laser_sensor_msg_15.radiation_type = 1;
//		laser_sensor_msg_15.range = (float)(sensorData15)/100;
//		laser_sensor_msg_15.header.frame_id = frame_id_15;
//		laser_sensor_data_15.publish(&laser_sensor_msg_15);
//	}
//}
//
//extern "C" void laser_sensor_handler_16(void)
//{
//	if (NBT_handler(&sensor16_data_nbt)) {
//		char frame_id_16[] = "sensor_frame_fl_2";
//		laser_sensor_msg_16.min_range = 0.0;
//		laser_sensor_msg_16.max_range = 2.0;
//		laser_sensor_msg_16.field_of_view = 0.436332;
//		laser_sensor_msg_16.radiation_type = 1;
//		laser_sensor_msg_16.range = (float)(sensorData16)/100;
//		laser_sensor_msg_16.header.frame_id = frame_id_16;
//		laser_sensor_data_16.publish(&laser_sensor_msg_16);
//	}
//}

extern "C" void rpm_right_front_handler(void)
{
	if (NBT_handler(&rpm_right_front_nbt))
	{
		if (sideRXDataRightFrontWheel == 2) {
		    uint_msg_right_front.data = speedRXDataRightFrontWheel;
		}
		else if (sideRXDataRightFrontWheel == 1) {
			uint_msg_right_front.data = -speedRXDataRightFrontWheel;
		}
		else {
			uint_msg_right_front.data = 0;
		}
    	if (nh.connected()) {
    		rpm_right_front.publish(&uint_msg_right_front);
    	}
	}
}

extern "C" void rpm_left_front_handler(void)
{
	if (NBT_handler(&rpm_left_front_nbt))
	{
		if (sideRXDataLeftFrontWheel == 1) {
		    uint_msg_left_front.data = speedRXDataLeftFrontWheel;
		}
		else if (sideRXDataLeftFrontWheel == 2) {
		  	uint_msg_left_front.data = -speedRXDataLeftFrontWheel;
		}
		else {
		  	uint_msg_left_front.data = 0;
		}
    	if (nh.connected()) {
    		rpm_left_front.publish(&uint_msg_left_front);
    	}
	}
}

extern "C" void rpm_right_back_handler(void)
{
	if (NBT_handler(&rpm_right_back_nbt))
	{
		if (sideRXDataRightBackWheel == 2) {
		    uint_msg_right_back.data = speedRXDataRightBackWheel;
		}
		else if (sideRXDataRightBackWheel == 1) {
			uint_msg_right_back.data = -speedRXDataRightBackWheel;
		}
		else {
			uint_msg_right_back.data = 0;
		}
    	if (nh.connected()) {
    		rpm_right_back.publish(&uint_msg_right_back);
    	}
	}
}

extern "C" void rpm_left_back_handler(void)
{
	if (NBT_handler(&rpm_left_back_nbt))
	{
		if (sideRXDataLeftBackWheel == 1) {
		  	uint_msg_left_back.data = speedRXDataLeftBackWheel;
		}
		else if (sideRXDataLeftBackWheel == 2) {
			uint_msg_left_back.data = -speedRXDataLeftBackWheel;
		}
		else {
			uint_msg_left_back.data = 0;
		}
		if (nh.connected()) {
			rpm_left_back.publish(&uint_msg_left_back);
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
	if (NBT_handler(&ros_nbt))	{
		nh.spinOnce();
	}
}

