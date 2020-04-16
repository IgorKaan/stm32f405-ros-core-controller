/*
 * cpp_main.c

 */

#include "cpp_main.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "ringbuffer.h"
#include "ros.h"
#include "std_msgs/Byte.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt32.h"
#include "nbt.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

extern uint8_t RxBuffer[RxBufferSize];
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

extern uint8_t speedRXDataRight;
extern uint8_t speedRXDataLeft;
extern uint32_t leftCount, rightCount;
extern CAN_RxHeaderTypeDef wheel_RxHeader;
struct ringbuffer rb;
uint32_t a = 0;
int8_t pred_speedDataLeft = 60;
int8_t pred_speedDataRight = 60;

ros::NodeHandle nh;
std_msgs::String str_msg;
std_msgs::Int8 uint_msg_right;
std_msgs::Int8 uint_msg_left;
std_msgs::UInt32 id_msg;
std_msgs::UInt32 right_can;
std_msgs::UInt32 left_can;
geometry_msgs::Vector3 gyro_msg;
geometry_msgs::Vector3 accel_msg;

extern "C" void messageSide(const std_msgs::Byte& msg);
extern "C" void messageSpeed(const std_msgs::Byte& msg);
extern "C" void rpm_right_subCb(const std_msgs::Int8& msg);
extern "C" void rpm_left_subCb(const std_msgs::Int8& msg);
extern "C" void init_ROS();
ros::Publisher str("gyro", &str_msg);
ros::Publisher gyro("gyro", &gyro_msg);
ros::Publisher accel("accel", &accel_msg);
ros::Publisher rpm_right("rpm_right", &uint_msg_right);
ros::Publisher rpm_left("rpm_left", &uint_msg_left);
ros::Publisher right_can_msg("right_can_msg", &right_can);
ros::Publisher left_can_msg("left_can_msg", &left_can);
ros::Publisher id("id", &id_msg);
ros::Subscriber<std_msgs::Byte> sideD("side", messageSide);
ros::Subscriber<std_msgs::Byte> speedD("speed", messageSpeed);
ros::Subscriber<std_msgs::Int8> rpm_right_sub("rpm_right_sub", rpm_right_subCb);
ros::Subscriber<std_msgs::Int8> rpm_left_sub("rpm_left_sub", rpm_left_subCb);

static nbt_t id_nbt;
static nbt_t publish_nbt;
static nbt_t vector_nbt;
static nbt_t point_nbt;
static nbt_t gyro_nbt;
static nbt_t accel_nbt;
static nbt_t rpm_right_nbt;
static nbt_t rpm_left_nbt;
static nbt_t right_can_msg_nbt;
static nbt_t left_can_msg_nbt;
static nbt_t ros_nbt;

void reduce_speed() {
		HAL_Delay(2000);
}

extern "C" void rpm_right_subCb(const std_msgs::Int8& msg)
{
	//speedDataRight = 0;
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
	//speedDataLeft = 0;
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


extern "C" void messageSide(const std_msgs::Byte& msg)
{
	//sideData = msg.data;
}

extern "C" void messageSpeed(const std_msgs::Byte& msg)
{
	//speedData = msg.data;
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
	nh.advertise(str);
	nh.advertise(gyro);
	nh.advertise(accel);
	nh.advertise(rpm_right);
	nh.advertise(rpm_left);
	nh.advertise(right_can_msg);
	nh.advertise(left_can_msg);
	nh.advertise(id);
//	nh.subscribe(sideD);
//	nh.subscribe(speedD);
	nh.subscribe(rpm_left_sub);
	nh.subscribe(rpm_right_sub);


	NBT_init(&right_can_msg_nbt, 9);
	NBT_init(&left_can_msg_nbt, 9);
	NBT_init(&rpm_left_nbt, 9);
	NBT_init(&rpm_right_nbt, 9);
	NBT_init(&gyro_nbt, 9);
	NBT_init(&vector_nbt, 9);
	NBT_init(&accel_nbt, 9);
	NBT_init(&ros_nbt, 1);
	NBT_init(&id_nbt, 9);
}

extern "C" void id_handler(CAN_RxHeaderTypeDef &wheel_RxHeader)
{
	  if (NBT_handler(&id_nbt))
	  {

		  id_msg.data = wheel_RxHeader.StdId;
		  //id_msg.data = a;
		  //a++;
		  id.publish(&id_msg);

	  }
}

extern "C" void right_can_msg_handler(void)
{
	  if (NBT_handler(&right_can_msg_nbt))
	  {
		  //id_msg.data = (uint32_t)wheel_RxHeader->StdId;
		  right_can.data = rightCount;
		  right_can_msg.publish(&right_can);
	  }
}

extern "C" void left_can_msg_handler(void)
{
	  if (NBT_handler(&left_can_msg_nbt))
	  {
		  //id_msg.data = (uint32_t)wheel_RxHeader->StdId;
		  left_can.data = leftCount;
		  left_can_msg.publish(&left_can);
	  }
}


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
//		  if (uint_msg_right.data !=0)
		  rpm_right.publish(&uint_msg_right);
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
		  rpm_left.publish(&uint_msg_left);
	  }
}

extern "C" void point_handler(void)
{
  if (NBT_handler(&point_nbt))
	 {
//		  point_msg.x = x;
//		  point_msg.y = y;
//		  point_msg.z = z;//
//	  	  point_msg.x = aPitch;
//	  	  point_msg.y = aRoll;
//	  	  point_msg.z = aYaw;
		  //point.publish(&point_msg);
//		  x++;
//		  y++;
//		  z++;
	 }

}

extern "C" void gyro_handler(void)
{
    if (NBT_handler(&gyro_nbt))
	   {

		  gyro_msg.x = gyroX;
		  gyro_msg.y = gyroY;
		  gyro_msg.z = gyroZ;
		  gyro.publish(&gyro_msg);
//		  vx++;
//		  vy++;
//		  vz++;

       }
}

extern "C" void accel_handler(void)
{
    if (NBT_handler(&accel_nbt))
	   {
          accel_msg.x = accelX;
		  accel_msg.y = accelY;
		  accel_msg.z = accelZ;
		  accel.publish(&accel_msg);
//		  vx++;
//		  vy++;
//		  vz++;

       }
}


extern "C" void spinOnce(void)
{
	//nh.spinOnce();
	  if (NBT_handler(&ros_nbt))
	  {

		nh.spinOnce();

	  }
}

