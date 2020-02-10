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
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
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
extern int8_t sideData;
extern int8_t speedData;
struct ringbuffer rb;
uint32_t a = 0;

ros::NodeHandle nh;
std_msgs::String str_msg;
std_msgs::Int32 int_msg;
geometry_msgs::Vector3 gyro_msg;
geometry_msgs::Vector3 accel_msg;

extern "C" void messageSide(const std_msgs::Byte& msg);
extern "C" void messageSpeed(const std_msgs::Byte& msg);
extern "C" void init_ROS();
ros::Publisher gyro("gyro", &gyro_msg);
ros::Publisher accel("accel", &accel_msg);
ros::Subscriber<std_msgs::Byte> sideD("side", messageSide);
ros::Subscriber<std_msgs::Byte> speedD("speed", messageSpeed);

static nbt_t publish_nbt;
static nbt_t vector_nbt;
static nbt_t point_nbt;
static nbt_t gyro_nbt;
static nbt_t accel_nbt;
static nbt_t ros_nbt;

extern "C" void messageSide(const std_msgs::Byte& msg)
{
	sideData = msg.data;
}

extern "C" void messageSpeed(const std_msgs::Byte& msg)
{
	speedData = msg.data;
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
	//nh.advertise(point);
	//nh.advertise(chatter);
	nh.advertise(gyro);
	nh.advertise(accel);
	nh.subscribe(sideD);
	nh.subscribe(speedD);

	NBT_init(&gyro_nbt, 9);
	NBT_init(&vector_nbt, 9);
	NBT_init(&accel_nbt, 9);
	NBT_init(&ros_nbt, 1);
//	NBT_init(&LED_nbt, 100);
}

extern "C" void version_handler(void)
{
	  if (NBT_handler(&publish_nbt))
	  {
//		  int_msg.data = a;
//		  digit.publish(&int_msg);
//		  a++;
		  char version[] = "version: 0.3";
		  str_msg.data = version;
		  //chatter.publish(&str_msg);

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

