#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <arduino_imu/arduinoImu.h>

#include <sstream>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
//#include <string.h>

#define DEFAULT_SERIALPORT "/dev/ttyUSB0"
#define DEFAULT_BAUDRATE 57600

// Global data
FILE *fpSerial = NULL; // serial port file pointer
ros::Publisher pubArduinoImu;  // yaw, pitch, roll
ros::Publisher pubImu;  // ros imu sensor message
std::string frame_id;
//int arIndex; // index number
int fd = -1;

//Initialize serial port, return file descriptor
FILE *serialInit(char * port, int baud) 
{
  int BAUD = 0;
  struct termios newtio;
  FILE *fp = NULL;

  // Open the serial port as a file descriptor for low level configuration
  // read/write, not controlling terminal for process,
  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY );
  if ( fd<0 ) {
    ROS_ERROR("serialInit: Could not open serial device %s",port);
    return fp;
  }

  // set up new settings
  memset(&newtio, 0,sizeof(newtio));
  newtio.c_cflag =  CS8 | CLOCAL | CREAD;  //no parity, 1 stop bit
  newtio.c_iflag = IGNCR;    //ignore CR, other options off
  newtio.c_iflag |= IGNBRK;  //ignore break condition
  newtio.c_oflag = 0;        //all options off
  newtio.c_lflag = ICANON;     //process input as lines

  // activate new settings
  tcflush(fd, TCIFLUSH);

  //Look up appropriate baud rate constant
  switch (baud) {
     case 115200:
	BAUD = B115200;
	break;
     case 57600:
     default:
	BAUD = B57600;
	break;
     case 38400:
        BAUD = B38400;
        break;
     case 19200:
        BAUD  = B19200;
        break;
     case 9600:
        BAUD  = B9600;
        break;
  }

  if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0) {
    ROS_ERROR("serialInit: Failed to set serial baud rate: %d", baud);
    close(fd);
    return NULL;
  }
  tcsetattr(fd, TCSANOW, &newtio);
  tcflush(fd, TCIOFLUSH);

  //Open file as a standard I/O stream
  fp = fdopen(fd, "r+");
  if (!fp) {
    ROS_ERROR("serialInit: Failed to open serial stream %s", port);
    fp = NULL;
  }
  return fp;
}


// Receive responses from Razor imu
// and publish as a ROS message
void *rcvThread(void *arg)
{
  arduino_imu::arduinoImu ar;
  sensor_msgs::Imu imuMsg;
  imuMsg.orientation_covariance[0] = 0.02;
  imuMsg.orientation_covariance[4] = 0.02;
  imuMsg.orientation_covariance[8] = 0.02;
  imuMsg.angular_velocity_covariance[0] = 0.000001;
  imuMsg.angular_velocity_covariance[4] = 0.000001;
  imuMsg.angular_velocity_covariance[8] = 0.000001;
  imuMsg.linear_acceleration_covariance[0] = 0.000001;
  imuMsg.linear_acceleration_covariance[4] = 0.000001;
  imuMsg.linear_acceleration_covariance[8] = 0.000001;

  int rcvBufSize = 140;
  char imuData[rcvBufSize];   //received string from imu
  char *bufPos;
  char* msgArray[9];

  // initialise and clear msgArray
  for (int j=0; j<9; j++) {
    msgArray[j] = new char[15];
  }

  ROS_INFO("rcvThread: serial receive thread running");

  ros::Rate loop_rate(20); // 10 Hz

  while (ros::ok()) {

    // clear the first line as it may be incomplete after flush
    fgets(imuData,rcvBufSize, fpSerial); 

    bufPos = fgets(imuData, rcvBufSize, fpSerial);
    if (bufPos != NULL) {
     ROS_DEBUG("IMU Received Data: %s", imuData);

      // Remove '#YPR='
      char truncMsg[rcvBufSize];
      memcpy(truncMsg, &imuData[5], 135);

      // break into 3 strings separated by ','
      int i=0;
      char * tok;
      tok=strtok(truncMsg, ",");
      while (tok != NULL) {
        strcpy(msgArray[i], tok);
        tok=strtok(NULL,",");
        i++;
      }

      // store yaw, pitch, roll as degrees
      ar.roll = (float) atof(msgArray[0]);
      ar.pitch = -(float) atof(msgArray[1]);
      ar.yaw = -(float) atof(msgArray[2]);
      ar.angular_velocity.x = (float) atof(msgArray[6]);
      ar.angular_velocity.y = -(float) atof(msgArray[7]);
      ar.angular_velocity.z = -(float) atof(msgArray[8]);
      ar.linear_acceleration.x = (float) atof(msgArray[3]);
      ar.linear_acceleration.y = -(float) atof(msgArray[4]);
      ar.linear_acceleration.z = -(float) atof(msgArray[5]);
      pubArduinoImu.publish(ar);

      // publish ros imu message
      tf::Quaternion qt = tf::createQuaternionFromRPY(ar.roll*(M_PI/180), ar.pitch*(M_PI/180), ar.yaw*(M_PI/180));
      imuMsg.orientation.x = qt[0];
      imuMsg.orientation.y = qt[1];
      imuMsg.orientation.z = qt[2];
      imuMsg.orientation.w = qt[3];
      imuMsg.angular_velocity.x = ar.angular_velocity.x;
      imuMsg.angular_velocity.y = ar.angular_velocity.y;
      imuMsg.angular_velocity.z = ar.angular_velocity.z;
      imuMsg.linear_acceleration.x = ar.linear_acceleration.x;
      imuMsg.linear_acceleration.y = ar.linear_acceleration.y;
      imuMsg.linear_acceleration.z = ar.linear_acceleration.z;
      imuMsg.header.stamp = ros::Time::now();
      imuMsg.header.frame_id = "imu_link";
      pubImu.publish(imuMsg);
	
      // broadcast transform
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
      transform.setRotation(qt );

      static tf::TransformBroadcaster br;
      br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "base_link",frame_id) );
    }

    // flush the contents of the input
    tcflush(fd, TCIFLUSH);

    loop_rate.sleep();
  }
  return NULL;
}


int main(int argc, char **argv)
{
  std::string portstr;    //port name
  char port[80];
  int baud;     //baud rate 



  pthread_t rcvThrID;   //receive thread ID
  int err;

  //Initialize ROS
  ros::init(argc, argv, "arduino_imu");
  ros::NodeHandle rosNode("~");
  ROS_INFO("arduino_imu starting");


  rosNode.param<std::string>("port",portstr , DEFAULT_SERIALPORT);
  strcpy(port, portstr.c_str());

  rosNode.param<int>("baud",baud , DEFAULT_BAUDRATE);

  rosNode.param<std::string>("frame_id",frame_id , "imu_link");


  ROS_INFO("connection initializing (%s) at %d baud", port, baud);
  fpSerial = serialInit(port, baud);
  if (!fpSerial ) {
    ROS_ERROR("unable to create a new serial port");
    return 1;
  }
  ROS_INFO("serial connection successful");


  //Setup to publish ROS messages
  pubArduinoImu = rosNode.advertise<arduino_imu::arduinoImu>("ImuRaw", 1);
  pubImu = rosNode.advertise<sensor_msgs::Imu>("Imu", 1);

  //Create receive thread
  err = pthread_create(&rcvThrID, NULL, rcvThread, NULL);
  if (err != 0) {
    ROS_ERROR("unable to create receive thread");
    return 1;
  }

  //Process ROS messages and send serial commands to uController
  ros::spin();

  fclose(fpSerial);
  ROS_INFO("arduino_imu stopping");
  return 0;
}

