#include <Gyro.h>
#include <Wire.h>


Gyro gyro;
int sampleNum=500; 
int dc_offset_z,dc_offset_x,dc_offset_y;

unsigned long time; 
int sampleTime=10; 
int rate;

double noise=0; 

int prev_rate=0; 
double angle=0; 

void setup() {
  Serial.begin(115200);
 

if (!gyro.init())
{
  Serial.println("Failed to autodetect gyro type!");
  while (1);
  //Calculate initial DC offset and noise level of gyro 
}
gyro.enableDefault();

}

void loop() {

gyro.read();
Serial.print("x:");
Serial.print(gyro.g.x);
Serial.print("\ty:");
Serial.print(gyro.g.y);
Serial.print("\tz");
Serial.println(gyro.g.z);
delay(50);


}

  

