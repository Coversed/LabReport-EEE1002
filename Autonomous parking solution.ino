const int trigPin = 19; //Replace with Ultrasonic pins being used
const int echoPin = 25; //Replace with Ultrasonic pins being used
int angle;
int distance;
long duration;
float distance;
float total_180rotation, total_90rotation;
int leftMotor_speed, rightMotor_speed, servoAngle;
#include "mpu9250.h"
/* Mpu9250 object */
bfs::Mpu9250 imu;

void setup()
{
  Serial.begin(9600);
  While(!Serial) {}
  Wire.begin();   
  Wire.setClock(400000);
  /* I2C bus,  0x68 address */
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }
 
 // Sets trigger to output and echo to input
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);

}

void loop()
{
  
  leftMotor_speed = 255; //Max speed left wheel
  rightMotor_speed = 255; //Max speed right wheel
  servoAngle = 90; // Servo centered
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle); //Transmit information to Arduino
  delay(1000); //Drive for 1 second (1000 milliseconds)
  leftMotor_speed = 0; //No speed left
  rightMotor_speed = 0; // No speed right
  servoAngle = 90; // Servo centered
 Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle); //Transmit information to Arduino
 //Rotate 180 degrees
  leftMotor_speed = 180;
  rightMotor_speed = 0;
  servo angle = 135;
  while(total_180rotation <= 3.141592) // While rotation is less than pi which is 180 degrees in radians increment total rotation
  {
    total_180rotation = (total_180rotation + imu.gyro_x_radps());
  }
  //Reverse
  leftMotor_speed = -255; //Max speed backwards left wheel
  rightMotor_speed = -255;//Max speed backwards right wheel
  servoAngle = 90;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle); //Transmit information to Arduino
  distance = ultrasonic();
  while(distance >= 10)
  {
    // Continue to check
  }
  // Stop
  leftMotor_speed = 0; //No speed left
  rightMotor_speed = 0; // No speed right
  servoAngle = 90; // Servo centered
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle); //Transmit information to Arduino
  delay(1000) //stop for a second
  //Rotate 90 degrees
  leftMotor_speed = 180;
  rightMotor_speed = 0;
  servoAngle = 135;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  while(total_90rotation <= 1.5708) // While rotation is less than pi/2 which is 90 degrees in radians increment total rotation
  {
    total_90rotation = (total_90rotation + imu.gyro_x_radps());
  }
  //Reverse
  leftMotor_speed = -255; //Max speed backwards left wheel
  rightMotor_speed = -255;//Max speed backwards right wheel
  servoAngle = 90;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle); //Transmit information to Arduino
    while(distance >= 10)
  {
    // Continue to check
  }
  // Stop
  leftMotor_speed = 0; //No speed left
  rightMotor_speed = 0; // No speed right
  servoAngle = 90; // Servo centered
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle); //Transmit information to Arduino
  //End program
  Exit (0);
}



float ultrasonic()
{
  float distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return (duration * 0.0343 / 2);
}


void Transmit_to_arduino(int leftMotor_speed, int rightMotor_speed, int servoAngle)
{
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to arduino
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));  // first byte of leftMotor, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF)); // second byte of leftMotor, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8)); // first byte of rightMotor, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF));  // second byte of rightMotor, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8)); // first byte of servoAngle, containing bits 16 to 9
  Wire.write((byte)(servoAngle & 0x000000FF));  // second byte of servoAngle, containing the 8 LSB - bits 8 to 1          
  Wire.endTransmission(); // end transmission
}
