#include <PololuQTRSensors.h>
#include <OrangutanMotors.h>

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   7     // emitter is controlled by digital pin 2

// sensors 0 through 7 are connected to digital 
// pins 15,16,17,18,8,9,10,12 respectively
PololuQTRSensorsRC qtrrc((unsigned char[]) {12,10,9,8,18,17,16,15},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
OrangutanMotors motors;
// Motor speed set experimentally
int MotorSpeed = -55;
// Threshold set based on the lighting in room
int SensorThreshold = 750;


void setup()
{

}

void loop()
{
  // read the sensor values
  qtrrc.read(sensorValues,QTR_EMITTERS_ON);

  // then set the speed of the motors accordingly
  // this is done by slowing down one of the motors based on which sensor
  // picks up the line
  if (sensorValues[3] > SensorThreshold ) {
      motors.setM1Speed(MotorSpeed);
      motors.setM2Speed(MotorSpeed*0.9);
  }
  if (sensorValues[4] > SensorThreshold ) {
      motors.setM1Speed(MotorSpeed*0.9);
      motors.setM2Speed(MotorSpeed);
    }
  if (sensorValues[5] > SensorThreshold ) {
     motors.setM1Speed(MotorSpeed*0.7);
     motors.setM2Speed(MotorSpeed); 
    }
  if (sensorValues[2] > SensorThreshold ) {
     motors.setM1Speed(MotorSpeed);
     motors.setM2Speed(MotorSpeed*0.7);
  }
  if (sensorValues[6] > SensorThreshold ) {
     motors.setM1Speed(MotorSpeed*0);
     motors.setM2Speed(MotorSpeed);
  }
  if (sensorValues[1] > SensorThreshold ) {
      motors.setM1Speed(MotorSpeed);
      motors.setM2Speed(MotorSpeed*0);
    }
    
  if (sensorValues[7] > SensorThreshold ) {
     motors.setM1Speed(MotorSpeed*0);
     motors.setM2Speed(MotorSpeed); 
  }
  if (sensorValues[0] > SensorThreshold ) {
      motors.setM1Speed(MotorSpeed);
      motors.setM2Speed(MotorSpeed*0);
    }
    
}
