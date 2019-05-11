#include <Geometry.h>

/*
 * This example sketch will show you how to integrate the rate measurements from a gyro sensor without having to worry about gimbal lock or
 * truncating the integrated value back to +-pi.
 */

Rotation orientation;
float xdot, ydot, zdot, dt = 0.5;

// Just a stub to gather some gyro-ish data
void GetGyroData(float &xdot, float &ydot, float &zdot)
{
  xdot = random(-1,1) / 10.0;
  ydot = random(-1,1) / 10.0;
  zdot = random(-1,1) / 10.0;
}

void setup() 
{
  randomSeed(10); // seed the random sequence used in GetGyroData
  Serial.begin(115200);
}

void loop()
{  
  // First we get some gyro data, in practice you'd replace this function with an actual read from your sensor
  GetGyroData(xdot, ydot, zdot);

  // Apply the sequence of rotations reported by the gyro to the current pose
  orientation.RotateX(xdot * dt); // assuming x/y/zdot are in radians per second
  orientation.RotateY(ydot * dt);
  orientation.RotateZ(zdot * dt);

  // Print the orientation to the console. If you'd rather just look at the Euler angles then just add .ToEulerAngles() to the end of orientation
  Serial << orientation << "\n";

  delay(500);
}
