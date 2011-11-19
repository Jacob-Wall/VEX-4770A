#pragma config(Sensor, dgtl1, quad, sensorQuadEncoder)                      // configuration for the quadrature encoder (the sensor): name: quad; port: digital port 1
#pragma config(Motor, port2, liftMotor, tMotorNormal, openLoop, reversed)   // configuration for the motor: name: liftMotor; port: port 2

/***************************************************************************************************************************
 * pid.c - an example of a PID loop in ROBOTC
 * by Jacob Wall
 * Wednesday, November 16, 2011
 *
 * This is a simple PID loop implemented in ROBOTC that will maintain the position of an arm (at a previously set position).
 ***************************************************************************************************************************/


task main()
{
  // constants - they must be "tuned" at a later time to ensure more stability and faster reaction
  const float Kp = 1;      // proportional
  const float Ki = 1;      // integral
  const float Kd = 1;      // derivative

  const int target = 100;   // the target value for the arm's position, note that this may vary, also note that this will NOT be constant when practically applied

  float integral = 0;      // this will store the integral, initialized to 0
  float derivative = 0;    // this will store the derivative, initialized to 0

  float currentError = 0;  // this will store the current error, initialized to 0
  float lastError = 0;     // this will store the error of the previous iteration, initialized to 0

  int liftPos = 0;          // this will store the current value read in through the sensor, initialized to 0

  float output = 0;        // this will store the output value of the PID, and will be applied to the appropriate motor, initialized to 0

  ClearTimer(T1); // reset the timer

  // this is an intentional infinite loop, it will constantly monitor and correct using the PID control loop
  while (true)
  {
    liftPos = SensorValue[quad]; // reads the value of the quadrature encoder (sensor) into the liftPos (lift position)

    currentError = target - liftPos; // calculate the error

    integral += currentError * time1[T1];                // calculate the integral
    derivative = (currentError - lastError) / time1[T1]; // calculate the derivative

    ClearTimer(T1); // clears the timer, this will allow us to calculate the delta time each iteration

    output = Kp * currentError + Ki * integral + Kd * derivative; // calculate the output (or the result of the PID loop), note that the proportional is calculated here

    motor[liftMotor] = output; // apply the output to the motor, note that there may be certain constants to factor in here, but such constants are optional

    lastError = currentError; // set the lastError
  }
}
