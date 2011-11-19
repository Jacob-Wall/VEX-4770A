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
  // The PID loop has three main parts:
  // 1) Retrieve data from the quadrature encoder (sensor)
  // 2) Calculate PID values and apply them to the motor
  // 3) Prepare variables for the next loop iteration
  // 
  // In addition, it is wise to have a delay between cycles in order to give the timer decent values.

  // constants - they must be "tuned" at a later time to ensure more stability and faster reaction
  const float Kp = 1;      // proportional
  const float Ki = 1;      // integral
  const float Kd = 1;      // derivative

  const int target = 100;   // the target value for the arm's position, note that this may vary, also note that this will NOT be constant when practically applied
  // end constants

  // variables - these keep track of values significant to the PID loop between cycles
  float integral = 0; // this will store the integral
  int lastError = 0;  // this will store the error of the previous iteration used to calculate the D (derivative) term 

  // T1 will keep track of the time elapsed between each iteration in the loop
  ClearTimer(T1);

  // this is an intentional infinite loop, it will constantly monitor and correct using the PID control loop
  while (true)
  {
    // Optimization Note: The compiler will likely move the variable declarations outside the loop when compiled anyways,
    //                    so there's little to no overhead in defining them here.  It does create clearer code as we can clearly tell
    //                    which variables are and are not used between cycles of the loop.

    // Step 1: Retrieve data from the sensor.
    int liftPos = SensorValue[quad]; // quadrature encoders record in degrees

    // Step 2: Calculate PID values and apply them to the motor.
    int error = target - liftPos; // signed distance from target value

    // calculate the next step in the integral and weigh it according to the elapsed time
    // divide by 1000 to prevent the integral from reaching insanely high numbers and degrading accuracy
    integral += currentError * (time1[T1] / 1000);

    // calculate the derivative by noting the change in error   
    int derivative = error - lastError;

    // calculate the output by summing the 3 parts of PID and weighing them according to their respective tuning constants
    // Note: P (proportional) is calculated here
    int output = Kp * currentError + Ki * integral + Kd * derivative;

    // apply the output to the motor
    motor[liftMotor] = output;

    // Step 3: Prepare variables for the next iteration.
    lastError = error;
    ClearTimer(T1);

    // wait for a short amount of time so the timer will have a meaningful value
    wait1Msec(10); // 100 Hz
  }
}
