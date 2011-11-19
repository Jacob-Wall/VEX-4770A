#pragma config(Sensor, dgtl1, quad, sensorQuadEncoder)                      // configuration for the quadrature encoder (the sensor): name: quad; port: digital port 1
#pragma config(Motor, port2, liftMotor, tMotorNormal, openLoop, reversed)   // configuration for the motor: name: liftMotor; port: port 2

/***************************************************************************************************************************
 * pid.c - an example of a PID loop in ROBOTC
 * by Jacob Wall
 * Friday, November 18, 2011
 *
 * This is a simple PID loop implemented in ROBOTC that will maintain the position of an arm (at a previously set position).
 ***************************************************************************************************************************/

task maintainArmPosition(); // forward declaration

static bool pid_running = false; // used to determine whether or not the PID loop should be running
static float pid_targetValue = 100;  // the target value, this allows the target value to be changed, but its initialized to 0

task main()
{
  pid_running = true; // allowed the PID to run
  StartTask(maintainArmPosition); // start the task

  // allows the arm to move up and down (based on vexRT[Ch1], I have no idea what channel that is, I just made it up)
  while (true)
  {
    pid_targetValue += vexRT[Ch1]; // you may want to multiply the signal by a constant to reduce it, but we should wait until we actually have a robot to test out

    wait10Msec(5); // waits for 50 milliseconds
  }
}

// this task has the duty of maintaining the arms position (via the PID):
task maintainArmPosition()
{
  SensorValue[quad] = 0; // zeros the quadrature encoder

  // constants - these will need to be tuned later
  const float Kp = 1;
  const float Ki = 1;
  const float Kd = 1;
  // note: you may want to remove the "const" for tuning, that way you can change the values in the debugger

  // these variables will hold the derivative and integral for each iteration of the loop
  float derivative = 0;
  float integral = 0;

  // this will store the currentValue (read in from the sensor)
  float currentValue = 0;

  // these will be used to store the currentError (for the iteration) and the error of the previous iteration
  float currentError = 0;
  float previousError = 0;

  // this will store the output from the PID loop (after each iteration)
  float pidOutput = 0;

  while (true)
  {
    if (pid_running == true) // ensure that the PID should be running
    {
      currentValue = SensorValue[quad]; // get the latest value

      currentError = pid_targetValue - currentValue; // calculate the error, allegedly you can also do currentValue - targetValue if you'd prefer

      // TODO: Add some sort of checking to ensure that the integral does not exceed certain limits
      //   Option 1: use a limit:
      //     integral = abs(integral) < INTEGRAL_LIMIT ? integral + currentError : 0;
      //   Option 2: dampen the integral:
      //     integral = (2/3)*integral + currentError;
      //   Option 3: zero the integral when it changes signs or the error is zero
      //     if (currentError == 0 || (currentError > 0 && previousError < 0) || (currentError < 0 && previousError > 0)) integral = 0;
      //     else integral += currentError;

      integral += currentError; // calculate the integral

      derivative = currentError - previousError; // calculate the derivative

      previousError = currentError; // set the previousError

      pidOutput = (Kp * currentError) + (Ki * integral) + (Kd * derivative); // calculate the appropriate output

      // note: you may want to implement overflow protection, to prevent the pidOutput from being less than -127 or more than 127

      motor[liftMotor] = pidOutput; // apply the output
    }
    else // the PID loop wasn't supposed to be running
    {
      // zero the values
      currentError      = 0;
      previousError     = 0;
      integral          = 0;
      derivative        = 0;
      motor[liftMotor]  = 0;
    }

    wait10Msec(2.5); // waits for 25 milliseconds, this allows us to apply delta time to the constants
  }
}
