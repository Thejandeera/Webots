/*
 * File:          move_epuck.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>

// Added a new include file
#include <webots/motor.h>
#include <math.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
 void move_foward(WbDeviceTag left_motor, WbDeviceTag right_motor){
    wb_motor_set_position(left_motor, 10.88);
    wb_motor_set_position(right_motor, 10.88);
  
 }

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
    // get the motor devices
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  move_foward(left_motor, right_motor);
  
  
  while (wb_robot_step(TIME_STEP) != -1);

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
