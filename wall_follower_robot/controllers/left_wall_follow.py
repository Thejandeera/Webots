from controller import Robot

def run_robot(robot):
    """ Wall-following robot """
    
    # Get the time step of the current world
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28
    
    # Enable motors
    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    # Enable proximity sensors
    prox_sensors = []
    for ind in range(8):
        sensor_name = 'ps' + str(ind)
        prox_sensors.append(robot.getDistanceSensor(sensor_name))
        prox_sensors[ind].enable(timestep)
    
    # Main loop
    while robot.step(timestep) != -1:
        # Read the sensors
        for ind in range(8):
            print(f"ind: {ind}, val: {prox_sensors[ind].getValue()}")
        
        # Process sensor data
        left_wall = prox_sensors[5].getValue() > 80
        left_corner = prox_sensors[6].getValue() > 80
        front_wall = prox_sensors[7].getValue() > 80
        
        # Default motor speeds
        left_speed = max_speed
        right_speed = max_speed

        # Logic for wall following
        if front_wall:
            print("Turn right in place")
            left_speed = max_speed
            right_speed = -max_speed
        else:
            if left_wall:
                print("Drive forward")
                left_speed = max_speed
                right_speed = max_speed
            else:
                print("Turn left")
                left_speed = max_speed / 8
                right_speed = max_speed
            if left_corner :
                print("Came to close,turn right")
                left_speed = max_speed
                right_speed = max_speed/8

        # Set motor velocities
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    # Create the Robot instance
    my_robot = Robot()
    run_robot(my_robot)
