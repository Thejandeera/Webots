from controller import Robot, Motor, Gyro



def run_robot(robot):

    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(2)
    right_motor.setVelocity(-2)


    timestep = int(robot.getBasicTimeStep())
    
    # Initialize the gyro sensor
    gyro = robot.getDevice('gyro')
    gyro.enable(timestep)
    
    print("Gyro values (rad/s):")
    
    # Main loop to print gyro values
    x = robot.getTime()
    z = 0
    while robot.step(timestep) != -1:
        # Read gyro values
        gyro_values = gyro.getValues()
        
        # Print the gyro values
        print(f"X: {gyro_values[0]:.4f}, Y: {gyro_values[1]:.4f}, Z: {gyro_values[2]:.4f}")
        y = robot.getTime()
        print(y-x)
        z += gyro_values[2] * (y - x)
        print(z)
        x = y
        if(z >= 11873 or z <= -11873):
            break
    
    
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    



   
if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)