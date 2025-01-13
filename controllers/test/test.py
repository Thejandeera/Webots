from controller import Robot, Motor



def run_robot(robot):


    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28

    # get the motor devices
    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    
    
    # set the target position of the motors
    leftMotor.setPosition(float('inf'))
    leftMotor.setVelocity(0.0)
    
    rightMotor.setPosition(float('inf'))
    rightMotor.setVelocity(0.0)
    
    proxy_sensors = []
    for i in range(8):
        sensor_name = 'ps'+str(i)
        proxy_sensors.append(robot.getDistanceSensor(sensor_name))
        proxy_sensors[i].enable(timestep)
    
    while robot.step(timestep) != -1:
        for i in range(8):
            print(f"ind:{i}, val:{proxy_sensors[i].getValue()}")
            
        #Applying the wall following logic
        left_wall = proxy_sensors[5].getValue() > 80
        left_corner = proxy_sensors[6].getValue() > 80
        front_wall = proxy_sensors[7].getValue() > 80
        
        left_speed = max_speed
        right_speed = max_speed
        
        if front_wall:
            print("Turning right on spot")
            left_speed = max_speed
            right_speed = -max_speed
        else:
            if left_wall:
                print("Driving Forward")
                left_speed = max_speed
                right_speed = max_speed
            else:
                print("Turning Left")
                left_speed = max_speed/8
                right_speed = max_speed 
                
            if left_corner:
                print("Adjust wall position")
                left_speed = max_speed
                right_speed = max_speed/8           
        
            
        leftMotor.setVelocity(left_speed)
        rightMotor.setVelocity(right_speed)


   
if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)