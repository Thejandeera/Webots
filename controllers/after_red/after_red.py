"""
Final Code After Red
"""

from controller import Robot

MAX_SPEED = 6.28  # Maximum speed in rad/s

#Angle for  turning 90 degrees
ANGLE = 11600

def inisialize_sensor(timestep):
    proxy_sensors = []
    for i in range(8):
        sensor_name = 'ps'+str(i)
        proxy_sensors.append(robot.getDevice(sensor_name))
        proxy_sensors[i].enable(timestep)
        
    return proxy_sensors

def move_forward(robot):
    

    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(MAX_SPEED)
    right_motor.setVelocity(MAX_SPEED)


    time_needed = 2
    start_time = robot.getTime()
    timestep = int(robot.getBasicTimeStep())

    proxy_sensors = inisialize_sensor(timestep)


    while robot.step(timestep) != -1:
        sensor_value = proxy_sensors[0].getValue() > 80

        if robot.getTime() - start_time >= time_needed or sensor_value:
            break
            
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    return sensor_value

def turn_left(robot):
    timestep = int(robot.getBasicTimeStep())

    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    gyro = robot.getDevice('gyro')
    gyro.enable(timestep)

    speed = 1

    left_motor.setVelocity(-speed)  # Reverse left wheel
    right_motor.setVelocity(speed)  # Forward right wheel

    time_x = robot.getTime()
    angle = 0
    while robot.step(timestep) != -1:
        gyro_values = gyro.getValues()
        time_y = robot.getTime()
        angle += gyro_values[2] * (time_y - time_x)
        time_x = time_y
        if(angle >= ANGLE or angle <= -ANGLE):
            break
            
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

#Turn Right    
def turn_right(robot):
    timestep = int(robot.getBasicTimeStep())

    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    gyro = robot.getDevice('gyro')
    gyro.enable(timestep)

    speed = 1

    time_needed = 0.8

    left_motor.setVelocity(speed)  # Reverse left wheel
    right_motor.setVelocity(-speed)  # Forward right wheel

    time_x = robot.getTime()
    angle = 0
    while robot.step(timestep) != -1:
        gyro_values = gyro.getValues()
        time_y = robot.getTime()
        angle += gyro_values[2] * (time_y - time_x)
        time_x = time_y
        if(angle >= ANGLE or angle <= -ANGLE):
            break

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

#Turn Back
def turn_back(robot):
    timestep = int(robot.getBasicTimeStep())

    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    gyro = robot.getDevice('gyro')
    gyro.enable(timestep)

    speed = 1

    left_motor.setVelocity(speed)  # Reverse left wheel
    right_motor.setVelocity(-speed)  # Forward right wheel

    time_x = robot.getTime()
    angle = 0
    while robot.step(timestep) != -1:
        gyro_values = gyro.getValues()
        time_y = robot.getTime()
        angle += gyro_values[2] * (time_y - time_x)
        time_x = time_y
        if(angle >= 23400 or angle <= -23400):
            break

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
                
if __name__ == "__main__":
    robot = Robot()
    
    
    """
    Hard Codded Paths
    """
    
    #Path to Red to Yellow
    move_yellow = ['f', 'f', 'l', 'r', 'r', 'l', 'l', 'f', 'f', 'r', 'l', 'l', 'f', 'f', 'r', 'f'] 
   
    #Path to Yellow to Pink
    move_pink = ['b', 'f', 'f', 'l', 'f', 'f', 'r', 'r', 'l', 'f', 'f' ,'r', 'r', 'l', 'l','r', 'f', 'r', 'f', 'f']
    
    #Path to Pink to Brown
    move_brown = ['b', 'f', 'r']
    
    #Path to Brown to Green
    move_green = ['l', 'f', 'f', 'f', 'r', 'l', 'l', 'f', 'l', 'f', 'r', 'r', 'f','l','f','f','f', 'f','l', 'r', 'l', 'f', 'f', 'f','f'] 
    
    
    #Adding path to a list
    path = [move_yellow, move_pink, move_brown, move_green]
    
    #Iterrate every path
    for color in path:
        for move in color:
            print(move)
            if move == 'f':
                if move_forward(robot):
                    continue
                      
            elif move == 'l':
                turn_left(robot)
                if move_forward(robot):
                   continue
        
            elif move == 'r':
                turn_right(robot)
                if move_forward(robot):
                   continue
            elif move == 'b':
                turn_back(robot)
 
   