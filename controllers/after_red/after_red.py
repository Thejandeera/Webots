"""after_red controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

MAX_SPEED = 6.28  # Maximum speed in rad/s

def inisialize_sensor(timestep):
    proxy_sensors = []
    for i in range(8):
        sensor_name = 'ps'+str(i)
        proxy_sensors.append(robot.getDevice(sensor_name))
        proxy_sensors[i].enable(timestep)
        
    return proxy_sensors
# create the Robot instance.
def move_forward(robot):
    

    """Moves the robot forward by a specified distance."""
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(MAX_SPEED)
    right_motor.setVelocity(MAX_SPEED)

    # Calculate the time needed to move the distance

    time_needed = 2
    start_time = robot.getTime()
    timestep = int(robot.getBasicTimeStep())
    
    #getting sensor readings
    #getting sensor readings
    proxy_sensors = inisialize_sensor(timestep)


    while robot.step(timestep) != -1:
        # print(robot.getTime() - start_time)
        sensor_value = proxy_sensors[0].getValue() > 80

        if robot.getTime() - start_time >= time_needed or sensor_value:
            break
            
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    return sensor_value

def turn_left(robot):
    timestep = int(robot.getBasicTimeStep())

    """Turns the robot left by a specified angle in degrees."""
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
     # Initialize the gyro sensor
    gyro = robot.getDevice('gyro')
    gyro.enable(timestep)

    # Calculate the time needed for the turn
    speed = 2

    left_motor.setVelocity(-speed)  # Reverse left wheel
    right_motor.setVelocity(speed)  # Forward right wheel

    time_x = robot.getTime()
    angle = 0
    while robot.step(timestep) != -1:
        # print(robot.getTime() - start_time)
        gyro_values = gyro.getValues()
        time_y = robot.getTime()
        angle += gyro_values[2] * (time_y - time_x)
        time_x = time_y
        if(angle >= 11873 or angle <= -11873):
            break
            
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
def turn_right(robot):
    timestep = int(robot.getBasicTimeStep())

    """Turns the robot left by a specified angle in degrees."""
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    # Initialize the gyro sensor
    gyro = robot.getDevice('gyro')
    gyro.enable(timestep)

    # Calculate the time needed for the turn
    speed = 2

    time_needed = 0.8

    left_motor.setVelocity(speed)  # Reverse left wheel
    right_motor.setVelocity(-speed)  # Forward right wheel

    time_x = robot.getTime()
    angle = 0
    while robot.step(timestep) != -1:
        # print(robot.getTime() - start_time)
        gyro_values = gyro.getValues()
        time_y = robot.getTime()
        angle += gyro_values[2] * (time_y - time_x)
        time_x = time_y
        if(angle >= 11873 or angle <= -11873):
            break

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
def turn_back(robot):
    timestep = int(robot.getBasicTimeStep())

    """Turns the robot left by a specified angle in degrees."""
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    # Initialize the gyro sensor
    gyro = robot.getDevice('gyro')
    gyro.enable(timestep)

    # Calculate the time needed for the turn
    speed = 2

    left_motor.setVelocity(speed)  # Reverse left wheel
    right_motor.setVelocity(-speed)  # Forward right wheel

    time_x = robot.getTime()
    angle = 0
    while robot.step(timestep) != -1:
        # print(robot.getTime() - start_time)
        gyro_values = gyro.getValues()
        time_y = robot.getTime()
        angle += gyro_values[2] * (time_y - time_x)
        time_x = time_y
        if(angle >= 23700 or angle <= -23700):
            break

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
                
if __name__ == "__main__":
    robot = Robot()
    
    move_yellow = ['f', 'f', 'l', 'r', 'r', 'l', 'l', 'f', 'f', 'r', 'l', 'l', 'f', 'f', 'r', 'f'] 
    move_pink = ['b', 'f', 'f', 'l', 'f', 'f', 'r', 'r', 'l', 'f', 'f' ,'r', 'r', 'l', 'l','r', 'f', 'r', 'f', 'f']
    move_brown = ['b', 'f', 'r']
    move_green = ['l', 'f', 'f', 'f', 'r', 'l', 'l', 'f', 'l', 'f', 'r', 'r', 'f','l','f','f','f', 'f','l', 'r', 'l', 'f', 'f', 'f','f'] 
    
    path = [move_yellow, move_pink, move_brown, move_green]
    
    
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
    



        
    # #input the position 
    # facing_direction = 'x'

            
    # while maze[position[0]][position[1]] != 0:
        # next = find_next(position, maze)

        
        
        # for nex in next:
            # move = how_to_move(position, nex, facing_direction)          
            # print(move)
            # print(f"current pos: {position} next: {nex}")        
            # position[0], position[1] = nex[0], nex[1] 
            # if move == 'f':
                # if move_forward(robot):
                    # if len(next) > 1:
                        # continue
                    # else:
                        # position[0], position[1] = nex[0], nex[1]
                        # break
                # else:
                    # position[0], position[1] = nex[0], nex[1]
                    # break
                
            # elif move == 'l':
                # turn_left(robot)
                # if facing_direction == 'x':
                    # facing_direction = 'y'
                # elif facing_direction == '-x':
                    # facing_direction = '-y'
                # elif facing_direction == 'y':
                    # facing_direction = '-x'
                # elif facing_direction == '-y':
                    # facing_direction = 'x'
                # if move_forward(robot):
                    # if len(next) > 1:
                        # continue
                    # else:
                        # position[0], position[1] = nex[0], nex[1]
                        # break
                # else:
                    # position[0], position[1] = nex[0], nex[1]
                    # break
    
            # elif move == 'r':
                # turn_right(robot)
                # if facing_direction == 'x':
                    # facing_direction = '-y'
                # elif facing_direction == '-x':
                    # facing_direction = 'y'
                # elif facing_direction == 'y':
                    # facing_direction = 'x'
                # elif facing_direction == '-y':
                    # facing_direction = '-x'
                # if move_forward(robot):
                    # if len(next) > 1:
                        # continue
                    # else:
                        # position[0], position[1] = nex[0], nex[1]
                        # break 
                # else:
                    # position[0], position[1] = nex[0], nex[1]
                    # break
            
