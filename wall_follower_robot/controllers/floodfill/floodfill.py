from controller import Robot,  Motor, Gyro, DistanceSensor

# Constants
WHEEL_RADIUS = 0.0205  # Wheel radius in meters
MAX_SPEED = 6.28  # Maximum speed in rad/s

def inisialize_sensor(timestep):
    proxy_sensors = []
    for i in range(8):
        sensor_name = 'ps'+str(i)
        proxy_sensors.append(robot.getDevice(sensor_name))
        proxy_sensors[i].enable(timestep)
        
    return proxy_sensors

def adjust_path(robot, proxy_sensors, timestep):
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_corner = proxy_sensors[6].getValue() > 80
    left_wall = proxy_sensors[5].getValue() > 70
    right_wall = proxy_sensors[2].getValue() > 80
    right_corner = proxy_sensors[1].getValue() > 80
    
    if left_wall:
        turn_right(robot)
        while robot.step(timestep) != 1:
            back = proxy_sensors[4].getValue() > 70
            print(back)
            if back:
                left_motor.setVelocity(MAX_SPEED)
                right_motor.setVelocity(MAX_SPEED)
            else:
                break
        turn_left(robot)
        

        

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
    
def find_next(position, maze):
    x = position[0]
    y = position [1]
    current_val = maze[x][y]
    # print(x+1, y+1)
    next = []
    if x < 9:
        if current_val - 1 == maze[x+1][y]:
            next.append([x+1, y])
    if x > 0:
        if current_val -1 == maze[x-1][y]:
            next.append([x-1, y])

    if y < 9:
        if current_val - 1 == maze[x][y+1]:
            next.append([x, y+1])
    if y > 0:
        if current_val -1 == maze[x][y-1]:
            next.append([x, y-1])
            
    return next
 
    

def how_to_move(current, next, facing_direction):
    if facing_direction == 'x':
        if current[1]+1 == next[1] or current[1]-1 == next[1]:
            return 'f'
        else:
            if current[0]+1 == next[0]:
                return 'r'
            else:
                return 'l'
                
    elif facing_direction == '-x':
        if current[1]+1 == next[1] or current[1]-1 == next[1]:
            return 'f'
        else:
            if current[0]+1 == next[0]:
                return 'l'
            else:
                return 'r'
                
    elif facing_direction == 'y':
        if current[0]+1 == next[0] or current[0]-1 == next[0]:
            return 'f'
        else:
            if current[1]+1 == next[1]:
                return 'r'
            else:
                return 'l'
                
    elif facing_direction == '-y':
        if current[0]+1 == next[0] or current[0]-1 == next[0]:
            return 'f'
        else:
            if current[1]+1 == next[1]:
                return 'l'
            else:
                return 'r'
            
            

# Main program
if __name__ == "__main__":
    robot = Robot()
    
    
    #input the position 
    facing_direction = 'x'
    position = [3]
    position.append(1)
    print(position)

    maze = [[24, 21, 20, 19, 18, 17, 16, 11, 10, 9],
            [23, 22, 21, 20, 17, 16, 15, 12, 7, 8],
            [24, 25, 22, 19, 18, 10, 14, 13, 6, 7],
            [25, 24, 23, 6, 5, 2, 1, 0, 5, 6],
            [26, 25, 24, 7, 4, 3, 2, 1, 4, 7],
            [27, 26, 25, 8, 5, 4, 3, 2, 3, 8],
            [28, 27, 10, 9, 7, 8, 10, 3, 4, 5],
            [29, 28, 11, 12, 23, 14, 9, 8, 7, 6],
            [30, 31, 34, 35, 38, 15, 12, 11, 8, 11],
            [33, 32, 33, 36, 37, 16, 13, 10, 9, 10]]
    
    # print(maze)
    # next = find_next(position, maze)   
    # print(next)
    
    
            
    while maze[position[0]][position[1]] != 0:
        # print(position, maze[position[0]][position[1]])
        next = find_next(position, maze)

        print(len(next))
        
        
        for nex in next:
            move = how_to_move(position, nex, facing_direction)          
            print(move)
            print(f"current pos: {position} next: {nex}")        
            # position[0], position[1] = nex[0], nex[1] 
            if move == 'f':
                if move_forward(robot):
                    if len(next) > 1:
                        continue
                    else:
                        position[0], position[1] = nex[0], nex[1]
                        break
                else:
                    position[0], position[1] = nex[0], nex[1]
                    break
                
            elif move == 'l':
                turn_left(robot)
                if facing_direction == 'x':
                    facing_direction = 'y'
                elif facing_direction == '-x':
                    facing_direction = '-y'
                elif facing_direction == 'y':
                    facing_direction = '-x'
                elif facing_direction == '-y':
                    facing_direction = 'x'
                if move_forward(robot):
                    if len(next) > 1:
                        continue
                    else:
                        position[0], position[1] = nex[0], nex[1]
                        break
                else:
                    position[0], position[1] = nex[0], nex[1]
                    break
    
            elif move == 'r':
                turn_right(robot)
                if facing_direction == 'x':
                    facing_direction = '-y'
                elif facing_direction == '-x':
                    facing_direction = 'y'
                elif facing_direction == 'y':
                    facing_direction = 'x'
                elif facing_direction == '-y':
                    facing_direction = '-x'
                if move_forward(robot):
                    if len(next) > 1:
                        continue
                    else:
                        position[0], position[1] = nex[0], nex[1]
                        break 
                else:
                    position[0], position[1] = nex[0], nex[1]
                    break
               
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        

    