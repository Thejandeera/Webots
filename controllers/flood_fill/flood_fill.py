from controller import Robot

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

def adjust_path(left_motor, right_motor, proxy_sensors):
    left_corner = proxy_sensors[6].getValue() > 80
    left_wall = proxy_sensors[5].getValue() > 80
    right_wall = proxy_sensors[2].getValue() > 80
    right_corner = proxy_sensors[1].getValue() > 80
    
    if left_wall or left_corner:
        right_motor.setVelocity(MAX_SPEED / 2)
    elif right_wall or right_corner:
        left_motor.setVelocity(MAX_SPEED / 2)
    else:
        left_motor.setVelocity(MAX_SPEED)
        right_motor.setVelocity(MAX_SPEED)

        

def move_forward(robot):
    

    """Moves the robot forward by a specified distance."""
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(MAX_SPEED)
    right_motor.setVelocity(MAX_SPEED)

    # Calculate the time needed to move the distance

    time_needed = 1.88
    start_time = robot.getTime()
    timestep = int(robot.getBasicTimeStep())
    
    #getting sensor readings
    proxy_sensors = inisialize_sensor(timestep)

    
    while robot.step(timestep) != -1:
        # print(robot.getTime() - start_time)
        sensor_value = proxy_sensors[0].getValue() > 80
        adjust_path(left_motor, right_motor, proxy_sensors)
        if robot.getTime() - start_time >= time_needed or sensor_value:
            break

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

def turn_left(robot):
    """Turns the robot left by a specified angle in degrees."""
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    # Calculate the time needed for the turn
    speed = MAX_SPEED / 2

    time_needed = 0.8

    left_motor.setVelocity(-speed)  # Reverse left wheel
    right_motor.setVelocity(speed)  # Forward right wheel

    start_time = robot.getTime()
    timestep = int(robot.getBasicTimeStep())
    while robot.step(timestep) != -1:
        # print(robot.getTime() - start_time)
        if robot.getTime() - start_time >= time_needed:
            break

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
def turn_right(robot):
    """Turns the robot left by a specified angle in degrees."""
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    # Calculate the time needed for the turn
    speed = MAX_SPEED / 2

    time_needed = 0.8

    left_motor.setVelocity(speed)  # Reverse left wheel
    right_motor.setVelocity(-speed)  # Forward right wheel

    start_time = robot.getTime()
    timestep = int(robot.getBasicTimeStep())
    while robot.step(timestep) != -1:
        # print(robot.getTime() - start_time)
        if robot.getTime() - start_time >= time_needed:
            break

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
def find_next(position, maze):
    x = position[0]
    y = position [1]
    current_val = maze[x][y]
    # print(x+1, y+1)
    if x < 9:
        if current_val - 1 == maze[x+1][y]:
            return [x+1, y]
    if x > 0:
        if current_val -1 == maze[x-1][y]:
            return [x-1, y]
    if y < 9:
        if current_val - 1 == maze[x][y+1]:
            return [x, y+1]
    if y > 0:
        if current_val -1 == maze[x][y-1]:
            return [x, y-1]
 
    
    return move 

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
    position = [9, 0]

    maze = [[24, 21, 20, 19, 18, 17, 16, 11, 10, 9],
            [23, 22, 21, 20, 17, 16, 15, 12, 7, 8],
            [24, 25, 22, 19, 18, 10, 14, 13, 6, 7],
            [25, 24, 23, 6, 5, 2, 1, 0, 5, 6],
            [26, 25, 24, 7, 4, 3, 2, 1, 4, 7],
            [27, 26, 25, 8, 5, 4, 3, 2, 3, 8],
            [29, 27, 10, 9, 7, 8, 10, 3, 4, 5],
            [29, 28, 11, 12, 23, 14, 9, 8, 7, 6],
            [30, 31, 34, 35, 38, 15, 12, 11, 8, 11],
            [33, 32, 33, 36, 37, 16, 13, 10, 9, 10]]
    
    # print(maze)
    # next = find_next(position, maze)   
    # print(next)
    
    
            
    while maze[position[0]][position[1]] != 0:
        print(position, maze[position[0]][position[1]])
        next = find_next(position, maze)
        move = how_to_move(position, next, facing_direction)          
        print(next, move)
        
        if move == 'f':
            move_forward(robot)
            
        elif move == 'l':
            turn_left(robot)
            move_forward(robot)
            if facing_direction == 'x':
                facing_direction = 'y'
            elif facing_direction == '-x':
                facing_direction = '-y'
            elif facing_direction == 'y':
                facing_direction = '-x'
            elif facing_direction == '-y':
                facing_direction = 'x'

        elif move == 'r':
            turn_right(robot)
            move_forward(robot)
            if facing_direction == 'x':
                facing_direction = '-y'
            elif facing_direction == '-x':
                facing_direction = 'y'
            elif facing_direction == 'y':
                facing_direction = 'x'
            elif facing_direction == '-y':
                facing_direction = '-x'


        print(facing_direction)
            
        position[0], position[1] = next[0], next[1] 
        print(position)
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        

    
