from controller import Robot, Motor, Gyro, DistanceSensor

# Constants
WHEEL_RADIUS = 0.0205  # Wheel radius in meters
MAX_SPEED = 6.28  # Maximum speed in rad/s

def initialize_sensor(timestep):
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

    time_needed = 2
    start_time = robot.getTime()
    timestep = int(robot.getBasicTimeStep())
    
    proxy_sensors = initialize_sensor(timestep)

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

    speed = 2

    left_motor.setVelocity(-speed)
    right_motor.setVelocity(speed)

    time_x = robot.getTime()
    angle = 0
    while robot.step(timestep) != -1:
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

    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    gyro = robot.getDevice('gyro')
    gyro.enable(timestep)

    speed = 2

    left_motor.setVelocity(speed)
    right_motor.setVelocity(-speed)

    time_x = robot.getTime()
    angle = 0
    while robot.step(timestep) != -1:
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
    y = position[1]
    current_val = maze[x][y]
    next = []
    if x < 9:
        if current_val - 1 == maze[x+1][y]:
            next.append([x+1, y])
    if x > 0:
        if current_val - 1 == maze[x-1][y]:
            next.append([x-1, y])
    if y < 9:
        if current_val - 1 == maze[x][y+1]:
            next.append([x, y+1])
    if y > 0:
        if current_val - 1 == maze[x][y-1]:
            next.append([x, y-1])
            
    return next

def find_next_ascending(position, maze, current_target):
    """Find next position when moving in ascending order towards target."""
    x = position[0]
    y = position[1]
    next = []
    
    directions = [(1,0), (-1,0), (0,1), (0,-1)]
    for dx, dy in directions:
        new_x, new_y = x + dx, y + dy
        if (0 <= new_x < 10 and 0 <= new_y < 10 and 
            maze[new_x][new_y] > maze[x][y] and 
            maze[new_x][new_y] <= current_target):
            next.append([new_x, new_y])
    
    return next

def find_next_phase2(position, maze, current_val):
    """Find the next position with value current_val + 1."""
    x = position[0]
    y = position[1]
    next = []
    
    directions = [(1,0), (-1,0), (0,1), (0,-1)]
    for dx, dy in directions:
        new_x, new_y = x + dx, y + dy
        if (0 <= new_x < 10 and 0 <= new_y < 10 and 
            maze[new_x][new_y] == current_val + 1):
            next.append([new_x, new_y])
    
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

def execute_movement(robot, move, position, nex, facing_direction, next, current_value, maze):
    if move == 'f':
        if move_forward(robot):
            if len(next) > 1:
                return None, facing_direction, current_value
            else:
                return nex, facing_direction, maze[nex[0]][nex[1]]
        else:
            return nex, facing_direction, maze[nex[0]][nex[1]]
                    
    elif move == 'l':
        turn_left(robot)
        if facing_direction == 'x': new_facing = 'y'
        elif facing_direction == '-x': new_facing = '-y'
        elif facing_direction == 'y': new_facing = '-x'
        elif facing_direction == '-y': new_facing = 'x'
        
        if move_forward(robot):
            if len(next) > 1:
                return None, new_facing, current_value
            else:
                return nex, new_facing, maze[nex[0]][nex[1]]
        else:
            return nex, new_facing, maze[nex[0]][nex[1]]
    
    elif move == 'r':
        turn_right(robot)
        if facing_direction == 'x': new_facing = '-y'
        elif facing_direction == '-x': new_facing = 'y'
        elif facing_direction == 'y': new_facing = 'x'
        elif facing_direction == '-y': new_facing = '-x'
        
        if move_forward(robot):
            if len(next) > 1:
                return None, new_facing, current_value
            else:
                return nex, new_facing, maze[nex[0]][nex[1]]
        else:
            return nex, new_facing, maze[nex[0]][nex[1]]

# Main program
if __name__ == "__main__":
    robot = Robot()
    
    facing_direction = 'x'
    position = [3, 1]
    
    maze = [[24, 22, 22, 19, 18, 17, 16, 11, 10, 9],
            [23, 22, 21, 20, 17, 16, 15, 12, 7, 8],
            [24, 25, 22, 19, 18, 10, 14, 13, 6, 8],
            [25, 24, 23, 6, 5, 2, 1, 0, 5, 6],
            [26, 25, 24, 7, 4, 3, 2, 1, 4, 7],
            [27, 26, 25, 8, 5, 4, 3, 2, 3, 8],
            [29, 27, 10, 9, 7, 8, 10, 3, 4, 5],
            [29, 28, 11, 12, 23, 14, 9, 8, 7, 6],
            [30, 31, 34, 35, 38, 15, 12, 11, 8, 11],
            [33, 32, 33, 36, 37, 16, 13, 10, 9, 10]]
    
    print("Starting Phase 1: Moving to 0")
    # Phase 1: Navigate to 0
    while maze[position[0]][position[1]] != 0:
        next = find_next(position, maze)
        print(f"Current position: {position}, Current value: {maze[position[0]][position[1]]}")
        print(f"Available next positions: {next}")
        
        for nex in next:
            move = how_to_move(position, nex, facing_direction)
            print(f"Attempting move: {move} to position {nex}")
            
            new_position, new_facing, new_value = execute_movement(robot, move, position, nex, facing_direction, next, maze[position[0]][position[1]], maze)
            
            if new_position is not None:
                position = new_position
                facing_direction = new_facing
                break
    
    print("Starting Phase 2: Moving from 0 to 16")
    # Phase 2: Navigate from 0 to 16
    current_value = maze[position[0]][position[1]]
    while current_value < 16:
        print(f"Current position: {position}, Current value: {current_value}")
        next = find_next_phase2(position, maze, current_value)
        
        if not next:
            next = find_next_ascending(position, maze, 16)
        
        print(f"Available next positions: {next}")
        
        if next:
            for nex in next:
                move = how_to_move(position, nex, facing_direction)
                print(f"Attempting move: {move} to position {nex}")
                
                new_position, new_facing, new_value = execute_movement(robot, move, position, nex, facing_direction, next, current_value, maze)
                
                if new_position is not None:
                    position = new_position
                    facing_direction = new_facing
                    current_value = new_value
                    break
        
    print(f"Reached target! Final position: {position}, Final value: {maze[position[0]][position[1]]}")