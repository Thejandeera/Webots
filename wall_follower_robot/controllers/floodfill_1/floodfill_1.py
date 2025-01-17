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

def move_forward(robot):
    """Moves the robot forward by a specified distance."""
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(MAX_SPEED)
    right_motor.setVelocity(MAX_SPEED)

    time_needed = 2
    timestep = int(robot.getBasicTimeStep())
    start_time = robot.getTime()

    sensor = robot.getDevice('ps0')
    sensor.enable(timestep)
    
    while robot.step(timestep) != -1:
        if sensor.getValue() > 80:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            return True
        if robot.getTime() - start_time >= time_needed:
            break
            
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        return False

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

def find_next(position, maze, target_value):
    x = position[0]
    y = position[1]
    current_val = maze[x][y]
    next = []
    
    # For path to 0
    if target_value == 0:
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
    # For path to 16
    else:
        if x < 9:
            if current_val + 1 == maze[x+1][y]:
                next.append([x+1, y])
        if x > 0:
            if current_val + 1 == maze[x-1][y]:
                next.append([x-1, y])
        if y < 9:
            if current_val + 1 == maze[x][y+1]:
                next.append([x, y+1])
        if y > 0:
            if current_val + 1 == maze[x][y-1]:
                next.append([x, y-1])
    
    print(next)
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
    
    facing_direction = 'x'
    position = [9, 8]

    maze = [[24, 22, 22, 19, 18, 17, 16, 11, 10, 9],
            [23, 22, 21, 20, 17, 16, 15, 12, 7, 8],
            [24, 25, 22, 19, 18, 10, 14, 13, 6, 8],
            [25, 24, 23, 6, 5, 2, 1, 0, 5, 6],
            [26, 25, 24, 7, 4, 3, 2, 1, 4, 7],
            [27, 26, 25, 8, 5, 4, 3, 2, 3, 8],
            [29, 27, 10, 9, 7, 8, 10, 3, 3, 5],
            [29, 28, 11, 12, 23, 14, 9, 8, 7, 6],
            [30, 31, 34, 35, 38, 15, 12, 11, 8, 11],
            [33, 32, 33, 36, 37, 16, 13, 10, 9, 10]]
    
    # First navigate to 0
    target_value = 0
    while maze[position[0]][position[1]] != target_value:
        next = find_next(position, maze, target_value)
        
        for nex in next:
            move = how_to_move(position, nex, facing_direction)
            if move == 'f':
                if move_forward(robot):
                    if len(next) > 1:
                        continue
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

        print(f"Current direction: {facing_direction}")
        print(f"Current position: {position}")
    
    print("Reached value 0, now heading to 16")
    
        # After reaching 0, navigate to 16
    target_value = 16
    while maze[position[0]][position[1]] != target_value:
        next = find_next(position, maze, target_value)
        
        for nex in next:
            move = how_to_move(position, nex, facing_direction)
            if move == 'f':
                if move_forward(robot):
                    if len(next) > 1:
                        continue
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

        print(f"Current direction: {facing_direction}")
        print(f"Current position: {position}")
    
    print("Reached value 16!")

  