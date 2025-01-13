from controller import Robot, Motor

WHEEL_RADIUS = 0.0205  # Radius of the wheel in meters (example: 2 cm)
TARGET_DISTANCE = 1  
MAX_SPEED = 6.28

def find_next(position, maze):
    y = position[0]
    x = position [1]
    current_val = maze[x][y]
    move = []
    print(x+1, y+1)
    if x < 9:
        if current_val - 1 == maze[x+1][y]:
            move.append(['f', [x+1, y]])
    if y < 9:
        if current_val - 1 == maze[x][y+1]:
            move.append(['l', [x+1, y]])
    if y > 0:
        if current_val -1 == maze[x][y-1]:
            move.append('r')
    
    return move 



def run_robot(robot, maze):


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
        proxy_sensors.append(robot.getDevice(sensor_name))
        proxy_sensors[i].enable(timestep)
        
    position = [9, 0]

        
    while maze[position[0]][position[1]] == 0:
    
        while robot.step(timestep) != -1:
            for i in range(8):
                print(f"ind:{i}, val:{proxy_sensors[i].getValue()}")
                
            #Applying the wall following logic
            # left_wall = proxy_sensors[5].getValue() > 80
            # left_corner = proxy_sensors[6].getValue() > 80
            # front_wall = proxy_sensors[7].getValue() > 80
            
            left_speed = max_speed
            right_speed = max_speed
            
            next = find_next(position, maze)
            
            print(next)
            
            if next[0] == 'f':
                position[0] += 1
                next = find_next([])
    
                    
                
            leftMotor.setVelocity(left_speed)
            rightMotor.setVelocity(right_speed)
    



   
if __name__ == "__main__":
    my_robot = Robot()
    maze = [[24, 21, 20, 19, 18, 17, 16, 11, 10, 9],
            [23, 22, 21, 20, 17, 16, 15, 12, 7, 8],
            [24, 25, 22, 19, 18, 15, 14, 13, 6, 7],
            [25, 24, 23, 6, 5, 2, 1, 0, 5, 6],
            [26, 25, 24, 7, 4, 3, 2, 1, 4, 7],
            [27, 26, 25, 8, 5, 4, 3, 2, 3, 8],
            [28, 27, 10, 9, 7, 8, 10, 3, 4, 5],
            [29, 28, 11, 12, 23, 14, 9, 8, 7, 6],
            [30, 31, 34, 35, 38, 15, 12, 11, 8, 11],
            [33, 32, 33, 36, 37, 16, 13, 10, 9, 10]]
    run_robot(my_robot, maze)