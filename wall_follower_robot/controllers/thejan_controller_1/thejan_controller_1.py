from controller import Robot, Camera

# Constants
TIME_STEP = 64
MAX_SPEED = 6.28

# Initialize robot
robot = Robot()

# Get devices
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Enable proximity sensors
sensors = []
sensor_names = [
    'ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7'
]
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    sensors.append(sensor)

# Enable camera
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

# Helper function to check for obstacles
def check_obstacles():
    values = [sensor.getValue() for sensor in sensors]
    front = values[0] > 100 or values[7] > 100  # Front sensors (ps0 and ps7)
    left = values[5] > 100  # Left sensors (ps5)
    right = values[2] > 100  # Right sensors (ps2)
    return front, left, right

# Helper function to identify colors
def identify_color():
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    # Check the central pixel
    r = camera.imageGetRed(image, width, width // 2, height // 2)
    g = camera.imageGetGreen(image, width, width // 2, height // 2)
    b = camera.imageGetBlue(image, width, width // 2, height // 2)

    if r > 200 and g < 100 and b < 100:
        return 'Red'
    elif r < 100 and g > 200 and b < 100:
        return 'Green'
    elif r < 100 and g < 100 and b > 200:
        return 'Blue'
    elif r > 200 and g > 200 and b < 100:
        return 'Yellow'
    else:
        return 'None'

# Function to move the robot
def move_forward():
    left_motor.setVelocity(MAX_SPEED)
    right_motor.setVelocity(MAX_SPEED)

def turn_left():
    left_motor.setVelocity(MAX_SPEED * 0.2)
    right_motor.setVelocity(MAX_SPEED)

def turn_right():
    left_motor.setVelocity(MAX_SPEED)
    right_motor.setVelocity(MAX_SPEED * 0.2)

def stop():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

# Improved navigation algorithm
def navigate_maze():
    while robot.step(TIME_STEP) != -1:
        front, left, right = check_obstacles()
        color = identify_color()

        # Print color detection
        if color != 'None':
            print(f"Color detected: {color}")

        # Decision logic
        if front:  # Obstacle ahead
            if not right:  # Try turning right
                turn_right()
            elif not left:  # If right is blocked, turn left
                turn_left()
            else:  # Dead end, turn around
                turn_left()
                robot.step(TIME_STEP * 10)  # Give time to turn
        else:
            if left:  # Keep some distance from the left wall
                turn_right()
            else:  # Move forward if path is clear
                move_forward()

# Main loop
navigate_maze()
