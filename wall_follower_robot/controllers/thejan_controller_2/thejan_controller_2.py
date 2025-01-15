from controller import Robot, Camera, DistanceSensor
import numpy as np
from collections import deque

class ColorSequenceRobot:
    def __init__(self):
        # Initialize the robot
        self.robot = Robot()
        self.timestep = 32
        
        # Initialize motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        # Initialize camera
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)
        
        # Initialize distance sensors
        self.ps = []
        for i in range(8):
            sensor_name = f'ps{i}'
            self.ps.append(self.robot.getDevice(sensor_name))
            self.ps[i].enable(self.timestep)
        
        # Color sequence to follow
        self.color_sequence = [
            {'name': 'red', 'rgb': (255, 0, 0)},
            {'name': 'yellow', 'rgb': (255, 255, 0)},
            {'name': 'pink', 'rgb': (255, 0, 255)},
            {'name': 'brown', 'rgb': (165, 105, 30)},
            {'name': 'green', 'rgb': (0, 255, 0)}
        ]
        self.current_target = 0
        
        # Robot parameters
        self.MAX_SPEED = 15
        self.WALL_THRESHOLD = 80
        self.MIN_DIST = 120
        
        # Robot state
        self.state = 'EXPLORE'
        self.sub_state = 'FORWARD'
        self.turn_counter = 0
        
        # Maze memory
        self.visited_positions = set()
        self.current_position = (0, 0)
        self.current_direction = 0  # 0: North, 1: East, 2: South, 3: West
        self.path_memory = deque()
        self.last_turn_position = None
        
        # Movement tracking
        self.distance_since_last_record = 0
        self.position_record_interval = 0.25  # Record position every 0.25 meters
        self.last_sensor_readings = None
        
    def get_dominant_color(self, image):
        """Analyze camera image to detect dominant color"""
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        
        img = np.frombuffer(image, np.uint8).reshape((height, width, 4))
        center_img = img[height//3:2*height//3, width//3:2*width//3]
        avg_color = np.mean(center_img[:, :, :3], axis=(0, 1))
        
        return avg_color
    
    def color_matches_target(self, detected_color):
        """Check if detected color matches current target color with tolerance"""
        target_color = np.array(self.color_sequence[self.current_target]['rgb'])
        difference = np.abs(detected_color - target_color)
        return np.mean(difference) < 50
    
    def read_sensors(self):
        """Read and process distance sensor values"""
        return [sensor.getValue() for sensor in self.ps]
    
    def detect_walls(self, sensor_values):
        """Detect walls using sensor values"""
        front_wall = sensor_values[0] > self.WALL_THRESHOLD or sensor_values[7] > self.WALL_THRESHOLD
        left_wall = sensor_values[5] > self.WALL_THRESHOLD or sensor_values[6] > self.WALL_THRESHOLD
        right_wall = sensor_values[1] > self.WALL_THRESHOLD or sensor_values[2] > self.WALL_THRESHOLD
        
        return front_wall, left_wall, right_wall
    
    def update_position(self):
        """Update robot's position estimate based on movement"""
        if self.sub_state == 'FORWARD':
            self.distance_since_last_record += self.MAX_SPEED * self.timestep / 1000.0
            
            if self.distance_since_last_record >= self.position_record_interval:
                # Update position based on current direction
                if self.current_direction == 0:  # North
                    self.current_position = (self.current_position[0], self.current_position[1] + 1)
                elif self.current_direction == 1:  # East
                    self.current_position = (self.current_position[0] + 1, self.current_position[1])
                elif self.current_direction == 2:  # South
                    self.current_position = (self.current_position[0], self.current_position[1] - 1)
                elif self.current_direction == 3:  # West
                    self.current_position = (self.current_position[0] - 1, self.current_position[1])
                
                self.visited_positions.add(self.current_position)
                self.distance_since_last_record = 0
    
    def decide_movement(self, sensor_values):
        """Decide movement based on sensor values and maze memory"""
        front_wall, left_wall, right_wall = self.detect_walls(sensor_values)
        
        if self.sub_state == 'FORWARD':
            if front_wall:
                if not right_wall:
                    self.sub_state = 'TURN_RIGHT'
                    self.turn_counter = 0
                elif not left_wall:
                    self.sub_state = 'TURN_LEFT'
                    self.turn_counter = 0
                else:
                    self.sub_state = 'TURN_AROUND'
                    self.turn_counter = 0
            
        elif self.sub_state in ['TURN_LEFT', 'TURN_RIGHT', 'TURN_AROUND']:
            self.turn_counter += 1
            if self.turn_counter >= 20:  # Adjust this value based on robot's turning speed
                self.sub_state = 'FORWARD'
                if self.sub_state == 'TURN_RIGHT':
                    self.current_direction = (self.current_direction + 1) % 4
                elif self.sub_state == 'TURN_LEFT':
                    self.current_direction = (self.current_direction - 1) % 4
                elif self.sub_state == 'TURN_AROUND':
                    self.current_direction = (self.current_direction + 2) % 4
    
    def set_motor_speeds(self):
        """Set motor speeds based on current state and sub-state"""
        if self.sub_state == 'FORWARD':
            self.left_motor.setVelocity(self.MAX_SPEED)
            self.right_motor.setVelocity(self.MAX_SPEED)
        elif self.sub_state == 'TURN_LEFT':
            self.left_motor.setVelocity(-self.MAX_SPEED * 0.5)
            self.right_motor.setVelocity(self.MAX_SPEED * 0.5)
        elif self.sub_state == 'TURN_RIGHT':
            self.left_motor.setVelocity(self.MAX_SPEED * 0.5)
            self.right_motor.setVelocity(-self.MAX_SPEED * 0.5)
        elif self.sub_state == 'TURN_AROUND':
            self.left_motor.setVelocity(self.MAX_SPEED * 0.5)
            self.right_motor.setVelocity(-self.MAX_SPEED * 0.5)
    
    def run(self):
        """Main control loop"""
        while self.robot.step(self.timestep) != -1:
            # Get sensor readings
            sensor_values = self.read_sensors()
            camera_image = self.camera.getImage()
            detected_color = self.get_dominant_color(camera_image)
            
            # Check if we've completed the sequence
            if self.current_target >= len(self.color_sequence):
                self.left_motor.setVelocity(0)
                self.right_motor.setVelocity(0)
                break
            
            # Update position and memory
            self.update_position()
            
            # Check if current target color is found
            if self.color_matches_target(detected_color):
                # If we're close enough to the color
                if max(sensor_values[0], sensor_values[7]) > self.WALL_THRESHOLD:
                    print(f"Found color: {self.color_sequence[self.current_target]['name']}")
                    self.current_target += 1
                    if self.current_target < len(self.color_sequence):
                        # Reset exploration for next color
                        self.visited_positions.clear()
                        self.path_memory.clear()
                    continue
            
            # Decide and execute movement
            self.decide_movement(sensor_values)
            self.set_motor_speeds()
            
            # Store sensor readings for next iteration
            self.last_sensor_readings = sensor_values

# Create and run the robot controller
robot = ColorSequenceRobot()
robot.run()