from controller import Robot, Camera, DistanceSensor, Motor
import numpy as np

class EPuckController:
    def __init__(self):
        # Initialize the robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Initialize motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.max_speed = 6.28  # Max speed in radians/second
        
        # Initialize camera
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)
        
        # Initialize distance sensors
        self.ps = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            self.ps.append(self.robot.getDevice(sensor_name))
            self.ps[i].enable(self.timestep)
            
        # Color pattern to follow
        self.color_pattern = [
            (255, 0, 0),    # Red
            (255, 255, 0),  # Yellow
            (255, 0, 255),  # Pink
            (165, 105, 30), # Brown
            (0, 255, 0),    # Green
        ]
        self.current_color_index = 0
        
        # Navigation state
        self.state = 'SCAN'
        self.visited = set()
        self.current_position = (0, 0)
        self.direction = 0  # 0: North, 1: East, 2: South, 3: West
        
    def get_color_from_camera(self):
        """Detect dominant color from camera image"""
        image = self.camera.getImage()
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        
        # Sample center region of image
        center_x = width // 2
        center_y = height // 2
        sample_size = 10
        
        r_sum = g_sum = b_sum = 0
        count = 0
        
        for y in range(center_y - sample_size, center_y + sample_size):
            for x in range(center_x - sample_size, center_x + sample_size):
                if 0 <= x < width and 0 <= y < height:
                    idx = (y * width + x) * 4
                    r_sum += image[idx]
                    g_sum += image[idx + 1]
                    b_sum += image[idx + 2]
                    count += 1
        
        if count > 0:
            return (r_sum // count, g_sum // count, b_sum // count)
        return (0, 0, 0)
    
    def is_target_color(self, detected_color):
        """Check if detected color matches current target color"""
        target_color = self.color_pattern[self.current_color_index]
        threshold = 50  # Color matching threshold
        
        return all(abs(c1 - c2) <= threshold for c1, c2 in zip(detected_color, target_color))
    
    def get_distance_readings(self):
        """Get normalized readings from distance sensors"""
        return [self.ps[i].getValue() for i in range(8)]
    
    def detect_walls(self):
        """Detect walls using distance sensors"""
        readings = self.get_distance_readings()
        threshold = 80
        
        # Front sensors (ps0, ps7)
        front_wall = max(readings[0], readings[7]) > threshold
        
        # Left sensors (ps5, ps6)
        left_wall = max(readings[5], readings[6]) > threshold
        
        # Right sensors (ps1, ps2)
        right_wall = max(readings[1], readings[2]) > threshold
        
        return front_wall, left_wall, right_wall
    
    def flood_fill(self, start, target):
        """Implementation of flood fill algorithm for maze solving"""
        queue = [(start, [start])]
        visited = {start}
        
        while queue:
            current, path = queue.pop(0)
            if current == target:
                return path
                
            # Get possible movements (up, right, down, left)
            x, y = current
            neighbors = [(x, y-1), (x+1, y), (x, y+1), (x-1, y)]
            
            for next_pos in neighbors:
                if (next_pos not in visited and 
                    next_pos not in self.walls and 
                    0 <= next_pos[0] < self.maze_size and 
                    0 <= next_pos[1] < self.maze_size):
                    visited.add(next_pos)
                    queue.append((next_pos, path + [next_pos]))
        return None
    
    def move_forward(self, speed=0.5):
        """Move robot forward"""
        self.left_motor.setVelocity(self.max_speed * speed)
        self.right_motor.setVelocity(self.max_speed * speed)
    
    def turn(self, direction):
        """Turn robot left or right"""
        if direction == 'left':
            self.left_motor.setVelocity(-self.max_speed * 0.5)
            self.right_motor.setVelocity(self.max_speed * 0.5)
        else:
            self.left_motor.setVelocity(self.max_speed * 0.5)
            self.right_motor.setVelocity(-self.max_speed * 0.5)
    
    def stop(self):
        """Stop robot movement"""
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
    
    def run(self):
        """Main control loop"""
        while self.robot.step(self.timestep) != -1:
            if self.current_color_index >= len(self.color_pattern):
                self.stop()
                break
                
            # Get sensor readings
            front_wall, left_wall, right_wall = self.detect_walls()
            current_color = self.get_color_from_camera()
            
            # State machine for navigation
            if self.state == 'SCAN':
                if self.is_target_color(current_color):
                    self.current_color_index += 1
                    self.state = 'SEARCH_NEXT'
                    self.stop()
                else:
                    if not front_wall:
                        self.move_forward()
                    elif not right_wall:
                        self.state = 'TURN_RIGHT'
                    elif not left_wall:
                        self.state = 'TURN_LEFT'
                    else:
                        self.state = 'TURN_AROUND'
                        
            elif self.state == 'TURN_RIGHT':
                self.turn('right')
                if not front_wall:
                    self.state = 'SCAN'
                    
            elif self.state == 'TURN_LEFT':
                self.turn('left')
                if not front_wall:
                    self.state = 'SCAN'
                    
            elif self.state == 'TURN_AROUND':
                self.turn('right')
                if front_wall:
                    self.state = 'SCAN'
                    
            elif self.state == 'SEARCH_NEXT':
                # Update position and continue searching
                self.visited.add(self.current_position)
                self.state = 'SCAN'

# Create and run the controller
controller = EPuckController()
controller.run()