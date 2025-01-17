from controller import Robot

def run_robot(robot):
    """ Wall-following robot with enhanced color detection and termination condition """
    
    # Get the time step of the current world
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28
    
    # Enable motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    # Enable proximity sensors
    prox_sensors = []
    for ind in range(8):
        sensor_name = 'ps' + str(ind)
        prox_sensors.append(robot.getDevice(sensor_name))
        prox_sensors[ind].enable(timestep)
    
    # Enable the camera
    camera = robot.getDevice("camera")
    camera.enable(timestep)
    
    # Color detection parameters
    class ColorThreshold:
        def __init__(self, r_min, r_max, g_min, g_max, b_min, b_max):
            self.r_min, self.r_max = r_min, r_max
            self.g_min, self.g_max = g_min, g_max
            self.b_min, self.b_max = b_min, b_max
    
    # Optimized color thresholds
    color_thresholds = {
        "Red": ColorThreshold(180, 255, 0, 100, 0, 100),
        "Yellow": ColorThreshold(180, 255, 180, 255, 0, 100),
        "Pink": ColorThreshold(180, 255, 0, 150, 150, 255),
        "Brown": ColorThreshold(100, 180, 0, 100, 0, 50),
        "Green": ColorThreshold(0, 100, 180, 255, 0, 100)
    }
    
    def detect_color():
        """Enhanced color detection with confidence scoring"""
        image = camera.getImage()
        if image is None:
            return "Unknown", 0
        
        width = camera.getWidth()
        height = camera.getHeight()
        total_pixels = width * height
        
        # Dictionary to store color counts
        color_counts = {color: 0 for color in color_thresholds.keys()}
        
        # Sample every other pixel to improve performance
        for y in range(0, height, 2):
            for x in range(0, width, 2):
                pixel_index = 4 * (y * width + x)
                red = image[pixel_index + 2]
                green = image[pixel_index + 1]
                blue = image[pixel_index + 0]
                
                # Check each color threshold
                for color, threshold in color_thresholds.items():
                    if (threshold.r_min <= red <= threshold.r_max and
                        threshold.g_min <= green <= threshold.g_max and
                        threshold.b_min <= blue <= threshold.b_max):
                        color_counts[color] += 1
        
        # Calculate confidence scores
        confidence_threshold = 0.01  # Lower threshold for better sensitivity
        max_confidence = 0
        detected_color = "Unknown"
        
        for color, count in color_counts.items():
            confidence = count / (total_pixels / 4)  # Adjust for sampling every other pixel
            if confidence > confidence_threshold and confidence > max_confidence:
                max_confidence = confidence
                detected_color = color
        
        return detected_color, max_confidence
    
    # Variables for termination sequence
    green_detected = False
    forward_steps = 0
    FORWARD_DISTANCE = 200  # Increased to move approximately 2-3 tiles forward
    
    # Main loop
    while robot.step(timestep) != -1:
        # Read the sensors
        left_distance = prox_sensors[5].getValue()
        right_distance = prox_sensors[2].getValue()
        left_wall = left_distance > 80
        right_wall = right_distance > 80
        left_corner = prox_sensors[6].getValue() > 80
        front_wall = prox_sensors[7].getValue() > 80
        
        # Default motor speeds
        left_speed = max_speed
        right_speed = max_speed
        
        # Logic for wall following and centering between walls
        if not green_detected:
            if front_wall:
                print("Turn right in place")
                left_speed = max_speed
                right_speed = -max_speed
            else:
                if left_wall and right_wall:
                    print("Centering between two walls")
                    balance_factor = (left_distance - right_distance) / 10
                    left_speed = max_speed - balance_factor
                    right_speed = max_speed + balance_factor
                elif left_wall:
                    print("Drive forward along left wall")
                    left_speed = max_speed
                    right_speed = max_speed
                elif right_wall:
                    print("Drive forward along right wall")
                    left_speed = max_speed
                    right_speed = max_speed / 2
                else:
                    print("Searching for wall")
                    left_speed = max_speed / 8
                    right_speed = max_speed
                if left_corner:
                    print("Came too close, turn right")
                    left_speed = max_speed
                    right_speed = max_speed / 8
        
        # Enhanced color detection
        detected_color, confidence = detect_color()
        
        if detected_color != "Unknown":
            print(f"Detected Color: {detected_color} (Confidence: {confidence:.2%})")
            
            if detected_color == "Green" and confidence > 0.02:  # Higher confidence threshold for termination
                if not green_detected:
                    print("Green color detected! Moving forward to final position...")
                    green_detected = True
                    forward_steps = 0
        
        # Termination sequence
        if green_detected:
            forward_steps += 1
            # Move straight forward at constant speed
            left_speed = max_speed
            right_speed = max_speed
            
            if forward_steps > FORWARD_DISTANCE:  # Extended forward movement
                # Stop motors
                left_motor.setVelocity(0.0)
                right_motor.setVelocity(0.0)
                print("Executed successfully!")
                # Take one more step to ensure motors stop
                robot.step(timestep)
                break
        
        # Set motor velocities
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)