""" ARAP Webots Standard Controller """
from controller import Robot, Motor, LED, DistanceSensor, Camera
import sys
import random

class ARAP:
    # Robot constants
    MOTORS_NUMBER = 2
    DISTANCE_SENSORS_NUMBER = 8
    GROUND_SENSORS_NUMBER = 3
    LEDS_NUMBER = 10
    LEFT = 0
    RIGHT = 1
    LED_ON = 255
    LED_OFF = 0
    MAX_SPEED = 6.28
    DELAY = 0.5
    MULTIPLIER = 1
    OBSTACLE_DISTANCE = 0.02

    motor_names = ("left wheel motor", "right wheel motor")
    distance_sensors_names = ("ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7")
    ground_sensors_names = ("gs0", "gs1", "gs2")
    leds_names = ("led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9")
    camera_names = "camera"

    weights = [ [-1.3, -1.0], 
                [-1.3, -1.0], 
                [-0.5, 0.5], 
                [0.0, 0.0], 
                [0.0, 0.0], 
                [0.05, -0.5], 
                [-0.75, 0.0], 
                [-0.75, 0.0] ]

    lookup_table = [ [0.0, 4095.0, 0.002], 
                 [0.005, 2133.33, 0.003], 
                 [0.01, 1465.73, 0.007], 
                 [0.015, 601.46, 0.0406], 
                 [0.02, 383.84, 0.01472], 
                 [0.03, 234.93, 0.0241], 
                 [0.04, 158.03, 0.0287], 
                 [0.05, 120.0, 0.04225], 
                 [0.06, 104.09, 0.03065], 
                 [0.07, 67.19, 0.04897] ]

    offsets = [MULTIPLIER * MAX_SPEED, MULTIPLIER * MAX_SPEED]
    
    def get_timestep(self):
        return int(self.robot.getBasicTimeStep())


    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())

        self.distance_sensors = []
        self.distance_sensors_values = []
        self.distance_range = 0.0

        self.ground_sensors = []
        self.ground_sensors_values = []

        self.leds = []
        self.leds_values = []

        self.speeds = [0.0, 0.0]
        self.camera = None
        self.left_motor = None
        self.right_motor = None

        self.counter = 0
        self.camera_interval = 0
        self.red = 0
        self.green = 0
        self.blue = 0

        self.init_devices()

    def init_devices(self):
        for i in range(self.DISTANCE_SENSORS_NUMBER):
            self.distance_sensors.append(self.robot.getDevice(self.distance_sensors_names[i]))
            self.distance_sensors_values.append(0.0)
            self.distance_sensors[i].enable(self.time_step)

        for i in range(self.GROUND_SENSORS_NUMBER):
            self.ground_sensors.append(self.robot.getDevice(self.ground_sensors_names[i]))
            self.ground_sensors_values.append(0.0)
            self.ground_sensors[i].enable(self.time_step)

        for i in range(self.LEDS_NUMBER):
            self.leds.append(self.robot.getDevice(self.leds_names[i]))
            self.leds_values.append(self.LED_OFF)
            self.leds[i].set(self.LED_OFF)

        self.camera = self.robot.getDevice(self.camera_names)
        self.camera.enable(self.time_step)

        self.left_motor = self.robot.getDevice(self.motor_names[self.LEFT])
        self.right_motor = self.robot.getDevice(self.motor_names[self.RIGHT])
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.step()

    def wait(self, sec):
        start_time = self.robot.getTime()
        while self.robot.getTime() < start_time + sec:
            self.step()
        return True

    def reset_actuator_values(self):
        self.speeds = [0.0, 0.0]
        self.leds_values = [self.LED_OFF] * self.LEDS_NUMBER
        self.distance_sensors_values = [0.0] * self.DISTANCE_SENSORS_NUMBER
        self.ground_sensors_values = [0.0] * self.GROUND_SENSORS_NUMBER

    def set_actuators(self):
        for i in range(self.LEDS_NUMBER):
            self.leds[i].set(self.leds_values[i])
        self.left_motor.setVelocity(self.speeds[self.LEFT])
        self.right_motor.setVelocity(self.speeds[self.RIGHT])

    def blink_leds(self):
        brightness = int(((self.counter / 10) % self.LEDS_NUMBER) * 255)
        if brightness > self.LED_ON:
            self.counter = 0
        self.leds_values = [brightness] * self.LEDS_NUMBER
        self.counter += 1

    def get_sensor_input(self):
        for i in range(self.DISTANCE_SENSORS_NUMBER):
            self.distance_sensors_values[i] = self.distance_sensors[i].getValue()

        for i in range(self.GROUND_SENSORS_NUMBER):
            self.ground_sensors_values[i] = self.ground_sensors[i].getValue()

        sensor_total = self.distance_sensors_values[0] + self.distance_sensors_values[7]
        for j in range(len(self.lookup_table)):
            if sensor_total >= self.lookup_table[j][1]:
                self.distance_range = self.lookup_table[j][0]
                break

        for i in range(self.DISTANCE_SENSORS_NUMBER):
            self.distance_sensors_values[i] = min(self.distance_sensors_values[i] / 4096.0, 1.0)

        return self.distance_range

    def get_time_step(self):
        if self.time_step == -1:
            self.time_step = int(self.robot.getBasicTimeStep())
        return self.time_step

    def step(self):
        if self.robot.step(self.get_time_step()) == -1:
            sys.exit(0)

    def get_camera_image(self, interval):
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        image = self.camera.getImage()

        if self.camera_interval >= interval:
            self.red = self.green = self.blue = 0
            for x in range(width):
                for y in range(height):
                    self.red += self.camera.imageGetRed(image, width, x, y)
                    self.green += self.camera.imageGetGreen(image, width, x, y)
                    self.blue += self.camera.imageGetBlue(image, width, x, y)
            total_pixels = width * height
            self.red //= total_pixels
            self.green //= total_pixels
            self.blue //= total_pixels
            self.camera_interval = 0
        else:
            self.red = self.green = self.blue = 0
            self.camera_interval += 1

        return self.red, self.green, self.blue

    def ground_obstacles_detected(self):
        for value in self.ground_sensors_values:
            if value < 500.0:
                return True
        return False

    def front_obstacles_detected(self):
        average = (self.distance_sensors_values[0] + self.distance_sensors_values[7]) / 2.0
        return average > self.OBSTACLE_DISTANCE

    def back_obstacles_detected(self):
        average = (self.distance_sensors_values[3] + self.distance_sensors_values[4]) / 2.0
        return average > self.OBSTACLE_DISTANCE

    def left_obstacles_detected(self):
        average = (self.distance_sensors_values[5] + self.distance_sensors_values[6]) / 2.0
        return average > self.OBSTACLE_DISTANCE

    def right_obstacles_detected(self):
        average = (self.distance_sensors_values[1] + self.distance_sensors_values[2]) / 2.0
        return average > self.OBSTACLE_DISTANCE

    def run_braitenberg(self):
        for i in range(2):
            self.speeds[i] = self.offsets[i]
            for j in range(self.DISTANCE_SENSORS_NUMBER):
                self.speeds[i] += self.distance_sensors_values[j] * self.weights[j][i] * self.MAX_SPEED
            self.speeds[i] = max(-self.MAX_SPEED, min(self.speeds[i], self.MAX_SPEED))

    def move(self, left_multiplier, right_multiplier):
        self.left_motor.setVelocity(left_multiplier * self.MAX_SPEED)
        self.right_motor.setVelocity(right_multiplier * self.MAX_SPEED)
        self.wait(self.DELAY)

    def move_forward(self):
        self.move(self.MULTIPLIER, self.MULTIPLIER)

    def move_backward(self):
        self.move(-self.MULTIPLIER, -self.MULTIPLIER)

    def turn_left(self):
        self.move(-self.MULTIPLIER, self.MULTIPLIER)

    def turn_right(self):
        self.move(self.MULTIPLIER, -self.MULTIPLIER)
        
    def stop(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    
    def is_over_blue_patch(self, threshold_low=500, threshold_high=550):
        # Read current ground sensor values
        self.get_sensor_input()  # update ground_sensors_values
             
        # Check if all ground sensors detect values within blue patch range
        for value in self.ground_sensors_values:
            if value < threshold_low or value > threshold_high:
                return False
        return True
        
    def is_over_green_patch(self, threshold_low=300, threshold_high=450):
        # Read current ground sensor values
        self.get_sensor_input()  # update ground_sensors_values
             
        # Check if all ground sensors detect values within green patch range
        for value in self.ground_sensors_values:
            if value < threshold_low or value > threshold_high:
                return False
        return True  


        
    def turn_left_by_angle(self, angle_degrees):
        turn_duration_for_360 = 3.0  # seconds to complete a full turn (you may need to tune this)
        duration = (angle_degrees / 360.0) * turn_duration_for_360
    
        self.left_motor.setVelocity(-self.MULTIPLIER * self.MAX_SPEED)
        self.right_motor.setVelocity(self.MULTIPLIER * self.MAX_SPEED)
        
        start_time = self.robot.getTime()
        while self.robot.getTime() - start_time < duration:
            self.robot.step(int(self.robot.getBasicTimeStep()))
     
    def avoid_obstacles(self):
        #print("avoid obstacle behavior started")
        if self.front_obstacles_detected():
            self.move_backward()
            if random.randint(1, 2) == 1:
                self.turn_left()
            else:
                self.turn_right()
            return True
        return False     

        
    def consume_resource(self, resource_name, value, max_value=900, rate_per_sec=100):
        """
        Generic resource replenishment function.
        
        Parameters:
        - resource_name: 'thirst', 'hunger', etc. (for logging)
        - value: current value of the resource
        - max_value: max cap of the resource
        - rate_per_sec: rate at which the resource replenishes per second
    
        Returns:
        - updated resource value (clamped to max)
        """
        print(f"{resource_name.capitalize()} replenishing...")
    
        while value < max_value:
            value += rate_per_sec * (self.get_timestep() / 1000.0)
            self.stop()
            self.step()
            print(f"{resource_name.capitalize()}... Current {resource_name}: {int(value)}")
    
        value = max_value  # Clamp
        print(f"{resource_name.capitalize()} replenished.")
        return value
    
        
    def seek_resource(self, target_color, color_threshold=50, rotate_duration=2.7):
        """
        Rotates in place to look for the target color using the camera.
        If detected, returns True (resource found).
        If not, performs a short wander and returns False.
    
        Parameters:
        - target_color: 'blue', 'green', or 'red'
        - color_threshold: minimum intensity to consider detection
        - rotate_duration: how long to rotate (in seconds)
        """
    
        print(f"Seeking {target_color}...")
    
        rotation_speed = 0.5 * self.MAX_SPEED
        self.speeds[self.LEFT] = -rotation_speed
        self.speeds[self.RIGHT] = rotation_speed
    
        start_time = self.robot.getTime()
    
        while self.robot.getTime() - start_time < rotate_duration:
            self.set_actuators()
            self.step()
    
            red, green, blue = self.get_camera_image(5)
            print(f"Camera RGB: R={red}, G={green}, B={blue}")
    
            if target_color == 'blue' and blue > color_threshold and blue > green and blue > red:
                print("Blue detected! Locking on.")
                return True
            elif target_color == 'green' and green > color_threshold and green > red and green > blue:
                print("Green detected! Locking on.")
                return True
            elif target_color == 'red' and red > color_threshold and red > green and red > blue:
                print("Red detected! Locking on.")
                return True
    
        print(f"No {target_color} detected. Wandering forward before next rotation...")
    
        # Wander forward briefly (use random values or previous ones)
        self.speeds[self.LEFT] = 0.5 * self.MAX_SPEED
        self.speeds[self.RIGHT] = 0.5 * self.MAX_SPEED
    
        wander_start = self.robot.getTime()
        while self.robot.getTime() - wander_start < 3.0:
            self.set_actuators()
            self.step()
    
        return False

        
    def wander(self, last_random_time, left_speed, right_speed):
        current_time = self.robot.getTime()
        #print("Wandering behavior started")
        
        gs_values = [sensor.getValue() for sensor in self.ground_sensors]
        avg = sum(gs_values) / len(gs_values)
        #print(avg)
    
        # Use existing avoid_obstacles method
        if self.avoid_obstacles():
            self.set_actuators()
            self.step()
            return current_time, left_speed, right_speed  # Prevent instant speed change after obstacle
    
        # Random wandering logic
        if current_time - last_random_time > 2.0:
            base_speed = random.uniform(0.3 * self.MAX_SPEED, self.MAX_SPEED)
            speed_diff = random.uniform(-0.3, 0.3) * self.MAX_SPEED
            left_speed = base_speed + speed_diff
            right_speed = base_speed - speed_diff
            last_random_time = current_time
    
        self.speeds[self.LEFT] = max(-self.MAX_SPEED, min(left_speed, self.MAX_SPEED))
        self.speeds[self.RIGHT] = max(-self.MAX_SPEED, min(right_speed, self.MAX_SPEED))
    
        return last_random_time, left_speed, right_speed
        
    def lock_on_and_approach(self, target_color='blue', timeout=5.0):
        """
        Moves forward while avoiding obstacles and checking for the target ground patch.
        Returns True if the robot reaches the correct color patch, False if timed out.
    
        Parameters:
        - target_color: 'blue', 'green', or 'red'
        - timeout: time limit to reach the patch
        """
        start_time = self.robot.getTime()
    
        while self.robot.getTime() - start_time < timeout:
            self.reset_actuator_values()
            self.get_sensor_input()
    
            if self.avoid_obstacles():
                pass
            else:
                self.speeds[self.LEFT] = 0.8 * self.MAX_SPEED
                self.speeds[self.RIGHT] = 0.8 * self.MAX_SPEED
    
            self.set_actuators()
            self.step()
    
            if self.is_over_color_patch(target_color):
                print(f"{target_color.capitalize()} patch reached!")
                return True
    
        print(f"Timed out. Couldn't reach the {target_color} patch.")
        return False

        
   #generalisation done.----------------------------------
    
    def handle_resource(self, resource_name, resource_value, last_random_time, left_speed, right_speed, resource_timer, 
                    color, threshold=500, max_value=900, timeout=5.0):
        """
        Generalized handler for any resource-seeking behavior (thirst, hunger, etc.)
    
        Parameters:
        - resource_name: string (e.g., "thirst", "hunger")
        - resource_value: current resource value (e.g., thirst or hunger)
        - last_random_time, left_speed, right_speed: wandering params
        - resource_timer: time tracker for resource
        - color: 'blue', 'green', 'red', etc.
        - threshold: trigger point to seek resource
        - max_value: maximum value the resource can reach
        - timeout: max time allowed for approach phase
    
        Returns:
        - updated_resource_value, last_random_time, left_speed, right_speed, resource_timer, True/False (handled or not)
        """
        if resource_value < threshold:
            print(f"{resource_name.capitalize()} low ({int(resource_value)}). Seeking {color} resource...")
            found = self.seek_resource(color)
    
            if found:
                reached = self.lock_on_and_approach(target_color=color, timeout=timeout)
                if reached:
                    resource_value = self.consume_resource(resource_name, resource_value, max_value)
                    resource_timer = 0.0
                    self.last_time = self.robot.getTime()
                    resource_value = min(resource_value, max_value)
                else:
                    print(f"Failed to reach {color} patch. Re-attempting search...")
    
            return resource_value, last_random_time, left_speed, right_speed, resource_timer, True
    
        return resource_value, last_random_time, left_speed, right_speed, resource_timer, False
        
        
    def get_ground_color(self):
        """
        Returns an estimated RGB color tuple (red, green, blue)
        based on the ground sensor values.
        """
        gs_values = [sensor.getValue() for sensor in self.ground_sensors]
        avg = sum(gs_values) / len(gs_values)
        print(avg)
    
        # Basic threshold logic - you'll want to calibrate this with real sensor data
        if avg < 300:
            return (0, 0, 0)  # Black
        elif 370 <= avg < 400:
            return (0, 0, 255)  # Blue
        elif 420 <= avg < 440:
            return (0, 255, 0)  # Green
        else:
            return (0, 0, 0)    # unknown / black
    
    def is_over_color_patch(self, color):
        red, green, blue = self.get_ground_color()
    
        if color == 'blue':
            return blue > 120 and blue > red and blue > green
        elif color == 'green':
            return green > 120 and green > red and green > blue
        elif color == 'red':
            return red > 120 and red > green and red > blue
        else:
            return False
               
                    
            
                
