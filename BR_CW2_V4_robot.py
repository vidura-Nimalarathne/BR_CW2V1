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

        
    def drink_water(robot1, thirst):
        print("Drinking...")
        while thirst < 900:
            thirst += 100 * (robot1.get_timestep() / 1000.0)
            robot1.stop()
            robot1.step()
            print(f"Drinking... Current thirst: {int(thirst)}")
        thirst = 900  # clamp to max after loop
        print("Done drinking. Thirst replenished.")
        return thirst
        
    def seek_water(self):
        #print("Water seeking behavior started")

        rotation_speed = 0.5 * self.MAX_SPEED
        self.speeds[self.LEFT] = -rotation_speed
        self.speeds[self.RIGHT] = rotation_speed

        start_time = self.robot.getTime()
        rotate_duration = 2.7

        while self.robot.getTime() - start_time < rotate_duration:
            self.set_actuators()
            self.step()

            red, green, blue = self.get_camera_image(5)
            print(f"Camera RGB: R={red}, G={green}, B={blue}")
            if blue > 110 and blue > green and blue > red:
                print("Blue detected! Locking on.")
                return True

        print("No blue detected. Wandering forward before next rotation...")

        # Wander forward for 3 seconds
        last_random_time, left_speed, right_speed = self.wander(
            last_random_time, left_speed, right_speed
        )
    
        robot1.set_actuators()
        robot1.step()
        
    def wander(self, last_random_time, left_speed, right_speed):
        current_time = self.robot.getTime()
        #print("Wandering behavior started")
    
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
        
    def lock_on_and_approach(self, timeout=5.0):
        start_time = self.robot.getTime()
        #print("Lock on behavior started")

        while self.robot.getTime() - start_time < timeout:
            self.reset_actuator_values()
            self.get_sensor_input()

            if self.avoid_obstacles():
                pass  # obstacle avoidance handled internally
            else:
                self.speeds[self.LEFT] = 0.8 * self.MAX_SPEED
                self.speeds[self.RIGHT] = 0.8 * self.MAX_SPEED

            self.set_actuators()
            self.step()

            if self.is_over_blue_patch():
                print("Water patch reached!")
                return True  # success

        print("Timed out. Couldn't reach the water patch.")
        return False  # timeout
        
    def handle_thirst(self, thirst, last_random_time, left_speed, right_speed, thirst_timer):
        if thirst < 500:
            print(f"Thirsty ({int(thirst)}). Looking for water...")
            found = self.seek_water()
            if found:
                reached = self.lock_on_and_approach(timeout=5.0)
                if reached:
                    thirst = self.drink_water(thirst)
                    thirst_timer = 0.0
                    self.last_time = self.robot.getTime()  # Make sure self.last_time exists
                    thirst = min(thirst, 900)
                else:
                    print("Failed to reach water. Re-attempting search...")
            else:
                # Wander briefly and retry
                last_random_time, left_speed, right_speed = self.wander(
                    last_random_time, left_speed, right_speed
                )
                self.set_actuators()
                self.step()
    
            return thirst, last_random_time, left_speed, right_speed, thirst_timer, True
    
        return thirst, last_random_time, left_speed, right_speed, thirst_timer, False
    
