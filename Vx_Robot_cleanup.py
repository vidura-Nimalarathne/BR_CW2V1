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
    leds_names = tuple(f"led{i}" for i in range(LEDS_NUMBER))
    camera_name = "camera"

    weights = [
        [-1.3, -1.0], 
        [-1.3, -1.0], 
        [-0.5, 0.5], 
        [0.0, 0.0], 
        [0.0, 0.0], 
        [0.05, -0.5], 
        [-0.75, 0.0], 
        [-0.75, 0.0]
    ]

    lookup_table = [
        [0.0, 4095.0, 0.002], 
        [0.005, 2133.33, 0.003], 
        [0.01, 1465.73, 0.007], 
        [0.015, 601.46, 0.0406], 
        [0.02, 383.84, 0.01472], 
        [0.03, 234.93, 0.0241], 
        [0.04, 158.03, 0.0287], 
        [0.05, 120.0, 0.04225], 
        [0.06, 104.09, 0.03065], 
        [0.07, 67.19, 0.04897]
    ]

    offsets = [MULTIPLIER * MAX_SPEED, MULTIPLIER * MAX_SPEED]

    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())

        self.distance_sensors = []
        self.distance_sensors_values = [0.0] * self.DISTANCE_SENSORS_NUMBER

        self.ground_sensors = []
        self.ground_sensors_values = [0.0] * self.GROUND_SENSORS_NUMBER

        self.leds = []
        self.leds_values = [self.LED_OFF] * self.LEDS_NUMBER

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

    def get_timestep(self):
        return self.time_step

    def init_devices(self):
        # Initialize distance sensors
        for name in self.distance_sensors_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.time_step)
            self.distance_sensors.append(sensor)

        # Initialize ground sensors
        for name in self.ground_sensors_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.time_step)
            self.ground_sensors.append(sensor)

        # Initialize LEDs
        for name in self.leds_names:
            led = self.robot.getDevice(name)
            led.set(self.LED_OFF)
            self.leds.append(led)

        # Initialize camera
        self.camera = self.robot.getDevice(self.camera_name)
        self.camera.enable(self.time_step)

        # Initialize motors
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
        for i, led in enumerate(self.leds):
            led.set(self.leds_values[i])
        self.left_motor.setVelocity(self.speeds[self.LEFT])
        self.right_motor.setVelocity(self.speeds[self.RIGHT])

    def blink_leds(self):
        brightness = int(((self.counter / 10) % self.LEDS_NUMBER) * 255)
        if brightness > self.LED_ON:
            self.counter = 0
        self.leds_values = [brightness] * self.LEDS_NUMBER
        self.counter += 1

    def get_sensor_input(self):
        # Read distance sensors
        for i, sensor in enumerate(self.distance_sensors):
            self.distance_sensors_values[i] = sensor.getValue()

        # Read ground sensors
        for i, sensor in enumerate(self.ground_sensors):
            self.ground_sensors_values[i] = sensor.getValue()

        # Compute distance range using lookup table on front sensors
        sensor_total = self.distance_sensors_values[0] + self.distance_sensors_values[7]
        for val, threshold, _ in self.lookup_table:
            if sensor_total >= threshold:
                self.distance_range = val
                break

        # Normalize distance sensor values
        self.distance_sensors_values = [min(val / 4096.0, 1.0) for val in self.distance_sensors_values]

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
            self.red = 0
            self.green = 0
            self.blue = 0
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
        return any(value < 500.0 for value in self.ground_sensors_values)

    def front_obstacles_detected(self):
        avg = (self.distance_sensors_values[0] + self.distance_sensors_values[7]) / 2.0
        return avg > self.OBSTACLE_DISTANCE

    def back_obstacles_detected(self):
        avg = (self.distance_sensors_values[3] + self.distance_sensors_values[4]) / 2.0
        return avg > self.OBSTACLE_DISTANCE

    def left_obstacles_detected(self):
        avg = (self.distance_sensors_values[5] + self.distance_sensors_values[6]) / 2.0
        return avg > self.OBSTACLE_DISTANCE

    def right_obstacles_detected(self):
        avg = (self.distance_sensors_values[1] + self.distance_sensors_values[2]) / 2.0
        return avg > self.OBSTACLE_DISTANCE

    def run_braitenberg(self):
        for i in range(2):
            speed = self.offsets[i]
            for j in range(self.DISTANCE_SENSORS_NUMBER):
                speed += self.distance_sensors_values[j] * self.weights[j][i] * self.MAX_SPEED
            self.speeds[i] = max(-self.MAX_SPEED, min(speed, self.MAX_SPEED))

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

    def is_over_patch(self, threshold_low, threshold_high):
        self.get_sensor_input()
        return all(threshold_low <= val <= threshold_high for val in self.ground_sensors_values)

    def is_over_blue_patch(self):
        return self.is_over_patch(500, 550)

    def is_over_green_patch(self):
        return self.is_over_patch(300, 450)

    def is_over_red_patch(self):
        return self.is_over_patch(670, 700)

    def turn_left_by_angle(self, angle_degrees):
        turn_duration_360 = 3.0  # Tune as needed
        duration = (angle_degrees / 360.0) * turn_duration_360

        self.left_motor.setVelocity(-self.MULTIPLIER * self.MAX_SPEED)
        self.right_motor.setVelocity(self.MULTIPLIER * self.MAX_SPEED)

        start_time = self.robot.getTime()
        while self.robot.getTime() - start_time < duration:
            self.step()

    def avoid_obstacles(self):
        if self.front_obstacles_detected():
            self.move_backward()
            if random.choice([True, False]):
                self.turn_left()
            else:
                self.turn_right()
            return True
        return False

    def consume_resource(self, resource_name, value, max_value=900, rate_per_sec=100):
        print(f"{resource_name.capitalize()} replenishing...")
        while value < max_value:
            value += rate_per_sec * (self.get_timestep() / 1000.0)
            self.stop()
            self.step()
            print(f"{resource_name.capitalize()}... Current {resource_name}: {int(value)}")
        print(f"{resource_name.capitalize()} replenished.")
        return max_value

    def seek_resource(self, target_color, color_threshold=50, rotate_duration=2.7):
        print(f"Seeking {target_color}...")

        rotation_speed = 0.5 * self.MAX_SPEED
        self.speeds[self.LEFT] = -rotation_speed
        self.speeds[self.RIGHT] = rotation_speed

        start_time = self.robot.getTime()
        while self.robot.getTime() - start_time < rotate_duration:
            self.set_actuators()
            self.step()

            red, green, blue = self.get_camera_image(5)

            if target_color == 'blue' and blue > color_threshold > green and blue > red:
                print("Blue detected! Locking on.")
                return True
            elif target_color == 'green' and green > color_threshold > red and green > blue:
                print("Green detected! Locking on.")
                return True
            elif target_color == 'red' and red > color_threshold > green and red > blue:
                print("Red detected! Locking on.")
                return True

        print(f"No {target_color} detected. Wandering forward before next rotation...")
        self.speeds = [0.5 * self.MAX_SPEED] * 2

        wander_start = self.robot.getTime()
        while self.robot.getTime() - wander_start < 3.0:
            self.set_actuators()
            self.step()

        return False
def get_ground_sensor_avg(self):
    """Helper method to get average ground sensor value."""
    return sum(sensor.getValue() for sensor in self.ground_sensors) / len(self.ground_sensors)

def wander(self, last_random_time, left_speed, right_speed):
    current_time = self.robot.getTime()
    
    avg = self.get_ground_sensor_avg()
    #print(avg)

    if self.avoid_obstacles():
        self.set_actuators()
        self.step()
        return current_time, left_speed, right_speed  # Prevent instant speed change after obstacle

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

def handle_resource(self, resource_name, resource_value, last_random_time, left_speed, right_speed, resource_timer, 
                    color, threshold=500, max_value=900, timeout=5.0):
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
    avg = self.get_ground_sensor_avg()
    #print(avg)

    if avg < 300:
        return (0, 0, 0)  # Black
    elif 370 <= avg < 400:
        return (0, 0, 255)  # Blue
    elif 420 <= avg < 440:
        return (0, 255, 0)  # Green
    elif 670 <= avg < 700:
        return (255, 0, 0)  # Red
    else:
        return (0, 0, 0)

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

def sleep(self, sleep_level, thirst, hunger, health, 
          sleep_threshold=700, max_sleep=900, rate_per_sec=100, 
          health_penalty_rate=2, duration=5):
    if thirst > sleep_threshold and hunger > sleep_threshold:
        print("Conditions met. Sleeping...")
        self.stop()
        start_time = self.robot.getTime()
        while self.robot.getTime() - start_time < duration:
            self.blink_leds()  # simulate "lights off"
            self.set_actuators()
            self.step()

            sleep_level += rate_per_sec * (self.get_timestep() / 1000.0)
            if sleep_level > max_sleep:
                sleep_level = max_sleep
                break

            print(f"Sleeping... Current sleep level: {int(sleep_level)}")

        print("Finished sleeping.")
    else:
        print("Cannot sleep now. Hunger or thirst too low.")
        if sleep_level <= 0:
            print("Sleep depleted. Losing health...")
            health -= health_penalty_rate * (self.get_timestep() / 1000.0)
            health = max(0, health)

    return sleep_level, health

def search_coloured_boxes(self, red_box_found, green_box_found, blue_box_found, search_log):
    red, green, blue = self.get_camera_image(5)

    for color, val, found_flag in [('red', red, red_box_found),
                                  ('green', green, green_box_found),
                                  ('blue', blue, blue_box_found)]:
        if val > 70 and not found_flag:
            others = {'red': (green, blue), 'green': (red, blue), 'blue': (red, green)}
            if all(other_val < 40 for other_val in others[color]):
                search_log.append(f"{color.capitalize()} box found")
                search_log.reverse()
                print(search_log)
                if color == 'red':
                    red_box_found = True
                elif color == 'green':
                    green_box_found = True
                elif color == 'blue':
                    blue_box_found = True

    return red_box_found, green_box_found, blue_box_found

def move_for_duration(self, left_speed, right_speed, duration):
    """Helper to set wheel speeds and move for a given duration."""
    self.left_motor.setVelocity(left_speed)
    self.right_motor.setVelocity(right_speed)
    end_time = self.robot.getTime() + duration
    while self.robot.getTime() < end_time:
        self.step()

def bail_out_from_radiation(self):
    self.get_sensor_input()
    print("[Bailout] Starting bailout, GS values:", self.ground_sensors_values)

    self.move_for_duration(-0.5 * self.MAX_SPEED, -0.5 * self.MAX_SPEED, 0.1)  # short reverse step

    timeout = self.robot.getTime() + 3.0
    red_detected = self.is_over_red_patch()
    while red_detected and self.robot.getTime() < timeout:
        self.move_for_duration(-0.5 * self.MAX_SPEED, -0.5 * self.MAX_SPEED, 0.1)
        self.get_sensor_input()
        red_detected = self.is_over_red_patch()

    direction = random.choice(["left", "right"])
    print(f"[Bailout] Escaped red zone. Turning {direction}.")
    if direction == "left":
        self.move_for_duration(-0.5 * self.MAX_SPEED, 0.5 * self.MAX_SPEED, 0.5)
    else:
        self.move_for_duration(0.5 * self.MAX_SPEED, -0.5 * self.MAX_SPEED, 0.5)

    print("[Bailout] Moving forward after turn.")
    self.move_for_duration(0.5 * self.MAX_SPEED, 0.5 * self.MAX_SPEED, 1.0)

def handle_radiation_zone(self, current_time, last_time, health,
                          radiation_exposure, post_radiation_timer,
                          inside_radiation, radiation_start_time):
    self.get_sensor_input()
    red_detected = self.is_over_red_patch()

    if red_detected:
        if not inside_radiation:
            print("[Radiation] Entered red patch.")
            radiation_start_time = current_time
            inside_radiation = True
            health -= 10  # Immediate damage on entry
            radiation_exposure = True
            print(f"[Radiation] Damage taken! Health now: {health}")
        else:
            elapsed_inside = current_time - radiation_start_time
            if elapsed_inside >= 1.0:
                health -= 10
                print(f"[Radiation] Damage taken! Health now: {health}")
                radiation_start_time = current_time

        self.bail_out_from_radiation()
        return True, health, radiation_exposure, post_radiation_timer, inside_radiation, radiation_start_time

    else:
        if inside_radiation:
            print("[Radiation] Exited red patch.")
            inside_radiation = False
            post_radiation_timer = current_time + 2.0  # cool-off

        if radiation_exposure and current_time < post_radiation_timer:
            self.step()
            return True, health, radiation_exposure, post_radiation_timer, inside_radiation, radiation_start_time

    return False, health, radiation_exposure, post_radiation_timer, inside_radiation, radiation_start_time
