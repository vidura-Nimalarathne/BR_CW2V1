import robot
import random

def main():
    red = 0
    green = 0
    blue = 0

    red_box_found = False
    blue_box_found = False
    green_box_found = False
    search_log = []

    sensor_range = 0.0
    robot1 = robot.ARAP()
    robot1.init_devices()

    # Initialize random movement variables
    base_speed = random.uniform(0.5 * robot1.MAX_SPEED, robot1.MAX_SPEED)
    speed_diff = random.uniform(-0.5, 0.5) * robot1.MAX_SPEED
    left_speed = base_speed + speed_diff
    right_speed = base_speed - speed_diff
    last_random_time = robot1.robot.getTime()

    while (True):
        robot1.reset_actuator_values()
        sensor_range = robot1.get_sensor_input()
        robot1.blink_leds()
        red, green, blue = robot1.get_camera_image(5)
        colours_detected = [red, green, blue]

        if red > 70 and not red_box_found and green < 40 and blue < 40:
            search_log.append("Red box found")
            search_log.reverse()
            print(search_log)
            red_box_found = True

        elif blue > 70 and not blue_box_found and red < 40 and green < 40:
            search_log.append("Blue box found")
            search_log.reverse()
            print(search_log)
            blue_box_found = True

        elif green > 70 and not green_box_found and blue < 40 and red < 40:
            search_log.append("Green box found")
            search_log.reverse()
            print(search_log)
            green_box_found = True

        if robot1.front_obstacles_detected(): 
            robot1.move_backward()
            robot1.turn_left()

        elif robot1.front_obstacles_detected():
            robot1.move_backward()
            robot1.turn_right()

        else:
            # Update random movement pattern every 2 seconds
            current_time = robot1.robot.getTime()
            if current_time - last_random_time > 2.0:
                base_speed = random.uniform(0.2 * robot1.MAX_SPEED, robot1.MAX_SPEED)
                speed_diff = random.uniform(-0.2, 0.2) * robot1.MAX_SPEED
                left_speed = base_speed + speed_diff
                right_speed = base_speed - speed_diff
                last_random_time = current_time

            # Always apply last known speeds
            robot1.speeds[robot1.LEFT] = max(-robot1.MAX_SPEED, min(left_speed, robot1.MAX_SPEED))
            robot1.speeds[robot1.RIGHT] = max(-robot1.MAX_SPEED, min(right_speed, robot1.MAX_SPEED))

        robot1.set_actuators()
        robot1.step()

    print("All objects have been found! Exiting search mode...")    
    robot1.stop()

if __name__ == "__main__":
    main()
