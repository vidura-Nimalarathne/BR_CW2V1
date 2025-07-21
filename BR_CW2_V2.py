import robot
import random

# Highest-priority behavior: search for coloured boxes
def search_coloured_boxes(robot1, red_box_found, green_box_found, blue_box_found, search_log):
    red, green, blue = robot1.get_camera_image(5)

    if red > 70 and not red_box_found and green < 40 and blue < 40:
        search_log.append("Red box found")
        search_log.reverse()
        print(search_log)
        red_box_found = True

    elif green > 70 and not green_box_found and red < 40 and blue < 40:
        search_log.append("Green box found")
        search_log.reverse()
        print(search_log)
        green_box_found = True

    elif blue > 70 and not blue_box_found and red < 40 and green < 40:
        search_log.append("Blue box found")
        search_log.reverse()
        print(search_log)
        blue_box_found = True

    return red_box_found, green_box_found, blue_box_found

# Medium-priority behavior: obstacle avoidance
def avoid_obstacles(robot1):
    if robot1.front_obstacles_detected():
        robot1.move_backward()
        robot1.turn_left()
        return True
    return False

# Lowest-priority behavior: wandering
def wander(robot1, last_random_time, left_speed, right_speed):
    current_time = robot1.robot.getTime()
    if current_time - last_random_time > 2.0:
        base_speed = random.uniform(0.3 * robot1.MAX_SPEED, robot1.MAX_SPEED)
        speed_diff = random.uniform(-0.3, 0.3) * robot1.MAX_SPEED
        left_speed = base_speed + speed_diff
        right_speed = base_speed - speed_diff
        last_random_time = current_time

    robot1.speeds[robot1.LEFT] = max(-robot1.MAX_SPEED, min(left_speed, robot1.MAX_SPEED))
    robot1.speeds[robot1.RIGHT] = max(-robot1.MAX_SPEED, min(right_speed, robot1.MAX_SPEED))
    return last_random_time, left_speed, right_speed

def main():
    robot1 = robot.ARAP()
    robot1.init_devices()

    red_box_found = False
    green_box_found = False
    blue_box_found = False
    search_log = []

    # Random wandering parameters
    base_speed = random.uniform(0.5 * robot1.MAX_SPEED, robot1.MAX_SPEED)
    speed_diff = random.uniform(-0.5, 0.5) * robot1.MAX_SPEED
    left_speed = base_speed + speed_diff
    right_speed = base_speed - speed_diff
    last_random_time = robot1.robot.getTime()

    while True:
        robot1.reset_actuator_values()
        robot1.get_sensor_input()
        robot1.blink_leds()

        # Highest priority: coloured box detection
        red_box_found, green_box_found, blue_box_found = search_coloured_boxes(
            robot1, red_box_found, green_box_found, blue_box_found, search_log
        )

        # If all boxes found, print and reset
        if red_box_found and green_box_found and blue_box_found:
            print("Woohooo!!!! All boxes found!")
            red_box_found = False
            green_box_found = False
            blue_box_found = False
            search_log.clear()

        # Medium priority: obstacle avoidance
        if not (red_box_found or green_box_found or blue_box_found):
            if avoid_obstacles(robot1):
                robot1.set_actuators()
                robot1.step()
                continue  # Skip wandering if obstacle handled

        # Lowest priority: wandering
        last_random_time, left_speed, right_speed = wander(
            robot1, last_random_time, left_speed, right_speed
        )

        robot1.set_actuators()
        robot1.step()

if __name__ == "__main__":
    main()
