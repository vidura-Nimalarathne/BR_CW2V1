import robot
import random
import math

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

def main():
    robot1 = robot.ARAP()
    robot1.init_devices()

    red_box_found = False
    green_box_found = False
    blue_box_found = False
    search_log = []

    # Resource stats
    thirst = 600
    hunger = 600
    fatigue = 600
    health = 600

    last_time = robot1.robot.getTime()
    last_display_time = last_time
    thirst_timer = 0.0
    hunger_timer = 0.0

    # Wandering parameters
    base_speed = random.uniform(0.5 * robot1.MAX_SPEED, robot1.MAX_SPEED)
    speed_diff = random.uniform(-0.5, 0.5) * robot1.MAX_SPEED
    left_speed = base_speed + speed_diff
    right_speed = base_speed - speed_diff
    last_random_time = robot1.robot.getTime()

    while True:
        robot1.reset_actuator_values()
        robot1.get_sensor_input()
        robot1.blink_leds()

        # Update resource decay
        current_time = robot1.robot.getTime()
        elapsed = current_time - last_time
        thirst_timer += elapsed
        hunger_timer += elapsed

        if thirst_timer >= 1.0:
            thirst -= 5 * int(thirst_timer)
            thirst_timer -= int(thirst_timer)
            thirst = max(0, thirst)

        if hunger_timer >= 1.0:
            hunger -= 2 * int(hunger_timer)
            hunger_timer -= int(hunger_timer)
            hunger = max(0, hunger)

        last_time = current_time

        # Display status every 5 seconds
        if current_time - last_display_time >= 5.0:
            print(f"Thirst Level: {thirst} | Hunger Level: {hunger}")
            last_display_time = current_time

        # Handle thirst
        thirst, last_random_time, left_speed, right_speed, thirst_timer, thirsty = robot1.handle_resource(
            resource_name="thirst",
            resource_value=thirst,
            last_random_time=last_random_time,
            left_speed=left_speed,
            right_speed=right_speed,
            resource_timer=thirst_timer,
            color='blue',
            threshold=500,
            max_value=900,
            timeout=5.0
        )

        if thirsty:
            continue

        # Handle hunger
        hunger, last_random_time, left_speed, right_speed, hunger_timer, hungry = robot1.handle_resource(
            resource_name="hunger",
            resource_value=hunger,
            last_random_time=last_random_time,
            left_speed=left_speed,
            right_speed=right_speed,
            resource_timer=hunger_timer,
            color='green',
            threshold=500,
            max_value=900,
            timeout=5.0
        )

        if hungry:
            continue

        # Priority 1: Search for colored boxes
        red_box_found, green_box_found, blue_box_found = search_coloured_boxes(
            robot1, red_box_found, green_box_found, blue_box_found, search_log
        )

        if red_box_found and green_box_found and blue_box_found:
            print("Woohooo!!!! All boxes found!")
            red_box_found = green_box_found = blue_box_found = False
            search_log.clear()

        # Priority 2: Obstacle avoidance
        if not (red_box_found or green_box_found or blue_box_found):
            if robot1.avoid_obstacles():
                robot1.set_actuators()
                robot1.step()
                continue

        # Priority 3: Wander
        last_random_time, left_speed, right_speed = robot1.wander(
            last_random_time, left_speed, right_speed
        )

        robot1.set_actuators()
        robot1.step()

if __name__ == "__main__":
    main()
