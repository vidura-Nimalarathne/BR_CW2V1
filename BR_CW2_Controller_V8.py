import robot
import random

# Behavior priority dictionary (lower number = higher priority)
BEHAVIOR_PRIORITIES = {
    "Die": 1,
    "Avoid Obstacles": 2,
    "Radiation Zone": 3,
    "Thirst": 4,
    "Drink water": 5,
    "Hunger": 6,
    "Foraging": 7,
    "Fatigue": 8,
    "Spawning": 9,    # changed from 8.5 to 9
    "Sleep": 10,      # changed from 9 to 10
    "Search Colored Boxes": 11,
    "Wander": 12
}

def main():
    robot1 = robot.ARAP()
    robot1.init_devices()

    # Initialize box found flags and search log
    red_box_found = False
    green_box_found = False
    blue_box_found = False
    search_log = []

    # Resource stats
    thirst = 600
    hunger = 600
    fatigue = 600
    health = 600

    # Radiation zone behavior state
    radiation_exposure = False
    post_radiation_timer = 0.0
    inside_radiation = False
    radiation_start_time = 0.0

    last_time = robot1.robot.getTime()
    last_display_time = last_time
    thirst_timer = 0.0
    hunger_timer = 0.0
    fatigue_timer = 0.0

    # Wandering parameters
    base_speed = random.uniform(0.5 * robot1.MAX_SPEED, robot1.MAX_SPEED)
    speed_diff = random.uniform(-0.5, 0.5) * robot1.MAX_SPEED
    left_speed = base_speed + speed_diff
    right_speed = base_speed - speed_diff
    last_random_time = robot1.robot.getTime()

    # Behavior tracking
    last_behavior = None
    executed_behaviors = set()

    # New flags for spawning
    foraging_just_finished = False
    spawn_requested = False
    spawning_active = False  # To track if we are in spawning process

    while True:
        robot1.reset_actuator_values()
        robot1.get_sensor_input()
        robot1.blink_leds()

        # Update resource decay timers
        current_time = robot1.robot.getTime()
        elapsed = current_time - last_time
        thirst_timer += elapsed
        hunger_timer += elapsed
        fatigue_timer += elapsed

        # Decay thirst
        if thirst_timer >= 1.0:
            thirst -= 5 * int(thirst_timer)
            thirst_timer -= int(thirst_timer)
            thirst = max(0, thirst)

        # Decay hunger
        if hunger_timer >= 1.0:
            hunger -= 2 * int(hunger_timer)
            hunger_timer -= int(hunger_timer)
            hunger = max(0, hunger)

        # Decay fatigue
        if fatigue_timer >= 1.0:
            fatigue -= 1 * int(fatigue_timer)
            fatigue_timer -= int(fatigue_timer)
            fatigue = max(0, fatigue)

        # Print resource status
        if current_time - last_display_time >= 5.0:
            print(f"Thirst Level: {thirst} | Hunger Level: {hunger} | Fatigue Level: {fatigue} | Health: {health}")
            last_display_time = current_time

        sorted_behaviors = sorted(BEHAVIOR_PRIORITIES.items(), key=lambda x: x[1])
        behavior_executed = False

        for behavior_name, priority in sorted_behaviors:
            if behavior_name == "Die":
                if health <= 0:
                    if last_behavior != behavior_name:
                        executed_behaviors.add(behavior_name)
                        print(f"Unique behaviors so far: {sorted(executed_behaviors)}")
                        last_behavior = behavior_name
                    print("Health depleted! Robot stopping.")
                    robot1.stop()
                    behavior_executed = True
                    break

            elif behavior_name == "Avoid Obstacles":
                if robot1.avoid_obstacles():
                    if last_behavior != behavior_name:
                        executed_behaviors.add(behavior_name)
                        print(f"Unique behaviors so far: {sorted(executed_behaviors)}")
                        last_behavior = behavior_name
                    robot1.set_actuators()
                    robot1.step()
                    behavior_executed = True
                    break

            elif behavior_name == "Radiation Zone":
                radiation_triggered, health, radiation_exposure, post_radiation_timer, inside_radiation, radiation_start_time = \
                    robot1.handle_radiation_zone(
                        current_time=current_time,
                        last_time=last_time,
                        health=health,
                        radiation_exposure=radiation_exposure,
                        post_radiation_timer=post_radiation_timer,
                        inside_radiation=inside_radiation,
                        radiation_start_time=radiation_start_time
                    )

                if radiation_triggered:
                    if last_behavior != behavior_name:
                        executed_behaviors.add(behavior_name)
                        print(f"Unique behaviors so far: {sorted(executed_behaviors)}")
                        last_behavior = behavior_name
                    behavior_executed = True
                    break

            elif behavior_name == "Thirst":
                if thirst < 500:
                    continue

            elif behavior_name == "Drink water":
                if thirst < 500:
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
                        if last_behavior != behavior_name:
                            executed_behaviors.add(behavior_name)
                            print(f"Unique behaviors so far: {sorted(executed_behaviors)}")
                            last_behavior = behavior_name
                        behavior_executed = True
                        break

            elif behavior_name == "Hunger":
                if hunger < 500:
                    continue

            elif behavior_name == "Foraging":
                if hunger < 500:
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
                        if last_behavior != behavior_name:
                            executed_behaviors.add(behavior_name)
                            print(f"Unique behaviors so far: {sorted(executed_behaviors)}")
                            last_behavior = behavior_name
                        behavior_executed = True

                        # Mark for spawning activation
                        foraging_just_finished = True
                        spawn_requested = False  # Reset spawn flag for next spawn cycle
                        break

            elif behavior_name == "Fatigue":
                if fatigue < 500:
                    continue

            elif behavior_name == "Spawning":
                if foraging_just_finished and not spawn_requested and thirst > 600 and health > 500 and fatigue > 500:
                    print("Spawning behavior active: seeking white patch.")

                    # First seek white patch by rotating with camera
                    found_white = robot1.seek_resource('white', color_threshold=50, rotate_duration=3.0)
                    if found_white:
                        print("White patch detected. Locking on and approaching...")
                        # Lock on and approach white patch
                        reached_white = robot1.lock_on_and_approach('white', timeout=8.0)
                        if reached_white:
                            print("Reached white patch, sending spawn request.")
                            robot1.send_spawn_request(distance=0.2)  # Send spawn request to supervisor
                            spawn_requested = True
                            foraging_just_finished = False  # Reset flag after spawn
                            behavior_executed = True
                            if last_behavior != behavior_name:
                                executed_behaviors.add(behavior_name)
                                print(f"Unique behaviors so far: {sorted(executed_behaviors)}")
                                last_behavior = behavior_name
                            break
                        else:
                            print("Failed to reach white patch within timeout.")
                    else:
                        print("White patch not found during seek.")
                else:
                    # Conditions not met or already spawned, do nothing this cycle
                    pass

            elif behavior_name == "Sleep":
                if fatigue < 500:
                    fatigue, health = robot1.sleep(
                        sleep_level=fatigue,
                        thirst=thirst,
                        hunger=hunger,
                        health=health,
                        sleep_threshold=700,
                        max_sleep=900,
                        rate_per_sec=100,
                        health_penalty_rate=2,
                        duration=5
                    )
                    if last_behavior != behavior_name:
                        executed_behaviors.add(behavior_name)
                        print(f"Unique behaviors so far: {sorted(executed_behaviors)}")
                        last_behavior = behavior_name
                    behavior_executed = True
                    break

            elif behavior_name == "Search Colored Boxes":
                red_box_found, green_box_found, blue_box_found = robot1.search_coloured_boxes(
                    red_box_found, green_box_found, blue_box_found, search_log
                )
                if red_box_found and green_box_found and blue_box_found:
                    print("Woohooo!!!! All boxes found!")
                    red_box_found = green_box_found = blue_box_found = False
                    search_log.clear()
                if last_behavior != behavior_name:
                    executed_behaviors.add(behavior_name)
                    print(f"Unique behaviors so far: {sorted(executed_behaviors)}")
                    last_behavior = behavior_name

            elif behavior_name == "Wander":
                if not behavior_executed:
                    last_random_time, left_speed, right_speed = robot1.wander(
                        last_random_time, left_speed, right_speed
                    )
                    if last_behavior != behavior_name:
                        executed_behaviors.add(behavior_name)
                        print(f"Unique behaviors so far: {sorted(executed_behaviors)}")
                        last_behavior = behavior_name
                    behavior_executed = True
                    break

        # Step robot
        robot1.set_actuators()
        robot1.step()
        last_time = current_time  # <- Make sure it's here after the loop

if __name__ == "__main__":
    main()
