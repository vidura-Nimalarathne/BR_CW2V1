import robot
import random

# Only *actions* are listed here — stats (thirst/hunger/fatigue) are not.
BEHAVIOR_PRIORITIES = {
    "Die": 1,
    "Radiation Zone": 2,
    "Seek Resource": 3,
    "Lock On & Approach": 4,
    "Drink Water": 5,          # triggered by thirst stat
    "Foraging": 6,             # triggered by hunger stat
    "Spawning": 7,
    "Sleep": 8,                # triggered by fatigue stat
    "Search Colored Boxes": 9,
    "Avoid Obstacles": 10,     # kept (assignment requirement) but also called reflexively in higher behaviors
    "Wander": 11
}

def main():
    robot1 = robot.ARAP()
    robot1.init_devices()

    # Box search
    red_box_found = False
    green_box_found = False
    blue_box_found = False
    search_log = []

    # Stats
    thirst = 600
    hunger = 600
    fatigue = 600
    health = 600

    # Radiation state
    radiation_exposure = False
    post_radiation_timer = 0.0
    inside_radiation = False
    radiation_start_time = 0.0

    # Decay timers
    last_time = robot1.robot.getTime()
    last_display_time = last_time
    thirst_timer = 0.0
    hunger_timer = 0.0
    fatigue_timer = 0.0

    # Wander params
    base_speed = random.uniform(0.5 * robot1.MAX_SPEED, robot1.MAX_SPEED)
    speed_diff = random.uniform(-0.5, 0.5) * robot1.MAX_SPEED
    left_speed = base_speed + speed_diff
    right_speed = base_speed - speed_diff
    last_random_time = robot1.robot.getTime()

    # Unique-behavior logging
    executed_behaviors = set()
    unique_behaviors_ordered = []

    last_unique_print_time = last_time  # For printing unique behaviors every 5 seconds

    def log_behavior_once(name: str):
        if name not in executed_behaviors:
            executed_behaviors.add(name)
            unique_behaviors_ordered.append(name)
            ordered = sorted(executed_behaviors, key=lambda n: BEHAVIOR_PRIORITIES[n])
            print(f"Unique behaviors so far ({len(ordered)}): {ordered}")

    # Spawning flags
    foraging_just_finished = False
    spawn_requested = False

    # Decoupled seek / lock-on state
    # pending_seek: (resource_name, color, rotate_duration, color_threshold)
    pending_seek = None
    # pending_lock_on: (resource_name, color, timeout, max_value)
    pending_lock_on = None

    # Tunables
    THIRST_THRESHOLD = 500
    HUNGER_THRESHOLD = 500
    FATIGUE_THRESHOLD = 500
    RESOURCE_MAX = 900

    while True:
        robot1.reset_actuator_values()
        robot1.get_sensor_input()
        robot1.blink_leds()

        # Update decay
        current_time = robot1.robot.getTime()
        elapsed = current_time - last_time
        thirst_timer += elapsed
        hunger_timer += elapsed
        fatigue_timer += elapsed

        if thirst_timer >= 1.0:
            thirst -= 10 * int(thirst_timer)
            thirst_timer -= int(thirst_timer)
            thirst = max(0, thirst)

        if hunger_timer >= 1.0:
            hunger -= 5 * int(hunger_timer)
            hunger_timer -= int(hunger_timer)
            hunger = max(0, hunger)

        if fatigue_timer >= 1.0:
            fatigue -= 1 * int(fatigue_timer)
            fatigue_timer -= int(fatigue_timer)
            fatigue = max(0, fatigue)

        # Periodic status print
        if current_time - last_display_time >= 2.0:
            print(f"Thirst: {thirst} | Hunger: {hunger} | Fatigue: {fatigue} | Health: {health}")
            last_display_time = current_time

        # Print unique behaviors encountered every 5 seconds
        if current_time - last_unique_print_time >= 5.0:
            ordered = sorted(executed_behaviors, key=lambda n: BEHAVIOR_PRIORITIES[n])
            print(f"[{current_time:.2f}s] Unique behaviors so far ({len(ordered)}): {ordered}")
            last_unique_print_time = current_time

        sorted_behaviors = sorted(BEHAVIOR_PRIORITIES.items(), key=lambda x: x[1])
        behavior_executed = False

        for behavior_name, _ in sorted_behaviors:
            # 1) DIE
            if behavior_name == "Die":
                if health <= 0:
                    log_behavior_once(behavior_name)
                    print("Health depleted! Robot stopping.")
                    robot1.stop()
                    behavior_executed = True
                    break

            # 2) RADIATION ZONE
            elif behavior_name == "Radiation Zone":
                (radiation_triggered,
                 health,
                 radiation_exposure,
                 post_radiation_timer,
                 inside_radiation,
                 radiation_start_time) = robot1.handle_radiation_zone(
                    current_time=current_time,
                    last_time=last_time,
                    health=health,
                    radiation_exposure=radiation_exposure,
                    post_radiation_timer=post_radiation_timer,
                    inside_radiation=inside_radiation,
                    radiation_start_time=radiation_start_time
                )

                if radiation_triggered:
                    log_behavior_once(behavior_name)
                    behavior_executed = True
                    break

            # 3) SEEK RESOURCE  (runs only if requested)
            elif behavior_name == "Seek Resource":
                if pending_seek is not None:
                    log_behavior_once(behavior_name)

                    resource_name, color, rotate_duration, color_threshold = pending_seek
                    found = robot1.seek_resource(
                        target_color=color,
                        color_threshold=color_threshold,
                        rotate_duration=rotate_duration
                    )

                    if found:
                        timeout = 5.0
                        pending_lock_on = (resource_name, color, timeout, RESOURCE_MAX)

                    pending_seek = None
                    behavior_executed = True
                    break

            # 4) LOCK ON & APPROACH  (runs only if requested)
            elif behavior_name == "Lock On & Approach":
                if pending_lock_on is not None:
                    log_behavior_once(behavior_name)

                    resource_name, color, to, max_val = pending_lock_on
                    reached = robot1.lock_on_and_approach(target_color=color, timeout=to)

                    if reached:
                        if resource_name == "thirst":
                            thirst = robot1.consume_resource("thirst", thirst, max_val)
                            thirst = min(thirst, max_val)
                            thirst_timer = 0.0
                        elif resource_name == "hunger":
                            hunger = robot1.consume_resource("hunger", hunger, max_val)
                            hunger = min(hunger, max_val)
                            hunger_timer = 0.0
                        robot1.last_time = robot1.robot.getTime()
                    else:
                        print(f"Failed to reach {color} patch. Re-attempting search...")

                    pending_lock_on = None
                    behavior_executed = True
                    break

            # 5) DRINK WATER  (stats gate -> requests Seek/Lock)
            elif behavior_name == "Drink Water":
                if thirst < THIRST_THRESHOLD:
                    # <-- FIX: log as soon as it's triggered
                    log_behavior_once(behavior_name)

                    (thirst, last_random_time, left_speed, right_speed, thirst_timer,
                     handled, wants_seek, seek_args, wants_lock_on, lock_on_args) = robot1.handle_resource(
                        resource_name="thirst",
                        resource_value=thirst,
                        last_random_time=last_random_time,
                        left_speed=left_speed,
                        right_speed=right_speed,
                        resource_timer=thirst_timer,
                        color='blue',
                        threshold=THIRST_THRESHOLD,
                        max_value=RESOURCE_MAX,
                        timeout=5.0,
                        request_seek_and_lock_on=True,
                        rotate_duration=3.0,
                        color_threshold=50
                    )

                    if wants_seek and pending_seek is None and pending_lock_on is None:
                        pending_seek = seek_args  # (resource_name, color, rotate_duration, color_threshold)
                        behavior_executed = True
                        break

                    if handled and not wants_seek and not wants_lock_on:
                        # safe to leave this here; set prevents double print
                        log_behavior_once(behavior_name)
                        behavior_executed = True
                        break

            # 6) Forage requests Seek/Lock)
            elif behavior_name == "Foraging":
                if hunger < HUNGER_THRESHOLD:
                    # <-- FIX: log as soon as it's triggered
                    log_behavior_once(behavior_name)

                    (hunger, last_random_time, left_speed, right_speed, hunger_timer,
                     handled, wants_seek, seek_args, wants_lock_on, lock_on_args) = robot1.handle_resource(
                        resource_name="hunger",
                        resource_value=hunger,
                        last_random_time=last_random_time,
                        left_speed=left_speed,
                        right_speed=right_speed,
                        resource_timer=hunger_timer,
                        color='green',
                        threshold=HUNGER_THRESHOLD,
                        max_value=RESOURCE_MAX,
                        timeout=5.0,
                        request_seek_and_lock_on=True,
                        rotate_duration=3.0,
                        color_threshold=50
                    )

                    if wants_seek and pending_seek is None and pending_lock_on is None:
                        pending_seek = seek_args
                        foraging_just_finished = True
                        spawn_requested = False
                        behavior_executed = True
                        break

                    if handled and not wants_seek and not wants_lock_on:
                        log_behavior_once(behavior_name)
                        behavior_executed = True
                        break

            # 7) Spawn a baby!
            elif behavior_name == "Spawning":
                if foraging_just_finished and not spawn_requested and thirst > 600 and health > 500 and fatigue > 500:
                    log_behavior_once(behavior_name)
                    print("Spawning behavior active: seeking white patch.")

                    found_white = robot1.seek_resource('white', color_threshold=50, rotate_duration=3.0)
                    if found_white:
                        print("White patch detected. Locking on and approaching...")
                        reached_white = robot1.lock_on_and_approach('white', timeout=8.0)
                        if reached_white:
                            print("Reached white patch, sending spawn request.")
                            robot1.send_spawn_request(distance=0.2)
                            spawn_requested = True
                            foraging_just_finished = False
                            behavior_executed = True
                            break
                        else:
                            print("Failed to reach white patch within timeout.")
                    else:
                        print("White patch not found during seek.")

            # 8) Sleep
            elif behavior_name == "Sleep":
                if fatigue < FATIGUE_THRESHOLD:
                    log_behavior_once(behavior_name)
                    fatigue, health = robot1.sleep(
                        sleep_level=fatigue,
                        thirst=thirst,
                        hunger=hunger,
                        health=health,
                        sleep_threshold=700,
                        max_sleep=RESOURCE_MAX,
                        rate_per_sec=100,
                        health_penalty_rate=2,
                        duration=5
                    )
                    behavior_executed = True
                    break

            # 9) Search for 3 coloured boxes as a hobby. once bump in to a coloured box (3 boxes of RGB), will be recorded in to a list
            elif behavior_name == "Search Colored Boxes":
                red_box_found, green_box_found, blue_box_found = robot1.search_coloured_boxes(
                    red_box_found, green_box_found, blue_box_found, search_log
                )
                if red_box_found and green_box_found and blue_box_found:
                    print("Woohooo!!!! All boxes found!")
                    red_box_found = green_box_found = blue_box_found = False
                    search_log.clear()
                log_behavior_once(behavior_name)

            # 10) Obstacle avoidance
            elif behavior_name == "Avoid Obstacles":
                if robot1.avoid_obstacles():
                    log_behavior_once(behavior_name)
                    robot1.set_actuators()
                    robot1.step()
                    behavior_executed = True
                    break

            # 11) Wander - random wandering
            elif behavior_name == "Wander":
                if not behavior_executed:
                    log_behavior_once(behavior_name)
                    last_random_time, left_speed, right_speed = robot1.wander(
                        last_random_time, left_speed, right_speed
                    )
                    behavior_executed = True
                    break

        # Apply motors and step
        robot1.set_actuators()
        robot1.step()
        last_time = current_time

if __name__ == "__main__":
    main()
