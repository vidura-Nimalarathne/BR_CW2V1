# controller_brooks.py
from dataclasses import dataclass
from typing import Optional, List, Set
import random

import robot  # your robot.py (BRCW2)

# ---------- Command & World State ----------

@dataclass
class MotorCmd:
    left: float = 0.0
    right: float = 0.0
    stop: bool = False  # Highest layer can force a full stop


@dataclass
class WorldState:
    # internal stats
    thirst: int = 600
    hunger: int = 600
    fatigue: int = 600
    health: int = 600

    # timers
    thirst_timer: float = 0.0
    hunger_timer: float = 0.0
    fatigue_timer: float = 0.0
    last_time: float = 0.0

    # radiation
    radiation_exposure: bool = False
    post_radiation_timer: float = 0.0
    inside_radiation: bool = False
    radiation_start_time: float = 0.0

    # box search
    red_box_found: bool = False
    green_box_found: bool = False
    blue_box_found: bool = False
    search_log: List[str] = None

    # resource pipeline
    pending_seek: Optional[tuple] = None       # (resource_name, color, rotate_duration, color_threshold)
    pending_lock_on: Optional[tuple] = None    # (resource_name, color, timeout, max_value)
    foraging_just_finished: bool = False
    spawn_requested: bool = False

    # bookkeeping
    unique_behaviors_seen: Set[str] = None

    def __post_init__(self):
        if self.search_log is None:
            self.search_log = []
        if self.unique_behaviors_seen is None:
            self.unique_behaviors_seen = set()


# ---------- Behavior Base Class ----------

class Behavior:
    """Brooks-style layer. Higher priority number = higher layer (executed last, can override)."""
    priority: int = 0

    def __init__(self, rb: robot.BRCW2):
        self.robot = rb

    def should_run(self, state: WorldState) -> bool:
        return True

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        return cmd

    def name(self) -> str:
        return self.__class__.__name__


# ---------- Concrete Behaviors ----------

# 0) Lowest: Wander
class Wander(Behavior):
    priority = 0

    def __init__(self, rb):
        super().__init__(rb)
        self.last_random_time = -1e9
        self.left = 0.0
        self.right = 0.0

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        if cmd.stop:
            return cmd

        now = self.robot.robot.getTime()
        interval = 2.0
        if now - self.last_random_time > interval:
            base_speed = random.uniform(0.3 * self.robot.MAX_SPEED, self.robot.MAX_SPEED)
            speed_diff = random.uniform(-0.3, 0.3) * self.robot.MAX_SPEED
            self.left = base_speed + speed_diff
            self.right = base_speed - speed_diff
            self.last_random_time = now

        cmd.left = self.left
        cmd.right = self.right
        return cmd


# 1) Avoid Obstacles
class AvoidObstacles(Behavior):
    priority = 1

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        if cmd.stop:
            return cmd

        # This function internally moves/waits; after it returns we override final speeds anyway.
        if self.robot.avoid_obstacles():
            # Use what avoid_obstacles decided (robot.speeds aren't set there, but motors were driven).
            # We just zero our speeds so we don't immediately override the reflex motion.
            cmd.left = 0.0
            cmd.right = 0.0
        return cmd


# 2) Search Colored Boxes (hobby)
class SearchColoredBoxes(Behavior):
    priority = 2

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        r, g, b = state.red_box_found, state.green_box_found, state.blue_box_found
        r, g, b = self.robot.search_coloured_boxes(r, g, b, state.search_log)
        state.red_box_found, state.green_box_found, state.blue_box_found = r, g, b

        if r and g and b:
            print("Woohooo!!!! All boxes found!")
            state.red_box_found = state.green_box_found = state.blue_box_found = False
            state.search_log.clear()

        return cmd


# 3) Sleep
class Sleep(Behavior):
    priority = 3
    FATIGUE_THRESHOLD = 500

    def should_run(self, state: WorldState) -> bool:
        return state.fatigue < self.FATIGUE_THRESHOLD

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        cmd.left = 0.0
        cmd.right = 0.0
        state.fatigue, state.health = self.robot.sleep(
            sleep_level=state.fatigue,
            thirst=state.thirst,
            hunger=state.hunger,
            health=state.health,
            sleep_threshold=700,
            max_sleep=900,
            rate_per_sec=100,
            health_penalty_rate=2,
            duration=5
        )
        return cmd


# 4) Spawning
class Spawning(Behavior):
    priority = 4

    def should_run(self, state: WorldState) -> bool:
        return (state.foraging_just_finished and
                not state.spawn_requested and
                state.thirst > 600 and state.health > 200 and state.fatigue > 500)

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        print("Spawning behavior active: seeking white patch.")
        found_white = self.robot.seek_resource('white', color_threshold=50, rotate_duration=3.0)

        # While seeking, the robot drove itself; we leave cmd as is
        if found_white:
            print("White patch detected. Locking on and approaching.")
            reached_white = self.robot.lock_on_and_approach('white', timeout=8.0)
            if reached_white:
                print("Reached white patch, sending spawn request.")
                self.robot.send_spawn_request(distance=0.2)
                state.spawn_requested = True
                state.foraging_just_finished = False
            else:
                print("Failed to reach white patch within timeout.")
        else:
            print("White patch not found during seek.")

        # Override motion during spawning step to be safe
        cmd.left = 0.0
        cmd.right = 0.0
        return cmd


# 5) Foraging (Hunger)
class ForagingMotivation(Behavior):
    priority = 5
    HUNGER_THRESHOLD = 500
    RESOURCE_MAX = 900

    def should_run(self, state: WorldState) -> bool:
        return (state.hunger < self.HUNGER_THRESHOLD and
                state.pending_seek is None and
                state.pending_lock_on is None)

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        state.pending_seek = ("hunger", "green", 3.0, 50)
        return cmd


# 6) Drink Water (Thirst)
class DrinkWaterMotivation(Behavior):
    priority = 6
    THIRST_THRESHOLD = 500
    RESOURCE_MAX = 900

    def should_run(self, state: WorldState) -> bool:
        return (state.thirst < self.THIRST_THRESHOLD and
                state.pending_seek is None and
                state.pending_lock_on is None)

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        state.pending_seek = ("thirst", "blue", 3.0, 50)
        return cmd


# 7) Seek Resource
class SeekResource(Behavior):
    priority = 7
    RESOURCE_MAX = 900

    def should_run(self, state: WorldState) -> bool:
        return state.pending_seek is not None

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        res_name, color, rotate_duration, color_threshold = state.pending_seek
        found = self.robot.seek_resource(
            target_color=color,
            color_threshold=color_threshold,
            rotate_duration=rotate_duration
        )
        if found:
            timeout = 5.0
            state.pending_lock_on = (res_name, color, timeout, self.RESOURCE_MAX)
        state.pending_seek = None

        # During seek, robot moved itself. Stop lower cmds for this step.
        cmd.left = 0.0
        cmd.right = 0.0
        return cmd


# 8) Lock On & Approach
class LockOnAndApproach(Behavior):
    priority = 8

    def should_run(self, state: WorldState) -> bool:
        return state.pending_lock_on is not None

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        resource_name, color, timeout, max_val = state.pending_lock_on
        reached = self.robot.lock_on_and_approach(target_color=color, timeout=timeout)

        if reached:
            if resource_name == "thirst":
                state.thirst = self.robot.consume_resource("thirst", state.thirst, max_val)
                state.thirst = min(state.thirst, max_val)
                state.thirst_timer = 0.0
            elif resource_name == "hunger":
                state.hunger = self.robot.consume_resource("hunger", state.hunger, max_val)
                state.hunger = min(state.hunger, max_val)
                state.hunger_timer = 0.0
                # Mark that foraging just finished so Spawning can consider acting
                state.foraging_just_finished = True
                state.spawn_requested = False

            self.robot.last_time = self.robot.robot.getTime()
        else:
            print(f"Failed to reach {color} patch. Re-attempting search.")

        state.pending_lock_on = None

        cmd.left = 0.0
        cmd.right = 0.0
        return cmd


# 9) Radiation Zone
class RadiationZone(Behavior):
    priority = 9

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        (radiation_triggered,
         state.health,
         state.radiation_exposure,
         state.post_radiation_timer,
         state.inside_radiation,
         state.radiation_start_time) = self.robot.handle_radiation_zone(
            current_time=self.robot.robot.getTime(),
            last_time=state.last_time,
            health=state.health,
            radiation_exposure=state.radiation_exposure,
            post_radiation_timer=state.post_radiation_timer,
            inside_radiation=state.inside_radiation,
            radiation_start_time=state.radiation_start_time
        )

        if radiation_triggered:
            # Let radiation logic own movement this tick
            cmd.left = 0.0
            cmd.right = 0.0

        return cmd


# 10) Die (highest)
class Die(Behavior):
    priority = 10

    def should_run(self, state: WorldState) -> bool:
        return state.health <= 0

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        print("Health depleted! Robot stopping.")
        cmd.stop = True
        cmd.left = 0.0
        cmd.right = 0.0
        return cmd


# ---------- Controller loop ----------

def run(rb: robot.BRCW2):
    state = WorldState()
    state.last_time = rb.robot.getTime()

    layers = [
        Wander(rb),                 # 0
        AvoidObstacles(rb),         # 1
        SearchColoredBoxes(rb),     # 2
        Sleep(rb),                  # 3
        Spawning(rb),               # 4
        ForagingMotivation(rb),     # 5
        DrinkWaterMotivation(rb),   # 6
        SeekResource(rb),           # 7
        LockOnAndApproach(rb),      # 8
        RadiationZone(rb),          # 9
        Die(rb),                    # 10
    ]
    layers.sort(key=lambda b: b.priority)

    last_display_time = state.last_time
    last_unique_print_time = state.last_time

    while True:
        rb.reset_actuator_values()
        rb.get_sensor_input()
        rb.blink_leds()

        # ===== Stats decay =====
        now = rb.robot.getTime()
        elapsed = now - state.last_time
        state.last_time = now
        state.thirst_timer += elapsed
        state.hunger_timer += elapsed
        state.fatigue_timer += elapsed

        if state.thirst_timer >= 1.0:
            state.thirst -= 5 * int(state.thirst_timer)
            state.thirst_timer -= int(state.thirst_timer)
            state.thirst = max(0, state.thirst)

        if state.hunger_timer >= 1.0:
            state.hunger -= 2 * int(state.hunger_timer)
            state.hunger_timer -= int(state.hunger_timer)
            state.hunger = max(0, state.hunger)

        if state.fatigue_timer >= 1.0:
            state.fatigue -= 1 * int(state.fatigue_timer)
            state.fatigue_timer -= int(state.fatigue_timer)
            state.fatigue = max(0, state.fatigue)

        # ===== Subsumption pass: bottom -> top =====
        cmd = MotorCmd()
        for b in layers:
            if b.should_run(state):
                # log unique behaviors
                if b.name() not in state.unique_behaviors_seen:
                    state.unique_behaviors_seen.add(b.name())
                    ordered = sorted(
                        state.unique_behaviors_seen,
                        key=lambda n: next(x.priority for x in layers if x.name() == n)
                    )
                    print(f"Unique behaviors so far ({len(ordered)}): {ordered}")

                cmd = b.act(state, cmd)
                if cmd.stop:
                    break

        # ===== Apply final command =====
        rb.speeds[rb.LEFT] = max(-rb.MAX_SPEED, min(cmd.left,  rb.MAX_SPEED))
        rb.speeds[rb.RIGHT] = max(-rb.MAX_SPEED, min(cmd.right, rb.MAX_SPEED))
        if cmd.stop:
            rb.stop()

        rb.set_actuators()
        rb.step()  # advance simulation

        # ===== Debug prints =====
        if now - last_display_time >= 2.0:
            print(f"Thirst: {state.thirst} | Hunger: {state.hunger} | Fatigue: {int(state.fatigue)} | Health: {state.health}")
            last_display_time = now

        if now - last_unique_print_time >= 5.0:
            ordered = sorted(
                state.unique_behaviors_seen,
                key=lambda n: next(x.priority for x in layers if x.name() == n)
            )
            print(f"[{now:.2f}s] Unique behaviors so far ({len(ordered)}): {ordered}")
            last_unique_print_time = now


def main():
    rb = robot.BRCW2()
    rb.init_devices()  # already called in __init__, but harmless
    run(rb)


if __name__ == "__main__":
    main()
