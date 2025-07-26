#This subsumption architecture is brooks styled

from dataclasses import dataclass
from typing import Optional, List, Set
import random

import robot  # robot.py

# ---------- Command & World State ----------
#container for epuck motor 
@dataclass
class MotorCmd:
    left: float = 0.0 #final left motor speed on the current tick
    right: float = 0.0 #final right motor speed on the current tick
    stop: bool = False  # Highest layer (die) to force robot to stop

#container for all robot internal states and flags
@dataclass
class WorldState:
    # internal stats of epuck
    thirst: int = 600
    hunger: int = 600
    fatigue: int = 600
    health: int = 600

    # timers to use for internal stats handling
    thirst_timer: float = 0.0
    hunger_timer: float = 0.0
    fatigue_timer: float = 0.0
    last_time: float = 0.0

    # radiation handling variables and flags
    radiation_exposure: bool = False
    post_radiation_timer: float = 0.0
    inside_radiation: bool = False
    radiation_start_time: float = 0.0

    # coloured box search variables and flags
    red_box_found: bool = False
    green_box_found: bool = False
    blue_box_found: bool = False
    search_log: List[str] = None

    # class variables of resource pipeline and flags
    pending_seek: Optional[tuple] = None       # (resource_name, color, rotate_duration, color_threshold)
    pending_lock_on: Optional[tuple] = None    # (resource_name, color, timeout, max_value)
    foraging_just_finished: bool = False
    spawn_requested: bool = False

    # for recording unique behaviors demonstrated
    unique_behaviors_seen: Set[str] = None
    
    """
    to check if search_log or unique_behaviors_seen are None and then replace them with empty lists/sets to ensure 
    worldstate objects get their own instead of sharing with all instances.
    """
    def __post_init__(self):
        if self.search_log is None:
            self.search_log = []
        if self.unique_behaviors_seen is None:
            self.unique_behaviors_seen = set()



""" Base class for all behaviors in my subsumption architecture
higher the priority number, runs later to allow them to subsume lower level behaviors"""

class Behavior:
    
    priority: int = 0

    def __init__(self, rb: robot.BRCW2): # Keeping a handle to robot wrapper then behaviors can read sensors and set actuators
        self.robot = rb

    def should_run(self, state: WorldState) -> bool:  #Decide if wants to run this tick.is normally always true; concrete behaviors usually override this.
        return True

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:  #to perform the behavior’s action and modify the MotorCmd. Real behaviors override and return the updated cmd.
        return cmd

    def name(self) -> str: #helper for logging/debugging: returns the class name.
        return self.__class__.__name__


# ---------- Concrete Behaviors ----------

# priority 0) Lowest: Wander behavior. move around randomly until/unless higher priority behavior takes over
class Wander(Behavior):
    priority = 0

    def __init__(self, rb): #constructor for wander subclass
        super().__init__(rb) #call the parent class constructor so the base class can do setup 
        self.last_random_time = -1e9 #  # old timestamp so that on the first tick the code immediately pick a random speed.
        self.left = 0.0
        self.right = 0.0

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:  # If a higher-priority layer told to stop, no override.
        if cmd.stop:
            return cmd
        
        # change direction and speed every 2 seconds
        now = self.robot.robot.getTime()
        interval = 2.0
        
        ## if it's time to pick a new random pair of wheel speeds (if its 2s later)
        if now - self.last_random_time > interval:
            base_speed = random.uniform(0.3 * self.robot.MAX_SPEED, self.robot.MAX_SPEED) # Choosing a base forward speed between 0.3 : 1 of max speed.
            speed_diff = random.uniform(-0.3, 0.3) * self.robot.MAX_SPEED #adding a small random speed difference between the two wheels so it curves gently and naturally
            #storing new speeds
            self.left = base_speed + speed_diff
            self.right = base_speed - speed_diff
            #remembering when the speed was updated last
            self.last_random_time = now
        #apply the speeds to the outgoing command
        cmd.left = self.left
        cmd.right = self.right
        return cmd


# 1) Avoid Obstacles behavior
class Avoid_obstacles(Behavior):
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

"""
2) Search Colored Boxes (hobby). Set to a low priority as it's just a side quest. There are three boxes, green,blue and red,
placed in the arena. robot is to find them by bumping in to them when it moves randomly around. Once found,
it is addeed in to a list of found boxes.
"""
class Search_colored_boxes(Behavior): 
    priority = 2

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd: #Grab the current flags for which coloured boxes we've already found and copying them to locals.
        r, g, b = state.red_box_found, state.green_box_found, state.blue_box_found
        
         #Call helper function that actually does the sensing & logging.
        #returns updated booleans for red/green/blue and also writes into state.search_log
        r, g, b = self.robot.search_coloured_boxes(r, g, b, state.search_log)
        state.red_box_found, state.green_box_found, state.blue_box_found = r, g, b # Store the possibly updated results back into the shared world state
        
        #if all 3 boxes are found, display celebration msg, reset the list and repeat again.
        if r and g and b:
            print("Woohooo!!!! All boxes found!")
            state.red_box_found = state.green_box_found = state.blue_box_found = False
            state.search_log.clear()

        return cmd


# 3) Sleep behavior.
class Sleep(Behavior):
    priority = 3
    FATIGUE_THRESHOLD = 500 # when the fatigue stat drops bellow threshold, it is considered 'tired' and needs 'sleep' when conditions are met (cannot sleep hungry and thirsty).

    def should_run(self, state: WorldState) -> bool:# Only trigger the sleep behavior when we're actually tired.
         
        #  Lower fatigue value means epuck more tired.
        return state.fatigue < self.FATIGUE_THRESHOLD

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd: #the act of sleeping
        #robot stop during sleep.
        cmd.left = 0.0
        cmd.right = 0.0
        
        state.fatigue, state.health = self.robot.sleep( # Offload the actual sleeping logic to a helper in robot wrapper.
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


# 4) Spawning behavior for epuck /  actual spawning is done by supervisor robot. epuck will just send a message to supervisor
class Spawning(Behavior):
    priority = 4

    def should_run(self, state: WorldState) -> bool: #spawning triggers only after successfully foraging while thirst,health and fatigue are above thresholds.
        return (state.foraging_just_finished and
                not state.spawn_requested and
                state.thirst > 600 and state.health > 200 and state.fatigue > 500)

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd: #epucks spawning routine 
        print("Spawning behavior active: seeking white patch.")
        
         #First, scan around to see if camera can find a white patch to spawn on.
        # The helper handle the rotate/scan logic.
        found_white = self.robot.seek_resource('white', color_threshold=50, rotate_duration=3.0)

        # if white was found via camera
        if found_white:
            print("White patch detected. Locking on and approaching.")
            #move towards the white patch using helper method.
            reached_white = self.robot.lock_on_and_approach('white', timeout=8.0)
            if reached_white:
                print("Reached white patch, sending spawn request.")
                #use spawn request method in robot.py to send a msg to supervisor to spawn at a specific distance from the epuck
                self.robot.send_spawn_request(distance=0.2)
                state.spawn_requested = True #ensure no spaming
                state.foraging_just_finished = False #only one attempt per foraging
            else:
                print("Failed to reach white patch within timeout.")
        else:
            print("White patch not found during seek.")

        # Override motion during spawning step to be safe so that robots dont end up on top of each other.
        cmd.left = 0.0
        cmd.right = 0.0
        return cmd


# 5) Foraging behavior (replenishes hunger stat)
class Foraging(Behavior):
    priority = 5
    HUNGER_THRESHOLD = 500 #threshold to trigger foraging behavior
    RESOURCE_MAX = 900 #hunger stat upper limit during foraging behavior

    def should_run(self, state: WorldState) -> bool: #checking condition before running behavior
        return (state.hunger < self.HUNGER_THRESHOLD and #check if actually bellow the hunger threoshold
                state.pending_seek is None and # checks if robot is in the middle of a seek resource behavior
                state.pending_lock_on is None) #checks if robot is in the middle of a lock on behavior

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        """Instead of doing the seeking here,"job" for the Seek_resource layer (priority 7) is created.
        Tuple layout I chose:(which stat, "colour of the patch, rotate_duration, color_threshold)"""
        state.pending_seek = ("hunger", "green", 3.0, 50)
        return cmd


# 6) Drink Water behavior (replenishes thirst stat)
class Drink_water(Behavior):
    priority = 6
    THIRST_THRESHOLD = 500 #threshold to trigger drink water behavior
    RESOURCE_MAX = 900 #thirst stat upper limit during foraging behavior

    def should_run(self, state: WorldState) -> bool:#checking if drink water behavior should start
        return (state.thirst < self.THIRST_THRESHOLD and #ensures current thirst level is bellow threshold
                state.pending_seek is None and #ensure robot is not in the middle of a seek resource behavior
                state.pending_lock_on is None) #ensures robot is not in the middle of a lock on behavior

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        """
        Instead of doing the seeking here, a "job" for the SeekResource layer (priority 7) is created.
        Tuple layout I chose: (which stat, colour of the patch, rotate_duration, color_threshold)
        """
        state.pending_seek = ("thirst", "blue", 3.0, 50)
        return cmd



# 7) Seek resourse behavior. this behavior is designed to seek resources(coloured patches on arena) based on the situation (hunger,thirst)
class Seek_resource(Behavior):
    priority = 7
    RESOURCE_MAX = 900

    def should_run(self, state: WorldState) -> bool: #ensure it is ok to run. only run when hunger/thirst has queued a job
        return state.pending_seek is not None 

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        # Unpack the "job" that was scheduled by Foraging/Drink_water.
        res_name, color, rotate_duration, color_threshold = state.pending_seek
        
        # Ask the robot wrapper to actually rotate/scan for the colour.
        # This function drives the robot itself while scanning.
        found = self.robot.seek_resource(
            target_color=color,
            color_threshold=color_threshold,
            rotate_duration=rotate_duration
        )
        if found:
            # If robot spotted the resource, move to next phase:
            # a tighter lock-on + approach handled by the next layer, lock on and approach.
            timeout = 5.0
            state.pending_lock_on = (res_name, color, timeout, self.RESOURCE_MAX)
        state.pending_seek = None

        # During seek, robot moved itself. Stops lower cmds for this step.
        cmd.left = 0.0
        cmd.right = 0.0
        return cmd


# 8) Lock On & Approach behavior for final approach and cosumption
class Lock_on_and_approach(Behavior):
    priority = 8

    def should_run(self, state: WorldState) -> bool: # Only run if a previous step, Seek_resource, scheduled a lock-on job.
        return state.pending_lock_on is not None

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        
        # Unpack what robot need to approach
        resource_name, color, timeout, max_val = state.pending_lock_on
        
        # Try to visually lock on and drive to the coloured patch.The helper function(in robot.py) handles the movement internally
        reached = self.robot.lock_on_and_approach(target_color=color, timeout=timeout)

        if reached: # If robot actually reached it, consume the resource and refill the right stat, clamped to max threshold
            if resource_name == "thirst": #do for thirst (drink water)
                state.thirst = self.robot.consume_resource("thirst", state.thirst, max_val)
                state.thirst = min(state.thirst, max_val)
                state.thirst_timer = 0.0
            elif resource_name == "hunger": #for foraging(hunger)
                state.hunger = self.robot.consume_resource("hunger", state.hunger, max_val)
                state.hunger = min(state.hunger, max_val)
                state.hunger_timer = 0.0
                # Mark that foraging just finished so Spawning can consider acting
                state.foraging_just_finished = True
                state.spawn_requested = False
             # Update robot's last_time so the decay math (the gradual reduction of stats) remains consistent
            self.robot.last_time = self.robot.robot.getTime()
        else:
            print(f"Failed to reach {color} patch. Re-attempting search.")
            
          # Clear this job no matter what.
        state.pending_lock_on = None

       # Since the lock_on helper already drove the robot, zero speeds here so lower layers don't immediately overwrite things.
        cmd.left = 0.0
        cmd.right = 0.0
        return cmd


# 9) Radiation Zone
class Radiation_zone(Behavior):
    priority = 9

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        """Call the helper function in my robot class to handle everything about radiation.It checks if robot is standing on a red patch, 
        updates health, and decides if we need to escape. It returns a lot of updated values, unpacking them all back into the stats """
        (radiation_triggered,
         state.health,
         state.radiation_exposure,
         state.post_radiation_timer,
         state.inside_radiation,
         state.radiation_start_time) = self.robot.handle_radiation_zone(
            current_time=self.robot.robot.getTime(), # current simulation time
            last_time=state.last_time, # last tick time (for timing logic)
            health=state.health, # current health (will drop if we stay in radiation)
            radiation_exposure=state.radiation_exposure,
            post_radiation_timer=state.post_radiation_timer,
            inside_radiation=state.inside_radiation,
            radiation_start_time=state.radiation_start_time
        )
        
        # If robot just triggered a radiation reaction (like reversing out), stops the regular motor commands, escape motion isn't overridden.
        if radiation_triggered:
            cmd.left = 0.0
            cmd.right = 0.0

        return cmd


# 10) Die (highest priority because when dead, no other behavior can be done)
class Die(Behavior):
    priority = 10

    def should_run(self, state: WorldState) -> bool: #checks if health is 0 or lower (clamped so will only reach 0)
        return state.health <= 0

    def act(self, state: WorldState, cmd: MotorCmd) -> MotorCmd:
        print("Health depleted! Robot stopping.")
        
        # Set the stop flag so lower layers know not to overwrite the command.
        cmd.stop = True
        cmd.left = 0.0
        cmd.right = 0.0
        return cmd


# ---------- Controller loop ----------

def run(rb: robot.BRCW2):

    # Create a fresh world state to track all robots internal variables (thirst, hunger, etc.)
    state = WorldState()
    
    # Store the current simulation time to facilitate elapsed time calculations each tick.
    state.last_time = rb.robot.getTime()

    # Build the list of behaviors (lowest priority first, highest last).
    layers = [
        Wander(rb),                 # 0
        Avoid_obstacles(rb),        # 1
        Search_colored_boxes(rb),   # 2
        Sleep(rb),                  # 3
        Spawning(rb),               # 4
        Foraging(rb),               # 5
        Drink_water(rb),            # 6
        Seek_resource(rb),          # 7
        Lock_on_and_approach(rb),   # 8
        Radiation_zone(rb),         # 9
        Die(rb),                    # 10
    ]
    # Just in case, reorder the list manually, sort again by the declared priority field.
    layers.sort(key=lambda b: b.priority)
    
    
    # Bookkeeping so no spamming the console
    last_display_time = state.last_time
    last_unique_print_time = state.last_time
    
    # Main control loop
    while True:
        # Resetting any actuator commands from the previous tick
        rb.reset_actuator_values()
        # Reading all sensor values for this tick
        rb.get_sensor_input()
        # Just for feedback, blink the LEDs (not cruicial in anyway)
        rb.blink_leds()

        #Stats decay handling
        now = rb.robot.getTime()
        elapsed = now - state.last_time # how much time passed since last tick
        state.last_time = now
        
        # Accumulate elapsed time into the stat timers.
        state.thirst_timer += elapsed
        state.hunger_timer += elapsed
        state.fatigue_timer += elapsed

        if state.thirst_timer >= 1.0: # Decrease thirst every 1s in discrete steps.
            state.thirst -= 5 * int(state.thirst_timer) #5 per second to simulate becoming thirsty faster than becoming hungry
            state.thirst_timer -= int(state.thirst_timer)
            state.thirst = max(0, state.thirst)

        if state.hunger_timer >= 1.0: # Decrease hunger every 1s in discrete steps.
            state.hunger -= 2 * int(state.hunger_timer) #2 per second to show slower than thirst but faster than fatigue
            state.hunger_timer -= int(state.hunger_timer)
            state.hunger = max(0, state.hunger)

        if state.fatigue_timer >= 1.0: # Decrease fatigue (make robot tired) every 1s in discrete steps.
            state.fatigue -= 1 * int(state.fatigue_timer) #1 per second
            state.fatigue_timer -= int(state.fatigue_timer)
            state.fatigue = max(0, state.fatigue)

        #Subsumption pass: bottom -> top. Start with a blank motor command; each behavior modify it.
        cmd = MotorCmd()
        for b in layers:
            if b.should_run(state): 
                # Track which behaviors have actually run at least once for debugging
                if b.name() not in state.unique_behaviors_seen:
                    state.unique_behaviors_seen.add(b.name())
                    # Print them in order of their priority so to see progression clearly.
                    ordered = sorted(
                        state.unique_behaviors_seen,
                        key=lambda n: next(x.priority for x in layers if x.name() == n)
                    )
                    print(f"Unique behaviors so far ({len(ordered)}): {ordered}")
                    
                # Let the behavior act, possibly overwriting/augmenting the motor command.
                cmd = b.act(state, cmd)
                if cmd.stop:
                    break

        # Apply final commands. Clamp speeds to the robot's speed limits and write them into the robot's speed array.
        rb.speeds[rb.LEFT] = max(-rb.MAX_SPEED, min(cmd.left,  rb.MAX_SPEED))
        rb.speeds[rb.RIGHT] = max(-rb.MAX_SPEED, min(cmd.right, rb.MAX_SPEED))
        if cmd.stop:
            rb.stop() # 'hard stop' helper.
        
        # Push the final actuator values to the simulation.
        rb.set_actuators()
        rb.step()  # advance simulation one step

        # Debug prints
        if now - last_display_time >= 2.0: #print core stats every 2 seconds
            print(f"Thirst: {state.thirst} | Hunger: {state.hunger} | Fatigue: {int(state.fatigue)} | Health: {state.health}")
            last_display_time = now

        if now - last_unique_print_time >= 5.0: #reprint list of unique behaviors observed already every 5 seconds
            ordered = sorted(
                state.unique_behaviors_seen,
                key=lambda n: next(x.priority for x in layers if x.name() == n)
            )
            print(f"[{now:.2f}s] Unique behaviors so far ({len(ordered)}): {ordered}")
            last_unique_print_time = now


def main():
    # Set up webots robot wrapper.
    rb = robot.BRCW2()
    rb.init_devices()  # already called in __init__, but harmless
    run(rb) #start main control loop


if __name__ == "__main__":
    main()
