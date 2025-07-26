from controller import Supervisor
import math

#creating supervisor instance
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Setup receiver
receiver = supervisor.getDevice("receiver")
receiver.enable(timestep)
receiver.setChannel(1)  #robot emitter channel = supervisor channel = 1

#spawn bookkeeping
epuck_counter = 0 #how many spawns so far
max_spawn = 5  # maximum number of spawned robots capped at 5 to prevent overcrowding. 

#main loop
while supervisor.step(timestep) != -1:
    current_time = supervisor.getTime()
    
    # Drain the receiver queue to stop old message pileup
    while receiver.getQueueLength() > 0:
        message = receiver.getString()
        print(f"Received message: {message}")
        
        # Expect messages that start with "SPAWN"
        if message.strip().lower().startswith("spawn"):
        
        #enforcing spawn limit set above
            if epuck_counter >= max_spawn:
                print("Spawn limit reached. No more robots will be spawned.")
                receiver.nextPacket()
                continue
                
            # Parse "SPAWN:DEF_NAME" -> parts = ["SPAWN", "DEF_NAME"]
            parts = message.strip().split(":")
            if len(parts) == 2:
                sender_def = parts[1]
                
                # Try to find the sender robot using its DEF name
                target_robot = supervisor.getFromDef(sender_def)
            else:
                #sender is the default main robot with DEF "e-puck"
                target_robot = supervisor.getFromDef("e-puck")

            if target_robot is not None:
            #computing spawns possition and orientation
                 # Get sender's possition and orientation to place the new robot 20 cm in front of it to prevent it from droping on top
                position = target_robot.getField("translation").getSFVec3f()
                rotation = target_robot.getField("rotation").getSFRotation()
                
                #[rx, ry, rz, angle] for spawn
                angle = rotation[3]
                forward_dx = 0.20 * math.sin(angle)  # 20 cm forward
                forward_dz = 0.20 * math.cos(angle)
                
                # Move 0.20 m forward (z in my case) in the robot's heading direction.
                
                new_x = position[0] + forward_dx
                new_y = position[1]
                new_z = position[2] + forward_dz

                # Building a unique name & controller for the new robot
                robot_name = f"E-Puck_{epuck_counter}"
                controller_name = "braitenberg" #add the controller for spawn robot. used a generic controller available.

                # Create the PROTO string to import
                robot_proto = (
                    f'DEF {robot_name} E-puck {{ '
                    f'translation {new_x} {new_y} {new_z} '
                    f'rotation {rotation[0]} {rotation[1]} {rotation[2]} {rotation[3]} '
                    f'controller "{controller_name}" '
                    f'}}'
                )

                # Inject the new node into the scene tree
                root = supervisor.getRoot()
                children_field = root.getField("children")
                
                 # -1 appends to the end of the children list
                children_field.importMFNodeFromString(-1, robot_proto)

                print(f"Spawned: {robot_name} in front of {target_robot.getDef()} due to message")
                epuck_counter += 1

            else:
                print(f"ERROR: Robot with DEF '{sender_def if len(parts) == 2 else 'e-puck'}' not found!")

        # Move to the next packet in the receiver queu
        receiver.nextPacket()
