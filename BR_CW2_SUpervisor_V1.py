from controller import Supervisor
import math

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Setup receiver
receiver = supervisor.getDevice("receiver")
receiver.enable(timestep)
receiver.setChannel(1)  # must match robot emitter channel

epuck_counter = 0
max_spawn = 5  # maximum number of spawned robots

while supervisor.step(timestep) != -1:
    current_time = supervisor.getTime()

    while receiver.getQueueLength() > 0:
        message = receiver.getString()
        print(f"Received message: {message}")

        if message.strip().lower().startswith("spawn"):
            if epuck_counter >= max_spawn:
                print("Spawn limit reached. No more robots will be spawned.")
                receiver.nextPacket()
                continue

            parts = message.strip().split(":")
            if len(parts) == 2:
                sender_def = parts[1]
                target_robot = supervisor.getFromDef(sender_def)
            else:
                target_robot = supervisor.getFromDef("e-puck")

            if target_robot is not None:
                position = target_robot.getField("translation").getSFVec3f()
                rotation = target_robot.getField("rotation").getSFRotation()

                angle = rotation[3]
                forward_dx = 0.20 * math.sin(angle)  # 20 cm forward
                forward_dz = 0.20 * math.cos(angle)

                new_x = position[0] + forward_dx
                new_y = position[1]
                new_z = position[2] + forward_dz

                robot_name = f"E-Puck_{epuck_counter}"
                controller_name = "braitenberg"  # change as needed

                robot_proto = (
                    f'DEF {robot_name} E-puck {{ '
                    f'translation {new_x} {new_y} {new_z} '
                    f'rotation {rotation[0]} {rotation[1]} {rotation[2]} {rotation[3]} '
                    f'controller "{controller_name}" '
                    f'}}'
                )

                root = supervisor.getRoot()
                children_field = root.getField("children")
                children_field.importMFNodeFromString(-1, robot_proto)

                print(f"Spawned: {robot_name} in front of {target_robot.getDef()} due to message")
                epuck_counter += 1

            else:
                print(f"ERROR: Robot with DEF '{sender_def if len(parts) == 2 else 'e-puck'}' not found!")

        receiver.nextPacket()
