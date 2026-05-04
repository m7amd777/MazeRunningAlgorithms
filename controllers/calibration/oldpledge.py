"""Pledge algorithm maze solver for e-puck."""

from controller import Robot

NORTH = 0
EAST  = 1
SOUTH = 2
WEST  = 3

DIR_VECTORS = {
    NORTH: (0,  1),
    EAST:  (1,  0),
    SOUTH: (0, -1),
    WEST:  (-1, 0),
}


def run_robot(robot):
    timestep  = int(robot.getBasicTimeStep())
    max_speed = 6.28

    left_motor  = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    prox_sensors = []
    for i in range(8):
        s = robot.getDevice(f'ps{i}')
        s.enable(timestep)
        prox_sensors.append(s)

    # -------------------------
    # Tune these
    # -------------------------
    PAUSE_STEPS          = 30
    WALL_THRESHOLD_FRONT = 80
    WALL_THRESHOLD_SIDE  = 80
    TURN_STEPS_90        = 32
    TURN_STEPS_180       = 68
    FORWARD_STEPS_CELL   = 35
    TURN_SPEED           = 0.6 * max_speed
    FORWARD_SPEED        = 0.7 * max_speed

    # -------------------------
    # Pledge state
    # -------------------------
    PREFERRED_HEADING = NORTH
    heading      = NORTH
    pledge_mode  = "PREFERRED"  # "PREFERRED" or "WALL_FOLLOW"
    turn_counter = 0            # net right turns during wall-following
                                # right=+1, left=-1, back=+2

    # -------------------------
    # Motion state
    # -------------------------
    state        = "SENSE"
    action       = None
    action_steps = 0

    def stop():
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)

    def pause(steps):
        stop()
        for _ in range(steps):
            robot.step(timestep)

    def get_openings():
        left_blocked  = prox_sensors[5].getValue() > WALL_THRESHOLD_SIDE
        front_blocked = (prox_sensors[7].getValue() > WALL_THRESHOLD_FRONT or
                         prox_sensors[0].getValue() > WALL_THRESHOLD_FRONT)
        right_blocked = prox_sensors[2].getValue() > WALL_THRESHOLD_SIDE
        return not left_blocked, not front_blocked, not right_blocked

    def choose_move(left_open, front_open, right_open):
        nonlocal pledge_mode, turn_counter

        # Exit wall-follow if we've netted zero turns facing the preferred direction
        if pledge_mode == "WALL_FOLLOW" and turn_counter == 0 and heading == PREFERRED_HEADING:
            pledge_mode = "PREFERRED"
            print(">> Exiting wall-follow (back to preferred heading, net turns = 0)")

        if pledge_mode == "PREFERRED":
            if front_open:
                return "FRONT"
            # Wall ahead in preferred direction — start wall following
            pledge_mode = "WALL_FOLLOW"
            print(">> Entering wall-follow")

        # WALL_FOLLOW: right-hand rule
        # Priority: right > front > left > back
        if right_open:
            turn_counter += 1
            return "RIGHT"
        elif front_open:
            return "FRONT"
        elif left_open:
            turn_counter -= 1
            return "LEFT"
        else:
            turn_counter += 2
            return "BACK"

    def start_action(move):
        nonlocal state, action, action_steps
        if move == "LEFT":
            action = "TURN_LEFT"
            action_steps = TURN_STEPS_90
        elif move == "RIGHT":
            action = "TURN_RIGHT"
            action_steps = TURN_STEPS_90
        elif move == "BACK":
            action = "TURN_BACK"
            action_steps = TURN_STEPS_180
        else:
            action = "FORWARD"
            action_steps = FORWARD_STEPS_CELL
        state = "MOVE"

    def update_heading(turn_action):
        nonlocal heading
        if turn_action == "TURN_LEFT":
            heading = (heading - 1) % 4
        elif turn_action == "TURN_RIGHT":
            heading = (heading + 1) % 4
        elif turn_action == "TURN_BACK":
            heading = (heading + 2) % 4

    while robot.step(timestep) != -1:

        if state == "SENSE":
            left_open, front_open, right_open = get_openings()
            print(f"Heading:{heading}  Mode:{pledge_mode}  Counter:{turn_counter} | L:{left_open} F:{front_open} R:{right_open}")

            move = choose_move(left_open, front_open, right_open)
            print("Move:", move)
            start_action(move)

        elif state == "MOVE":
            if action == "TURN_LEFT":
                left_motor.setVelocity(-TURN_SPEED)
                right_motor.setVelocity(TURN_SPEED)
            elif action == "TURN_RIGHT":
                left_motor.setVelocity(TURN_SPEED)
                right_motor.setVelocity(-TURN_SPEED)
            elif action == "TURN_BACK":
                left_motor.setVelocity(TURN_SPEED)
                right_motor.setVelocity(-TURN_SPEED)
            elif action == "FORWARD":
                left_motor.setVelocity(FORWARD_SPEED)
                right_motor.setVelocity(FORWARD_SPEED)

            action_steps -= 1

            if action_steps <= 0:
                if action in ("TURN_LEFT", "TURN_RIGHT", "TURN_BACK"):
                    update_heading(action)
                    action = "FORWARD"
                    action_steps = FORWARD_STEPS_CELL
                elif action == "FORWARD":
                    pause(PAUSE_STEPS)
                    state = "SENSE"

    stop()


if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
