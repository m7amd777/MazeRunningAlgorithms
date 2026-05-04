"""Pledge algorithm maze solver for e-puck — left-hand wall rule + red stop."""

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

    left_motor  = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    prox_sensors = []
    for i in range(8):
        s = robot.getDevice(f'ps{i}')
        s.enable(timestep)
        prox_sensors.append(s)

    camera = robot.getDevice('camera')
    camera.enable(timestep)
    cam_w = camera.getWidth()
    cam_h = camera.getHeight()

    # -------------------------
    # Tune these
    # -------------------------
    PAUSE_STEPS          = 30
    WALL_THRESHOLD_FRONT = 80
    WALL_THRESHOLD_SIDE  = 75
    TURN_STEPS_90        = 32
    TURN_STEPS_180       = 68
    FORWARD_STEPS_CELL   = 35
    TURN_SPEED           = 0.6 * max_speed
    FORWARD_SPEED        = 0.7 * max_speed

    RED_PIXEL_THRESHOLD  = 2000   # red pixels in frame to trigger stop
    RED_MIN              = 150    # r channel floor
    GREEN_MAX            = 80     # g channel ceiling
    BLUE_MAX             = 80     # b channel ceiling

    # -------------------------
    # Pledge state
    # -------------------------
    PREFERRED_HEADING  = NORTH
    heading            = NORTH
    pledge_mode        = "PREFERRED"
    turn_counter       = 0       # left=-1, right=+1, back=+2
    wall_follow_moves  = 0

    # Position tracking for loop detection
    pos                = [0, 0]
    wall_follow_seen   = set()   # (x, y, heading) visited during wall-follow

    # -------------------------
    # Motion state
    # -------------------------
    state        = "SENSE"
    action       = None
    action_steps = 0

    def stop():
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)

    def red_pixel_count():
        img = camera.getImage()
        count = 0
        for y in range(cam_h):
            for x in range(cam_w):
                if (camera.imageGetRed(img, cam_w, x, y)   > RED_MIN and
                        camera.imageGetGreen(img, cam_w, x, y) < GREEN_MAX and
                        camera.imageGetBlue(img, cam_w, x, y)  < BLUE_MAX):
                    count += 1
        return count

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
        nonlocal pledge_mode, turn_counter, wall_follow_moves

        # Exit wall-follow: counter=0, facing preferred, and robot has moved at
        # least 2 cells since entering (prevents immediate exit-reentry)
        if (pledge_mode == "WALL_FOLLOW"
                and wall_follow_moves >= 2
                and turn_counter == 0
                and heading == PREFERRED_HEADING):
            pledge_mode = "PREFERRED"
            wall_follow_moves = 0
            wall_follow_seen.clear()
            print(f">> Exiting wall-follow | counter={turn_counter} heading={heading}")

        if pledge_mode == "PREFERRED":
            if front_open:
                return "FRONT"
            pledge_mode = "WALL_FOLLOW"
            wall_follow_moves = 0
            wall_follow_seen.clear()
            print(">> Entering wall-follow")

        # LEFT-hand rule: priority left > front > right > back
        # Counter: left=-1, right=+1, back=+2
        # The right-hand rule was causing clockwise perimeter loops because
        # right was always available at outer corners. Left-hand rule forces
        # the robot to take left-branch corridors instead.
        wall_follow_moves += 1

        if left_open:
            turn_counter -= 1
            return "LEFT"
        elif front_open:
            return "FRONT"
        elif right_open:
            turn_counter += 1
            return "RIGHT"
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

        reds = red_pixel_count()
        if reds >= RED_PIXEL_THRESHOLD:
            stop()
            print(f"RED DETECTED — {reds} pixels. Stopped.")
            continue

        if state == "SENSE":
            left_open, front_open, right_open = get_openings()
            print(f"Pos:{tuple(pos)} Heading:{heading}  Mode:{pledge_mode}  Counter:{turn_counter}  WFmoves:{wall_follow_moves} | L:{left_open} F:{front_open} R:{right_open}")

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
                    dx, dy = DIR_VECTORS[heading]
                    pos[0] += dx
                    pos[1] += dy

                    # Loop detection: same cell+heading seen twice in wall-follow
                    # means Pledge cannot escape — rotate preferred direction 90°
                    if pledge_mode == "WALL_FOLLOW":
                        key = (pos[0], pos[1], heading)
                        if key in wall_follow_seen:
                            PREFERRED_HEADING = (PREFERRED_HEADING + 1) % 4
                            turn_counter      = 0
                            wall_follow_moves = 0
                            wall_follow_seen.clear()
                            pledge_mode = "PREFERRED"
                            print(f">> Loop detected! Rotating preferred heading to {PREFERRED_HEADING}")
                        else:
                            wall_follow_seen.add(key)

                    pause(PAUSE_STEPS)
                    state = "SENSE"

    stop()


if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
