"""Wall follower for e-puck + red stop via camera."""

from controller import Robot


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

    camera = robot.getDevice('camera')
    camera.enable(timestep)
    cam_w = camera.getWidth()
    cam_h = camera.getHeight()

    # -------------------------
    # Tune these
    # -------------------------
    RED_PIXEL_THRESHOLD = 2000   # red pixels in frame to trigger stop
    RED_MIN             = 150    # r channel floor
    GREEN_MAX           = 80     # g channel ceiling
    BLUE_MAX            = 80     # b channel ceiling

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

    while robot.step(timestep) != -1:

        # --- Red check first ---
        reds = red_pixel_count()
        if reds >= RED_PIXEL_THRESHOLD:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            print(f"RED DETECTED — {reds} pixels. Stopped.")
            continue

        # --- Wall follower logic ---
        left_wall   = prox_sensors[5].getValue() > 80
        left_corner = prox_sensors[6].getValue() > 80
        front_wall  = prox_sensors[7].getValue() > 80

        left_speed  = max_speed
        right_speed = max_speed

        if front_wall:
            print("Turn right in place")
            left_speed  =  max_speed
            right_speed = -max_speed
        else:
            if left_wall:
                print("Move forward")
                left_speed  = max_speed
                right_speed = max_speed
            else:
                print("Turn left")
                left_speed  = max_speed / 4
                right_speed = max_speed
            if left_corner:
                print("Too close — drive right")
                left_speed  = max_speed
                right_speed = max_speed / 8

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
