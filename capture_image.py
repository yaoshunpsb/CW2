# Import Webots controller classes needed for the robot
# Robot = main robot controller
# Motor = control wheel motors
# DistanceSensor = read obstacle sensors
# Camera = access robot camera
from controller import Robot, Motor, DistanceSensor, Camera


# ================= CONSTANTS =================

# Maximum wheel speed of e-puck robot
MAX_SPEED = 6.28

# Multiplier used to slow the robot down
# Robot will move at 50% of its maximum speed
MULTIPLIER = 0.5

# Distance threshold for detecting an obstacle
# If sensor reading exceeds this value, robot assumes obstacle ahead
OBSTACLE_DISTANCE = 0.02


# Dog RGB colour measured from the camera
# These values were obtained from console output
R = 90
G = 76
B = 66

# Allowed difference between detected RGB and target RGB
# Helps handle lighting variation in Webots
TOLERANCE = 20


# ================= DISTANCE SENSORS =================

def get_distance_values(distance_sensors, distance_values):

    # Loop through all 8 proximity sensors
    for i in range(8):

        # Read sensor value
        # Raw value is divided by 4096 to normalize between 0 and 1
        val = distance_sensors[i].getValue() / 4096.0

        # Ensure the value does not exceed 1
        distance_values[i] = min(val, 1.0)


def front_obstacle(distance_values):

    # Calculate average reading from front sensors (ps0 and ps7)
    avg = (distance_values[0] + distance_values[7]) / 2.0

    # Return True if obstacle is detected in front
    return avg > OBSTACLE_DISTANCE


# ================= MOVEMENT =================

def move_forward(left_motor, right_motor):

    # Set both motors to move forward
    left_motor.setVelocity(MAX_SPEED * MULTIPLIER)
    right_motor.setVelocity(MAX_SPEED * MULTIPLIER)


def move_backward(left_motor, right_motor, robot, timestep):

    # Set motors to move backwards
    left_motor.setVelocity(-MAX_SPEED * MULTIPLIER)
    right_motor.setVelocity(-MAX_SPEED * MULTIPLIER)

    # Record start time
    start = robot.getTime()

    # Continue stepping simulation for 0.3 seconds
    while robot.getTime() < start + 0.3:
        robot.step(timestep)


def turn_left(left_motor, right_motor, robot, timestep):

    # Rotate robot left by spinning wheels in opposite directions
    left_motor.setVelocity(-MAX_SPEED * MULTIPLIER)
    right_motor.setVelocity(MAX_SPEED * MULTIPLIER)

    # Record start time
    start = robot.getTime()

    # Continue stepping simulation for 0.3 seconds
    while robot.getTime() < start + 0.3:
        robot.step(timestep)


# ================= CAMERA =================

def get_center_rgb(camera):

    # Get camera resolution
    width = camera.getWidth()
    height = camera.getHeight()

    # Get raw image data from camera
    image = camera.getImage()

    # Calculate center pixel coordinates
    x = int(width / 2)
    y = int(height / 2)

    # Extract RGB values from the center pixel
    r = camera.imageGetRed(image, width, x, y)
    g = camera.imageGetGreen(image, width, x, y)
    b = camera.imageGetBlue(image, width, x, y)

    # Return RGB values
    return r, g, b


# ================= DOG DETECTION =================

def is_dog(r, g, b):

    # Compare detected RGB values with dog colour
    # abs() ensures the difference is within tolerance
    if (abs(r - R) < TOLERANCE and
        abs(g - G) < TOLERANCE and
        abs(b - B) < TOLERANCE):

        return True

    # Otherwise return False
    return False


# ================= IMAGE CAPTURE =================

def capture_image(camera, image_id):

    # Create filename for captured image
    filename = f"capture_{image_id}.png"

    # Save camera image with 100% quality
    camera.saveImage(filename, 100)

    # Print confirmation message
    print("Image saved:", filename)


# ================= MAIN ROBOT =================

def run_robot(robot):

    # Get Webots timestep
    timestep = int(robot.getBasicTimeStep())


    # -------- Distance sensors setup --------

    # Names of the 8 proximity sensors
    sensor_names = ["ps0","ps1","ps2","ps3","ps4","ps5","ps6","ps7"]

    # List to store sensor objects
    distance_sensors = []

    # List to store sensor values
    distance_values = [0.0]*8

    # Initialize each sensor
    for name in sensor_names:

        sensor = robot.getDevice(name)
        sensor.enable(timestep)

        distance_sensors.append(sensor)


    # -------- Camera setup --------

    # Get camera device
    camera = robot.getDevice("camera")

    # Enable camera
    camera.enable(timestep)


    # -------- Motor setup --------

    # Get left and right motors
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")

    # Set motors to velocity control mode
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    # Initially stop the robot
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)


    # -------- Image capture variables --------

    # Counter for saved images
    image_counter = 0

    # Boolean flag to prevent repeated capture
    captured = False


    # ================= MAIN LOOP =================

    while robot.step(timestep) != -1:

        # Read distance sensor values
        get_distance_values(distance_sensors, distance_values)

        # Get RGB value from center camera pixel
        r, g, b = get_center_rgb(camera)

        # Print RGB values for debugging
        print("RGB:", r, g, b)


        # ===== OBJECT DETECTED =====

        if is_dog(r, g, b):

            print("DETECTED")

            # Stop robot
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)

            # Capture image only once
            if not captured:

                capture_image(camera, image_counter)

                image_counter += 1
                captured = True

        else:

            # Reset capture flag if object not detected
            captured = False


            # -------- Obstacle avoidance --------

            if front_obstacle(distance_values):

                move_backward(left_motor, right_motor, robot, timestep)
                turn_left(left_motor, right_motor, robot, timestep)

            else:

                move_forward(left_motor, right_motor)


# ================= ENTRY =================

# Python main entry point
if __name__ == "__main__":

    # Create robot instance
    my_robot = Robot()

    # Start robot controller
    run_robot(my_robot)