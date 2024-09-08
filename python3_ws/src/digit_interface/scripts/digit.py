#!/opt/miniconda/envs/rosallegro/bin/python3
# first of all, you need to source the virtualenv. Then source the devel setup.bash file.

# ROS IMPORTS
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import cv2

# DIGIT LIBRARY
from digit_interface import Digit

'''
    setup_digit function: reads the sensor id from the launch file and initializes the sensor.
    Arguments:
        - None
    Returns: object class of the sensor library.
'''
def setup_digit():
    # Get sensor id from the launch file
    sensor_ids = rospy.get_param("/sensor_id")
    sensor_ids_list = sensor_ids.split(",")

    # Initialize the digit sensors and store them in a list
    digit_sensors = []
    for sensor_id in sensor_ids_list:
        digit_sensor = Digit(sensor_id)
        digit_sensor.connect()
        
        # Setting up sensor configurations
        configure_digit_sensor(digit_sensor)
        
        digit_sensors.append((digit_sensor, sensor_id))

    return digit_sensors

'''
    configure_digit_sensor function: Sets up the desired configurations for the digit sensors.
    Arguments:
        - digit_sensor: sensor object
    Returns: None
'''
def configure_digit_sensor(digit_sensor):
    # Set the LED intensity
    digit_sensor.set_intensity(Digit.LIGHTING_MAX)  # Adjust the intensity as needed
    
    # Set the resolution to QVGA (you can change to others like VGA)
    qvga_res = Digit.STREAMS["QVGA"]
    digit_sensor.set_resolution(qvga_res)
    
    # Set the FPS to 30 fps
    fps_30 = Digit.STREAMS["QVGA"]["fps"]["30fps"]
    digit_sensor.set_fps(fps_30)

'''
    main function: initialize the ros node and the publisher. Runs an infinite loop
        where reads images from the sensor and publish them in the topic.
    Arguments:
        - None
    Returns: None
'''
def main():
    # Initialize the digit sensors
    digit_sensors = setup_digit()

    # Initialize the ROS node
    rospy.init_node("digit_interface_node", anonymous=True)

    # Create a dictionary to store publishers for each sensor
    publishers = {}

    # Create publishers dynamically based on the sensor ids
    for digit_sensor, sensor_id in digit_sensors:
        publishers[sensor_id] = rospy.Publisher(f"{sensor_id}/camera/image_color", Image, queue_size=10)

    # CVBridge to transform the images from numpy to ROS type
    cvbridge = CvBridge()

    # Infinite loop
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing Digit images!")

        # Iterate over each digit sensor and publish images
        for digit_sensor, sensor_id in digit_sensors:
            # Get image from the sensor
            digit_img = digit_sensor.get_frame()

            # Create the ROS message for the image
            image_msg = Image()
            image_msg.header.stamp = rospy.Time.now()
            image_msg.header.frame_id = "digit_camera"
            image_msg.height = digit_img.shape[0]
            image_msg.width = digit_img.shape[1]
            image_msg.step = digit_img.strides[0]
            image_msg.data = digit_img.flatten().tolist()
            image_msg.encoding = "bgr8"

            # Publish the image message
            publishers[sensor_id].publish(image_msg)

        # Control the rate of publication
        rospy.Rate(300).sleep()  # Set this to the desired FPS

if __name__ == '__main__':
    main()
