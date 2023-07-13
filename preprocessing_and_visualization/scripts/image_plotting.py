import matplotlib.pyplot as plt

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImagePlotter:
    def __init__(self, topic_image):
        # Set the Image callback
        self.cv_bridge = CvBridge()
        rospy.Subscriber(topic_image, Image, self.callback_image)

        # Run the image plotter
        self.image = None
        self.plot_image()

    def callback_image(self, message_image):
        # Convert the image message to an OpenCV image
        self.image = self.cv_bridge.imgmsg_to_cv2(message_image, desired_encoding='mono8')

    def plot_image(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.image is not None:
                plt.figure()
                plt.imshow(self.image, cmap='gray', vmin=0, vmax=255)
                plt.show()
                rate.sleep()

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('image_plotting')

    # Process argument
    topic_image_plot = rospy.get_param('~topic_image_plot', default='')
    if topic_image_plot == '':
        exit()

    # Create image plotter
    image_plotter = ImagePlotter(topic_image_plot)

    # Process the callbacks
    rospy.spin()
