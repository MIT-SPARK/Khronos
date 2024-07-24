import rospy
import cv2
import csv
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge

# Global variables to store depth data and CSV writer
csv_writer = None
image_data = []


def callback(rgb_msg, depth_msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        cv_depth = bridge.imgmsg_to_cv2(depth_msg, "32FC1")
    except CvBridge.CvBridgeError as e:
        print(e)

    # Save the data
    image_data.append((cv_image, cv_depth))


def spin():
    rgb_img, depth_img = image_data.pop(0)
    timestamp = rgb_img.header.stamp.to_nsec()

    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Get depth value at (x, y)
            depth = depth_img[y, x]

            # Convert (x, y, depth) to 3D point in camera reference frame
            # Requires camera intrinsic parameters
            x_camera = (x - cx) * depth / fx
            y_camera = (y - cy) * depth / fy
            z_camera = depth

            # Write data to CSV
            csv_writer.writerow([timestamp, x_world, y_world, z_world])

    cv2.setMouseCallback("Image Window", click_event)

    # Display the RGB image
    cv2.imshow("Image Window", rgb_img)
    cv2.waitKey(1)


def main():
    rospy.init_node("image_listener")
    global csv_writer
    csv_file = open("output.csv", "w")
    csv_writer = csv.writer(csv_file)

    # Subscribe to image and depth topics
    rgb_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
    depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)

    ats = message_filters.ApproximateTimeSynchronizer(
        [rgb_sub, depth_sub], queue_size=10, slop=0.5
    )
    ats.registerCallback(callback)

    # Setup OpenCV window and mouse callback
    cv2.namedWindow("Image Window")

    # Spin ROS
    rospy.spin()

    while not rospy.is_shutdown():
        spin()

    cv2.destroyAllWindows()
    csv_file.close()


if __name__ == "__main__":
    main()
