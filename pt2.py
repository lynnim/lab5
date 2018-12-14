#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

arrowL = cv2.imread('left1.png')
arrowR = cv2.imread('right1.png')
star = cv2.imread('star1.png')

template_arrowL = cv2.resize(arrowL, (0,0), fx=0.3, fy=0.3) 
template_arrowR = cv2.resize(arrowR, (0,0), fx=0.3, fy=0.3) 
template_star = cv2.resize(star, (0,0), fx=0.3, fy=0.3) 

gray_left = cv2.cvtColor(template_arrowL,cv2.COLOR_BGR2GRAY)
gray_right = cv2.cvtColor(template_arrowR,cv2.COLOR_BGR2GRAY)
gray_star = cv2.cvtColor(template_star,cv2.COLOR_BGR2GRAY)


class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    self.twist = Twist()

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_yellow = numpy.array([19, 100, 100])
    upper_yellow = numpy.array([39, 255, 255])
    y_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    lower_red = numpy.array([0, 100, 100])
    upper_red = numpy.array([10, 255, 255])
    r_mask = cv2.inRange(hsv, lower_red, upper_red)

    h, w, d = image.shape

    #for the yellow
    search_top = 5 * h / 6  # put the frame 5/6 of the way down
    search_bot = search_top + 20  # use the 20 units in front of the robot from that 5/6 of the way down
    y_mask[0:search_top, 0:w] = 0
    y_mask[search_bot:h, 0:w] = 0

    r_top = 4 * h / 5  # put the frame 5/6 of the way down
    r_bot = r_top + 20  # use the 20 units in front of the robot from that 5/6 of the way down
    r_mask[0:r_top, 0:w] = 0
    r_mask[r_bot:h, 0:w] = 0
    R = cv2.moments(r_mask)

    img = cv2.resize(image, (0,0), fx=0.5, fy=0.5)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    M = cv2.moments(y_mask)
    if M['m00'] > 0:
        if R['m00'] > 0:
            res_arrowL = cv2.matchTemplate(img_gray,gray_left,cv2.TM_CCOEFF_NORMED)
            res_arrowR = cv2.matchTemplate(img_gray,gray_right,cv2.TM_CCOEFF_NORMED)
            res_star = cv2.matchTemplate(img_gray,gray_star,cv2.TM_CCOEFF_NORMED)
    
            min_valL, max_valL, min_loc, max_loc = cv2.minMaxLoc(res_arrowL)
            min_valR, max_valR, min_loc, max_loc = cv2.minMaxLoc(res_arrowR)
            min_valS, max_valS, min_loc, max_loc = cv2.minMaxLoc(res_star)

            print("min_valL: ", min_valL)
            print("min_valR: ", min_valR)
            print("min_valS: ", min_valS)

            # r_cx = int(R['m10'] / R['m00'])
            # r_cy = int(R['m01'] / R['m00'])
            # cv2.circle(image, (r_cx, r_cy), 10, (225, 0, 0), -1)
            # print("RED DETECTED")
            # self.twist.linear.x = .2
            # self.twist.angular.z = -.06
            # self.cmd_vel_pub.publish(self.twist)

        else:
            print("MOVE FORWARD")
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            # BEGIN CONTROL
            err = cx - w / 2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
