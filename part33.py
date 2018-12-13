#!/usr/bin/env python
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
    self.STOP = False

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    h, w, d = image.shape

    lower_yellow = numpy.array([19, 100, 100])
    upper_yellow = numpy.array([39, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    lower_red = numpy.array([0, 100, 100])
    upper_red = numpy.array([10, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red, upper_red)

    search_top = 5 * h / 6  
    search_bot = search_top + 20  
    mid = w/2
    mask_yellow[0:search_top, 0:w] = 0
    mask_yellow[search_bot:h, 0:w] = 0
    Y = cv2.moments(mask_yellow)

    top_red = (6 * h / 7)+6
    bot_red = top_red + 20
    mid = w / 2
    mask_red[0:top_red, 0:w] = 0
    mask_red[bot_red:h, 0:w] = 0
    mask_red[0:h, 0:mid-10] = 0
    mask_red[0:h, mid+10:w] = 0
    R = cv2.moments(mask_red)

    img = cv2.resize(image, (0,0), fx=0.5, fy=0.5)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
 
    if Y['m00'] > 0 and not self.STOP:
        if R['m00'] > 0 and not self.STOP: 
            res_arrowL = cv2.matchTemplate(img_gray,gray_left,cv2.TM_CCOEFF_NORMED)
            res_arrowR = cv2.matchTemplate(img_gray,gray_right,cv2.TM_CCOEFF_NORMED)
            res_star = cv2.matchTemplate(img_gray,gray_star,cv2.TM_CCOEFF_NORMED)
    
            min_valL, max_valL, min_loc, max_loc = cv2.minMaxLoc(res_arrowL)
            min_valR, max_valR, min_loc, max_loc = cv2.minMaxLoc(res_arrowR)
            min_valS, max_valS, min_loc, max_loc = cv2.minMaxLoc(res_star)

            print("min_valL: ", min_valL)
            print("min_valR: ", min_valR)
            print("min_valS: ", min_valS)

            # if min_valL < -0.81 and min_valL < min_valR and min_valS > -0.05: 
            #     print("LEFT")
            #     self.twist.linear.x = .45
            #     self.twist.angular.z = 0.3
            #     self.cmd_vel_pub.publish(self.twist)
            #     self.twist.linear.x = .45
            #     self.twist.angular.z = 0.3
            
            # elif min_valR < -0.81 and min_valR < min_valL and min_valS > -0.05:
            #     print("RIGHT")
            #     self.twist.linear.x = .45
            #     self.twist.angular.z = -0.2
            #     self.cmd_vel_pub.publish(self.twist)
            #     self.twist.linear.x = .45
            #     self.twist.angular.z = -0.2
            #     self.cmd_vel_pub.publish(self.twist)
            
            # elif min_valS < -0.05 and min_valL < -0.81 and min_valR < -0.83:
            #     print("STOP")
            #     self.stop = True 
            #     for i in range(0,150):
            #       print(i)
            #       self.twist.linear.x = 1
            #       self.twist.angular.z = -1
            #       self.cmd_vel_pub.publish(self.twist)
            #     self.twist.linear.x = 1
            #     self.twist.angular.z = 0
            #     self.cmd_vel_pub.publish(self.twist)
            
            # else:
            #     self.twist.linear.x = 0.3
            #     self.cmd_vel_pub.publish(self.twist)

    else:
        if not self.STOP:
            cx = int(Y['m10'] / Y['m00'])
            cy = int(Y['m01'] / Y['m00'])
            cv2.circle(image, (cx, cy), 6, (0,0,255), -1)
            err = cx - w / 2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
        else:
            print("MOVING FORWARD")
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)

    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
