#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

right_arrow = cv2.imread('right1.png')
left_arrow = cv2.imread('left1.png')
star = cv2.imread('star1.png')

right_template = cv2.resize(right_arrow, (0,0), fx=0.3, fy=0.3) 
left_template = cv2.resize(left_arrow, (0,0), fx=0.3, fy=0.3) 
star_template = cv2.resize(star, (0,0), fx=0.3, fy=0.3) 

right_gray = cv2.cvtColor(right_template,cv2.COLOR_BGR2GRAY)
left_gray = cv2.cvtColor(left_template,cv2.COLOR_BGR2GRAY)
star_gray = cv2.cvtColor(star_template,cv2.COLOR_BGR2GRAY)

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

    red_top = (6*h/7)+6
    red_bot = red_top + 20
    mid = w / 2
    mask_red[0:red_top, 0:w] = 0
    mask_red[red_bot:h, 0:w] = 0
    mask_red[0:h, 0:mid-10] = 0
    mask_red[0:h, mid+10:w] = 0
    R = cv2.moments(mask_red)

    img = cv2.resize(image, (0,0), fx=0.5, fy=0.5)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if Y['m00'] > 0 and not self.STOP:
        if R['m00'] > 0 and not self.STOP: 
            right_res = cv2.matchTemplate(img_gray,right_gray,cv2.TM_CCOEFF_NORMED)
            left_res = cv2.matchTemplate(img_gray,left_gray,cv2.TM_CCOEFF_NORMED)
            star_res = cv2.matchTemplate(img_gray,star_gray,cv2.TM_CCOEFF_NORMED)
    
            min_valL, max_valL, min_loc, max_loc = cv2.minMaxLoc(resL)
            min_valR, max_valR, min_loc, max_loc = cv2.minMaxLoc(resR)
            min_valS, max_valS, min_loc, max_loc = cv2.minMaxLoc(resS)

            # print("min_valL: ", min_valL)
            # print("min_valR: ", min_valR)
            # print("min_valS: ", min_valS)

            if min_valL < -0.42 and min_valL > min_valR and -0.48 < min_valS < -0.4: 
                #print("LEFT")
                self.twist.linear.x = .45
                self.twist.angular.z = 0.3
                self.cmd_vel_pub.publish(self.twist)
                self.twist.linear.x = .45
                self.twist.angular.z = 0.3

            elif min_valR < -0.4 and min_valR > min_valL and -0.49 < min_valS < -0.44:
                #print("RIGHT")
                self.twist.linear.x = .45
                self.twist.angular.z = -0.2
                self.cmd_vel_pub.publish(self.twist)
                self.twist.linear.x = .45
                self.twist.angular.z = -0.2
                self.cmd_vel_pub.publish(self.twist)

            elif -0.43 < min_valS < -0.38 and min_valL < -0.42 and min_valR < -0.42:
                #print("STOP")
                self.stop = True 
                for i in range(0,150):
                  print(i)
                  self.twist.linear.x = 1
                  self.twist.angular.z = -1
                  self.cmd_vel_pub.publish(self.twist)
                self.twist.linear.x = 1
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
            else:
                self.twist.linear.x = 0.3
                self.cmd_vel_pub.publish(self.twist)
        else:
            if not self.stop:
                cx = int(Y['m10'] / Y['m00'])
                cy = int(Y['m01'] / Y['m00'])
                cv2.circle(image, (cx, cy), 20, (2, 166, 249), -1)
                err = cx - w / 2
                self.twist.linear.x = 0.2
                self.twist.angular.z = -float(err) / 100
                self.cmd_vel_pub.publish(self.twist)
            else:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)

    cv2.imshow("window", image)
    cv2.waitKey(3)
rospy.init_node('follower')
follower = Follower()
rospy.spin()
