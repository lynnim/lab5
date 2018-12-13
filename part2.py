#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    self.twist = Twist()
    self.stop = False

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

    lower_green = numpy.array([50, 100, 100])
    upper_green = numpy.array([70, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    lower_blue = numpy.array([110, 100, 100])
    upper_blue = numpy.array([130, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    search_top = 5 * h / 6 
    search_bot = search_top + 20 
    mid = w/2
    mask_yellow[0:search_top, 0:w] = 0
    mask_yellow[search_bot:h, 0:w] = 0
    Y = cv2.moments(mask_yellow)

    top_red = (6*h/7)+6
    bot_red = top_red + 20
    mid = w / 2
    mask_red[0:top_red, 0:w] = 0
    mask_red[bot_red:h, 0:w] = 0
    mask_red[0:h, 0:mid-10] = 0
    mask_red[0:h, mid+10:w] = 0
    R = cv2.moments(mask_red)

    top_green = 4 * h / 5
    bot_green = top_green + 30
    mask_green[0:top_green, 0:w] = 0
    mask_green[bot_green:h, 0:w] = 0
    G = cv2.moments(mask_green)

    top_blue = 4 * h / 5
    bot_blue = top_blue + 30
    mask_blue[0:top_blue, 0:w] = 0
    mask_blue[bot_blue:h, 0:w] = 0
    B = cv2.moments(mask_blue)

    if Y['m00'] > 0:
      if G['m00'] > 0:
        cx_green = int(G['m10'] / G['m00'])
        cy_green = int(G['m01'] / G['m00'])
        #cv2.circle(image, (cx_green, cy_green), 6, (0, 225, 0), -1)
        cv2.circle(image, (cx_green, cy_green), 6, (0, 0, 255), -1)
        #print("LEFT")
        self.twist.linear.x = .45
        self.twist.angular.z = 0.2
        self.cmd_vel_pub.publish(self.twist)

      elif B['m00'] > 0:
        cx_blue = int(B['m10'] / B['m00'])
        cy_blue = int(B['m01'] / B['m00'])
        cv2.circle(image, (cx_blue, cy_blue), 6, (0, 0, 255), -1)
        #cv2.circle(image, (cx_blue, cy_blue), 6, (225, 0, 0), -1)
        #print("RIGHT")
        self.twist.linear.x = .45
        self.twist.angular.z = -0.2
        self.cmd_vel_pub.publish(self.twist)

      elif R['m00'] > 0:
          cx_red = int(R['m10'] / R['m00'])
          cy_red = int(R['m01'] / R['m00'])
          cv2.circle(image, (cx_red, cy_red), 6, (0, 0, 225), -1)
          err = cx_red - w / 2
          #print("STOP")
          self.twist.linear.x = 0.5
          self.twist.angular.z = -float(err) / 100
          self.cmd_vel_pub.publish(self.twist)
          self.stop = True

      else:
          if self.stop:
              #print("stopping")
              for i in range(0,130):
                  print(i)
                  self.twist.linear.x = 1
                  self.twist.angular.z = -1
                  self.cmd_vel_pub.publish(self.twist)

              self.twist.linear.x = 0
              self.twist.angular.z = 0
              self.cmd_vel_pub.publish(self.twist)
          else:
              cx = int(Y['m10'] / Y['m00'])
              cy = int(Y['m01'] / Y['m00'])
              cv2.circle(image, (cx, cy), 6, (0, 0, 225), -1)
              #cv2.circle(image, (cx, cy), 6, (2, 166, 249), -1)
              err = cx - w / 2
              self.twist.linear.x = 0.55
              self.twist.angular.z = -float(err) / 100
              self.cmd_vel_pub.publish(self.twist)
    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
