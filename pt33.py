#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Get template images 
right_triangle = cv2.imread('right1.png')
left_triangle = cv2.imread('left1.png')
star = cv2.imread('star1.png')

templateL = cv2.resize(left_triangle, (0,0), fx=0.5, fy=0.5) 
templateR = cv2.resize(right_triangle, (0,0), fx=0.4, fy=0.4) 
templateS = cv2.resize(star, (0,0), fx=0.4, fy=0.4) 

grayR = cv2.cvtColor(templateR,cv2.COLOR_BGR2GRAY)
grayL = cv2.cvtColor(templateL,cv2.COLOR_BGR2GRAY)
grayStar = cv2.cvtColor(templateS,cv2.COLOR_BGR2GRAY)

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
    y_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    lower_red = numpy.array([0, 100, 100])
    upper_red = numpy.array([10, 255, 255])
    r_mask = cv2.inRange(hsv, lower_red, upper_red)

    search_top = 5 * h / 6  
    search_bot = search_top + 20  
    mid = w/2
    y_mask[0:search_top, 0:w] = 0
    y_mask[search_bot:h, 0:w] = 0
    Y = cv2.moments(y_mask)

    #for the red
    r_top = (6*h/7)+6
    r_bot = r_top + 20
    mid = w / 2
    r_mask[0:r_top, 0:w] = 0
    r_mask[r_bot:h, 0:w] = 0
    r_mask[0:h, 0:mid-10] = 0
    r_mask[0:h, mid+10:w] = 0
    R = cv2.moments(r_mask)

    # Resize image 
    img = cv2.resize(image, (0,0), fx=0.5, fy=0.5)
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if Y['m00'] > 0 and not self.stop:

        if R['m00'] > 0 and not self.stop: 
            resL = cv2.matchTemplate(imgGray,grayL,cv2.TM_CCOEFF_NORMED)
            resR = cv2.matchTemplate(imgGray,grayR,cv2.TM_CCOEFF_NORMED)
            resS = cv2.matchTemplate(imgGray,grayStar,cv2.TM_CCOEFF_NORMED)
    
            # Store the coordinates of matched area in a numpy array 
            min_valL, max_valL, min_loc, max_loc = cv2.minMaxLoc(resL)
            min_valR, max_valR, min_loc, max_loc = cv2.minMaxLoc(resR)
            min_valS, max_valS, min_loc, max_loc = cv2.minMaxLoc(resS)

            print("LEFT: " + str(min_valL))
            print("RIGHT: " + str(min_valR))
            print("STAR: " + str(min_valS))

            # if min_valL < -0.81 and min_valL < min_valR and min_valS > -0.05: 
            #     print("Turning left")
            #     self.twist.linear.x = .45
            #     self.twist.angular.z = 0.3
            #     self.cmd_vel_pub.publish(self.twist)
            #     self.twist.linear.x = .45
            #     self.twist.angular.z = 0.3
            # elif min_valR < -0.81 and min_valR < min_valL and min_valS > -0.05:
            #     print("Turning right")
            #     self.twist.linear.x = .45
            #     self.twist.angular.z = -0.2
            #     self.cmd_vel_pub.publish(self.twist)
            #     self.twist.linear.x = .45
            #     self.twist.angular.z = -0.2
            #     self.cmd_vel_pub.publish(self.twist)
            # elif min_valS < -0.05 and min_valL < -0.81 and min_valR < -0.83:
            #     print("Stopping")
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
            #     print("Moved foward a bit ")
            #     self.twist.linear.x = 0.3
            #     self.cmd_vel_pub.publish(self.twist)
                # END CONTROL
        else:
            if not self.stop:
                print("follow yellow")
                #calculate the centriod
                cx = int(Y['m10'] / Y['m00'])
                cy = int(Y['m01'] / Y['m00'])
                cv2.circle(image, (cx, cy), 20, (2, 166, 249), -1)
                # BEGIN CONTROL
                err = cx - w / 2
                self.twist.linear.x = 0.2
                self.twist.angular.z = -float(err) / 100
                self.cmd_vel_pub.publish(self.twist)
                # END CONTROL
            else:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)

    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
