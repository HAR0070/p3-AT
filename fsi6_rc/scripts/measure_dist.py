#!/usr/bin/env python3
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import roslib
roslib.load_manifest('cam_distance1')
import sys
import math




#Install Imutills di CMD
import imutils

#Import OpenCV
import cv2
import rospy


def callback(data):
        rospy.loginfo("node has been started:1")
        try:
            br = CvBridge()
            get_frame = br.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            print(e)
        frame= cv2.resize(get_frame, None, fx=1, fy=1, interpolation=cv2.INTER_AREA)
        orig = get_frame[:1080,0:1920]
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(frame, (15, 15), 0)
        thresh = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,2)
        kernel = np.ones((3,3),np.uint8)
        closing = cv2.morphologyEx(thresh,cv2.MORPH_CLOSE,kernel,iterations=3)

        result_img = closing.copy()
        contours,hierachy = cv2.findContours(result_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        hitung_objek = 0

                    
                    
        pixelsPerMetric = None
        for cnt in contours:
            area = cv2.contourArea(cnt)

                        
            if area < 1000 or area > 120000:
                continue

                        
            orig = frame.copy()
            box = cv2.minAreaRect(cnt)
            box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
            box = np.array(box, dtype="int")
            box = perspective.order_points(box)
            cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 64), 2)
            for (x, y) in box:
                cv2.circle(orig, (int(x), int(y)), 5, (0, 255, 64), -1)

                        
            (tl, tr, br, bl) = box
            (tltrX, tltrY) = midpoint(tl, tr)
            (blbrX, blbrY) = midpoint(bl, br)
            (tlblX, tlblY) = midpoint(tl, bl)
            (trbrX, trbrY) = midpoint(tr, br)

                        
            cv2.circle(orig, (int(tltrX), int(tltrY)), 0, (0, 255, 64), 5)
            cv2.circle(orig, (int(blbrX), int(blbrY)), 0, (0, 255, 64), 5)
            cv2.circle(orig, (int(tlblX), int(tlblY)), 0, (0, 255, 64), 5)
            cv2.circle(orig, (int(trbrX), int(trbrY)), 0, (0, 255, 64), 5)

                        
            cv2.line(orig, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)),
                                (255, 0, 255), 2)
            cv2.line(orig, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)),
                                (255, 0, 255), 2)

                        
            lebar_pixel = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
            panjang_pixel = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
            if pixelsPerMetric is None:
                    pixelsPerMetric = lebar_pixel
                    pixelsPerMetric = panjang_pixel
            lebar = lebar_pixel
            panjang = panjang_pixel



                        
            cv2.putText(orig, "L: {:.1f}CM".format(lebar_pixel/30.1422),(int(trbrX + 10), int(trbrY)), cv2.FONT_HERSHEY_SIMPLEX,0.7, (0,0,255), 2)
            cv2.putText(orig, "P: {:.1f}CM".format(panjang_pixel/30.1422),(int(tltrX - 15), int(tltrY - 10)), cv2.FONT_HERSHEY_SIMPLEX,0.7, (0,0,255), 2)
                        
            hitung_objek+=1
            

                    #Menampilkan Jumlah Objek yang terdeteksi
            cv2.putText(orig, "Jumlah: {}".format(hitung_objek),(10,50),cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2, cv2.LINE_AA)  
            cv2.imshow('Kamera',orig)
            height= lebar_pixel/30.1422
            #print(149 + -26.5*height + 2.04*math.pow(height,2) + -0.0552*math.pow(height,3))
            distance= 149 + -26.5*height + 2.04*math.pow(height,2) + -0.0552*math.pow(height,3)
            rospy.loginfo(distance)
            if distance<10:
                 print("Stop")

             #print(151-26.9*(height)+2.05*math.pow(height,2)-0.0552*math.pow(height,3))

            key = cv2.waitKey(1)
            if key == 27:
                break
def midpoint(ptA, ptB):
        return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)
def main():
    
    rospy.init_node('height_measure')
    rospy.Subscriber("usb_cam/image_rect_color",Image,callback)
    rospy.loginfo("node has been started")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
if __name__=='__main__':
    main()



                    
