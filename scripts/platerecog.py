#!/usr/bin/python
from math import sqrt
import rospy
import numpy as np
import cv2
from arobo.srv import FindDistance

def point_to_line_dist(point, line):
    """Calculate the distance between a point and a line segment.

    To calculate the closest distance to a line segment, we first need to check
    if the point projects onto the line segment.  If it does, then we calculate
    the orthogonal distance from the point to the line.
    If the point does not project to the line segment, we calculate the 
    distance to both endpoints and take the shortest distance.

    :param point: Numpy array of form [x,y], describing the point.
    :type point: numpy.core.multiarray.ndarray
    :param line: list of endpoint arrays of form [P1, P2]
    :type line: list of numpy.core.multiarray.ndarray
    :return: The minimum distance to a point.
    :rtype: float
    """
    # unit vector
    unit_line = line[1] - line[0]
    norm_unit_line = unit_line / np.linalg.norm(unit_line)

    # compute the perpendicular distance to the theoretical infinite line
    segment_dist = (
        np.linalg.norm(np.cross(line[1] - line[0], line[0] - point)) /
        np.linalg.norm(unit_line)
    )

    diff = (
        (norm_unit_line[0] * (point[0] - line[0][0])) + 
        (norm_unit_line[1] * (point[1] - line[0][1]))
    )

    x_seg = (norm_unit_line[0] * diff) + line[0][0]
    y_seg = (norm_unit_line[1] * diff) + line[0][1]

    endpoint_dist = min(
        np.linalg.norm(line[0] - point),
        np.linalg.norm(line[1] - point)
    )

    # decide if the intersection point falls on the line segment
    lp1_x = line[0][0]  # line point 1 x
    lp1_y = line[0][1]  # line point 1 y
    lp2_x = line[1][0]  # line point 2 x
    lp2_y = line[1][1]  # line point 2 y
    is_betw_x = lp1_x <= x_seg <= lp2_x or lp2_x <= x_seg <= lp1_x
    is_betw_y = lp1_y <= y_seg <= lp2_y or lp2_y <= y_seg <= lp1_y
    if is_betw_x and is_betw_y:
        return segment_dist
    else:
        # if not, then return the minimum distance to the segment endpoints
        return endpoint_dist


def handle_srv(req):
    [centrdist, mindist, error] = main(req.filename)
    return str([centrdist, mindist, error])
    
def main(filename):
    #read image
    image = cv2.imread("/home/georgy/catkin_ws/src/arobo/data/" + filename)

    #manually crop image
    image = image[int(image.shape[0]*0.1):int(image.shape[0]*0.85), int(image.shape[1]*0.03):int(image.shape[1]*0.96)]

    #median filter
    kernel = np.ones((7,7),np.float32)/49
    dst = cv2.filter2D(image,-1,kernel)       

    #manually choose best threshold setting
    #for i in range(140,150,1):
    #    upper = np.array([i,i,i])
    #    lower = np.array([0,0,0])
    #    mask = cv2.inRange(dst, lower,upper)
    #    print i
    #    cv2.imshow("Image",mask)
    #    cv2.waitKey(0)
    
    #146 was the best option
    i = 146
    upper = np.array([i,i,i])
    lower = np.array([0,0,0])
    mask = cv2.inRange(dst, lower,upper)

    #find contours in the masked image and keep 2 largest 
    (_, contours, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    c1 = max(contours, key = cv2.contourArea)
    del(contours[contours.index(c1)])
    c2 =  max(contours, key = cv2.contourArea)
    
    #approx contour to polygon
    peri1 = cv2.arcLength(c1, True)
    approx1 = cv2.approxPolyDP(c1, 0.05 * peri1, 2)
    peri2 = cv2.arcLength(c2, True)
    approx2 = cv2.approxPolyDP(c2, 0.05 * peri2, 2)
    #green bound
    cv2.drawContours(image, [approx1], -1, (0,255,0), 2)
    cv2.drawContours(image, [approx2], -1, (255,0,0), 2)
    
    #task 1
    #pixel distance between centres
    centr1, centr2 = sum(approx1)/4.0,sum(approx2)/4.0
    dist = centr2-centr1
    #pixel area of each plate
    area1 = cv2.contourArea(approx1)
    area2 = cv2.contourArea(approx2)
    area = (area1 + area2)/2.0
    #conv distance to mm
    mmdist = (sqrt(300.*200./area)*dist)
    mmdist = (mmdist[0,0]**2 + mmdist[0,1]**2)**(0.5)    

    #task 2
    #pixel min distance between counters
    #remove unnecessary dimension
    approx1 = np.squeeze(approx1,axis=1)
    approx2 = np.squeeze(approx2,axis=1)
    
    #find min in all combinations
    mindist = 10000
    for i in approx1:
        for j in range(4):
             tmp = point_to_line_dist(i,[approx2[j],approx2[(j+1)%4]])
             if tmp < mindist:
                 mindist = tmp
    for i in approx2:
        for j in range(4):
             tmp = point_to_line_dist(i,[approx1[j],approx1[(j+1)%4]])
             if tmp < mindist:
                 mindist = tmp
    #convert to mm
    mmmindist = sqrt(300.*200./area)*mindist

    print "Distance between rectangle centres = ", '{0:.2f}'.format(mmdist), "mm" 
    print "Nearest distance between rectangles = ", '{0:.2f}'.format(mmmindist), "mm" 
    
    #task 3
    #error estimation
    print "Error estimate:"
    print "1. Filter error"
    print "2. Camera fish eye error"
    print "3. Noise error"
    print "4. Borders error"
    print "5. Pixel to mm convertion error"
    print "6. Rectangle approximation"
    pix_error = sqrt(area1)-sqrt(area2)
    # + half filter kernel size
    pix_error = + 3.5
    #Conversion to mm
    mm_error = sqrt(300.*200./area)*pix_error
    print "Estimated error from filter + figure recog = ", '{0:.2f}'.format(mm_error), "mm"
    print "Real error could be bigger"
    cv2.imwrite("/home/georgy/catkin_ws/src/arobo/data/modified_" + filename, image)
    #cv2.imshow("Image", image)
    #cv2.waitKey(0)
    
    return [mmdist, mmmindist, mm_error]

        


if __name__ == "__main__":
    rospy.init_node("cvservice")
    s = rospy.Service("find_distance", FindDistance, handle_srv)
    rospy.spin()
