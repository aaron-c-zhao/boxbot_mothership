import numpy as np
import cv2
import math

vx = np.zeros((3, 0))
vy = np.zeros((3, 0))

x1 = 0
y1 = 0

def find_red_dot():

    capture = cv2.VideoCapture(4)
    # do this for each frame captured from the video
    while True:
        ret, image = capture.read()
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.GaussianBlur(image_gray, (7, 7), 1.5, image_gray, 1.5)
        rows = image_gray.shape[0]
        circles = cv2.HoughCircles(image_gray, cv2.HOUGH_GRADIENT, 1, rows/8, param1=100, param2=30, minRadius=1, maxRadius=80)
        mask = np.zeros(image.shape, image.dtype)

       
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv2.circle(mask, (i[0], i[1]), i[2], (255, 255, 255), -1, 8, 0)        
        
        image_circle = cv2.bitwise_and(mask, image) 

        #image_hsv = cv2.cvtColor(image_circle, cv2.COLOR_BGR2HSV)
        #low_red = cv2.inRange(image_hsv, (0, 100, 100), (20, 255, 255))
        #high_red = cv2.inRange(image_hsv, (150, 50, 50), (180, 255, 255))

        #image_red = cv2.bitwise_or(low_red, high_red) 
        edge = cv2.Canny(image_circle, 75, 200)

        valid_circles = []

        contours,_ = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 20:
                valid_circles.append(contour) 

        if len(valid_circles) > 0:
            x,y,w,h = cv2.boundingRect(valid_circles[0])

            xc = x + int(w/2) 
            yc = y + int(h/2)
            cv2.circle(image, (xc, yc), 2, (0, 255, 0), -1)
            cv2.circle(image, (xc, yc), int(w/2), (0, 255, 0), 2)
        
        cv2.circle(image, (1, 1), 2, (0, 0, 255), -1)
        cv2.imshow("caliration", image)
        key = cv2.waitKey(3) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows() 
            break    

    return xc, yc, w 

def find_max_contour():
    first_frame = None
    kernel = np.ones((20, 20), np.uint8)

    while True:
        image = cv2.imread('dots.jpg')

        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        image_gray = cv2.morphologyEx(image_gray, cv2.MORPH_CLOSE, kernel)

        image_gray = cv2.medianBlur(image_gray, 5)

        if first_frame is None:
            first_frame = image_gray 
            continue

        dif = cv2.absdiff(first_frame, image_gray)
        _, dif = cv2.threshold(dif, 50, 255, cv2.THRESH_BINARY)
        contrours,_ = cv2.findContours(dif, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        areas = [cv2.contourArea(c) for c in contrours]

        max_index = -1
        w = 0
        h = 0

        if len(areas) > 1:
            max_index = np.argmax(areas)
            x, y, w, h = cv2.boundingRect(contrours[max_index])
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        cv2.imshow('Frame', image)
        key = cv2.waitKey(0) & 0xFF
        if key == ord('q'):
            break
        if key == ord('c'):
            continue

    if max_index >= 0:
        return w, h 
    else:
        return -1, -1
    







def discover_rotation():
    global vx, vy
    global x1, y1
    # first point, the reference point
    x1, y1,_ = find_red_dot() 
    print("The first point is ({x}, {y})".format(x = x1, y = y1))

    # second point has the same y value as reference point in base frame 
    x2, y2, _ = find_red_dot()
    print("The second point is ({x}, {y})".format(x = x2, y = y2))

    # third point has the same x value as reference point in base frame
    x3, y3, _ = find_red_dot()
    print("The third point is ({x}, {y})".format(x = x3, y = y3))

    # find the x unit vector and the y unit vector of the base frame axis in the camera frame
    vx = np.array([x2 - x1, y2 - y1, 0], dtype=float)
    vx /= np.linalg.norm(vx)
    
    vy = np.array([x3-x1, y3 - y1, 0], dtype=float)
    vy /= np.linalg.norm(vy)

    # rotate around Z axis
    x_camera = np.array([1, 0, 0])
    y_camera = np.array([0, 1, 0])

    cos_a = np.dot(x_camera, vx)
    sin_a = math.sin(math.acos(cos_a)) 

    # first rotate around Z axis to align X axis
    rotate_z = np.array([[cos_a, -sin_a, 0], [sin_a, cos_a, 0], [0, 0, 1]])

    # rotate around x axis to align y axis
    vy = np.dot(rotate_z, vy)
    cos_b = np.dot(y_camera, vy)
    sin_b = math.sin(math.acos(cos_b)) 

    rotate_x = np.array([[1, 0, 0], [0, cos_b, -sin_b], [0, sin_b, cos_b]]) 
    # rotateion matrix
    rotate = np.matmul(rotate_x, rotate_z )

    return rotate


def discover_translation():
    print("Start pixel size calculation:")    

    # discover the size of the object in pixels
    x, y, w  = find_red_dot() 

    true_w = float(input("Enter the diameter of the circle: "))

    # size of each pixel
    pixel_to_meter = true_w / w; 

    base_o_in_cam = np.array([x1, y1, 0])

    # projections
    x_offset = abs(np.dot(base_o_in_cam, vx)) * pixel_to_meter
    y_offset = abs(np.dot(base_o_in_cam, vy)) * pixel_to_meter

    print('Please enter the coordinate of the first point in the base frame:')
    x1_base = float(input("x: "))
    y1_base = float(input("y: "))

    print(x1_base)
    print(y1_base)

    translation = np.array([[x1_base - x_offset], [y1_base + y_offset], [0.0]])

    return translation 
     
    




def main():
    rm = discover_rotation()
    tm = discover_translation()

    extra_row = np.array([[0, 0, 0, 1]]) 

    homo_m = np.concatenate((rm, tm), axis = 1)
    homo_m = np.concatenate((homo_m, extra_row), axis=0)
    
    print(homo_m)

    

if __name__ == "__main__":
    main()