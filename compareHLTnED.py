import numpy
import numpy as np
import cv2 as cv
import math

#masking Parameter
lower = np.array([0, 93, 0])
upper = np.array([225, 255, 255])

#ED Parameter
ed = cv.ximgproc.createEdgeDrawing()
# you can change parameters (refer the documentation to see all parameters)
EDParams = cv.ximgproc_EdgeDrawing_Params()
EDParams.MinPathLength = 0  # try changing this value between 5 to 1000
EDParams.PFmode = False  # defaut value try to swich it to True
EDParams.MinLineLength = 100  # try changing this value between 5 to 100
EDParams.NFAValidation = True  # defaut value try to swich it to False
ed.setParams(EDParams)

def drawLine2 (lines,img1):
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
            pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
            cv.line(img1, pt1, pt2, (0, 255, 0), 3, cv.LINE_AA)
            values.append([rho, theta])
    return img1

def drawED (lines,img2) :
    if lines is not None: # Check if the lines have been found and only then iterate over these and add them to the image
            lines = np.uint16(np.around(lines))
            for i in range(len(lines)):
                cv.line(img2, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]), (0, 0, 255), 1, cv.LINE_AA)
    return img2

values = []
img = cv.imread('epuk20.jpg')
img1 = cv.imread('epuk20.jpg')
img2 = cv.imread('epuk20.jpg')

hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
mask = cv.inRange(hsv, lower, upper)
#output = cv.bitwise_and(img,img, mask= mask)
blur = cv.GaussianBlur(mask, (5, 5), 0)
edges = cv.Canny(blur,10,150)

ed.detectEdges(blur)
linesED = ed.detectLines()

lines = cv.HoughLines(edges, 1, np.pi / 180, 90, None, 0, 0)

print(lines)
print(linesED)

cv.imshow("HLT",drawLine2(lines,img1))
cv.imshow("ED",drawED(linesED,img2))
cv.waitKey()
