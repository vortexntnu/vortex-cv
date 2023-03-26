import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

img = cv.imread("./data/path_bendy_full.png")

gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# Apply edge detection (Canny)
edges = cv.Canny(gray, 50, 150, apertureSize=3)

print(edges)

# Apply the Hough Transform to detect lines
lines = cv.HoughLines(edges, rho=1, theta=np.pi/180, threshold=100)

# Draw the detected lines on the original image
for line in lines:
    rho, theta = line[0]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(a))
    cv.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

plt.figure(1, figsize=(10, 10))
plt.imshow(img)
plt.show()