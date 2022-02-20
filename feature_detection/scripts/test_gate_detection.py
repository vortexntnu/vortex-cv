import numpy as np
import cv2 as cv2
import matplotlib.pyplot as plt


# def im2double(im):
#     """
#     Ensures that the returned image is a floating-point image,
#     with pixel values in the range [0,1].
#     """
#     if im.dtype == np.uint8: return im/255.0
#     else: return im

# def rgb_to_gray(I):
#     """
#     Converts a HxWx3 RGB image to a HxW grayscale image by averaging.
#     """
#     R = I[:,:,0]
#     G = I[:,:,1]
#     B = I[:,:,2]
#     return (R + G + B)/3.0

def draw_line(theta, rho, **args):
    """
    Draws a line given in normal form (rho, theta).
    Uses the current plot's xlim and ylim as bounds.
    """

    def clamp(a, b, a_min, a_max, rho, A, B):
        if a < a_min or a > a_max:
            a = np.fmax(a_min, np.fmin(a_max, a))
            b = (rho-a*A)/B
        return a, b

    x_min,x_max = np.sort(plt.xlim())
    y_min,y_max = np.sort(plt.ylim())
    c = np.cos(theta)
    s = np.sin(theta)
    if np.fabs(s) > np.fabs(c):
        x1 = x_min
        x2 = x_max
        y1 = (rho-x1*c)/s
        y2 = (rho-x2*c)/s
        y1,x1 = clamp(y1, x1, y_min, y_max, rho, s, c)
        y2,x2 = clamp(y2, x2, y_min, y_max, rho, s, c)
    else:
        y1 = y_min
        y2 = y_max
        x1 = (rho-y1*s)/c
        x2 = (rho-y2*s)/c
        x1,y1 = clamp(x1, y1, x_min, x_max, rho, c, s)
        x2,y2 = clamp(x2, y2, x_min, x_max, rho, c, s)
    plt.plot([x1, x2], [y1, y2], **args)

def derivative_of_gaussian(I, sigma):
    """
    Computes the gradient in the x and y direction using the derivatives
    of a 2-D Gaussian, as described in HW1 Task 3.6. Returns the gradient
    images (Ix, Iy) and the magnitude Im.
    """
    h = int(np.ceil(3*sigma))
    x = np.arange(2*h + 1) - h
    e = np.exp(-x**2/(2*sigma**2))
    g = e/np.sqrt(2*np.pi*sigma**2)
    d = -x*e/(sigma*sigma*sigma*np.sqrt(2*np.pi))
    Ix = np.zeros_like(I)
    Iy = np.zeros_like(I)
    for row in range(I.shape[0]): Ix[row,:] = np.convolve(I[row,:], d, mode='same')
    for col in range(I.shape[1]): Ix[:,col] = np.convolve(Ix[:,col], g, mode='same')
    for col in range(I.shape[1]): Iy[:,col] = np.convolve(I[:,col], d, mode='same')
    for row in range(I.shape[0]): Iy[row,:] = np.convolve(Iy[row,:], g, mode='same')
    return Ix, Iy, np.sqrt(Ix**2 + Iy**2)

def gaussian(I, sigma):
    """
    Applies a 2-D Gaussian blur with standard deviation sigma to
    a grayscale image I.
    """
    h = int(np.ceil(3*sigma))
    x = np.linspace(-h, +h, 2*h + 1)
    g = np.exp(-x**2/(2*sigma**2))/np.sqrt(2*np.pi*sigma**2)
    Ig = np.zeros_like(I)
    for row in range(I.shape[0]): Ig[row,:] = np.convolve(I[row,:], g, mode='same')
    for col in range(I.shape[1]): Ig[:,col] = np.convolve(Ig[:,col], g, mode='same')
    return Ig

def extract_edges(Ix, Iy, Im, threshold):
    """
    Returns the x, y coordinates of pixels whose gradient
    magnitude is greater than the threshold. Also, returns
    the angle of the image gradient at each extracted edge.
    """
    y,x = np.nonzero(Im > threshold)
    theta = np.arctan2(Iy[y,x], Ix[y,x])
    return x, y, theta

def extract_local_maxima(H, threshold):
    """
    Returns the row and column of cells whose value is strictly greater than its
    8 immediate neighbors, and greater than or equal to a threshold. The threshold
    is specified as a fraction of the maximum array value.

    Note: This returns (row,column) coordinates.
    """
    assert(len(H.shape) == 2) # Must be gray-scale array
    assert(threshold >= 0 and threshold <= 1) # Threshold is specified as fraction of maximum array value
    absolute_threshold = threshold*H.max()
    maxima = []
    for row in range(1, H.shape[0]-1):
        for col in range(1, H.shape[1]-1):
            window = H[row-1:row+2, col-1:col+2]
            center = window[1,1]
            window[1,1] = 0.0
            if center > window.max() and center >= absolute_threshold:
                maxima.append((row,col))
            window[1,1] = center
    maxima = np.array(maxima)
    return maxima[:,0], maxima[:,1]


def hough_transform(image,N_rho, N_theta,blur_sigma, line_threshold):
    ## calling functions 
    img_contrast = cv2.multiply(img_gray,2.3)
    Ix, Iy, Im     = derivative_of_gaussian(img_contrast, sigma=blur_sigma) # See HW1 Task 3.6
    x,y,theta      = extract_edges(Ix, Iy, Im, edge_threshold)

    ## getting height and weight
    height = image.shape[0]
    width = image.shape[1]

    ## initializing boundaries of polar coordinates
    rho_max = np.sqrt(height^2 + width^2)
    rho_min = -rho_max
    theta_max = np.pi
    theta_min = - theta_max

    ## computating accumulator array (hough space)
    H = np.zeros((N_rho, N_theta))
    rho = (x + 0.5)*np.cos(theta) + (y + 0.5)*np.sin(theta)
    rows = np.floor(N_rho*(rho - rho_min)/(rho_max - rho_min)).astype(np.int)
    cols = np.floor(N_theta*(theta - theta_min)/(theta_max - theta_min)).astype(np.int)
    for row,col in zip(rows,cols):
        if row >= 0 and row < N_rho and col >= 0 and col < N_theta:
            H[row,col] += 1
    
    ## extracting maxima
    peak_rows,peak_cols = extract_local_maxima(H, threshold=line_threshold)
    peak_rho   = rho_min + (rho_max - rho_min)*(peak_rows + 0.5)/N_rho
    peak_theta = theta_min + (theta_max - theta_min)*(peak_cols + 0.5)/N_theta

    print(peak_rows, peak_cols)

    plt.figure()
    plt.imshow(H, extent=[theta_min, theta_max, rho_max, rho_min], aspect='auto')
    plt.colorbar(label='Votes')
    plt.scatter(peak_theta, peak_rho, linewidths=1, edgecolor='red', color='none', s=40)
    plt.title('Accumulator array')
    plt.xlabel('$\\theta$ (radians)')
    plt.ylabel('$\\rho$ (pixels)')

    plt.figure()
    plt.imshow(image)
    plt.xlim([0, image.shape[1]])
    plt.ylim([image.shape[0], 0])
    for theta,rho in zip(peak_theta,peak_rho):
        draw_line(theta, rho, color='yellow')
    plt.title('Dominant lines')
    # plt.savefig('out_lines.png', bbox_inches='tight', pad_inches=0) # Uncomment to save figure

    plt.show()
    return H 


blur_sigma = 1
edge_threshold = 0.5
N_rho = 500
N_theta = 500
line_threshold = 0.2

img = cv2.imread('feature_detection/test_image/gate_day1_medium_yaw.png')
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# img_blur = cv2.blur(img_gray, (5,5)) # welche Kernel Size ist für diese Anwendung passend?

H = hough_transform(img_gray,N_rho, N_theta,blur_sigma, line_threshold)

# plt.figure()
# plt.imshow('Gate', H)

# image_output = np.hstack((img, H))
# cv2.imshow('Gate', H)
# # cv2.imshow('Gate', img)
# cv2.waitKey(0)


