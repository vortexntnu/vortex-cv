import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from skimage.feature import hog

import numpy as np
import cv2
from test_HOG_implementation import compute_hog


def compute_gradients(image):
    """
    Computes the gradient magnitude and direction of an image.
    
    Args:
        image: A NumPy array representing the input image.
        
    Returns:
        mag: A NumPy array representing the gradient magnitude of the input image.
        angle: A NumPy array representing the gradient direction of the input image.
    """
    dx = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=5)
    dy = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=5)
    mag, angle = cv2.cartToPolar(dx, dy, angleInDegrees=True)
    return mag, angle


def compute_hog_descriptor(image,
                           cell_size=(8, 8),
                           block_size=(2, 2),
                           nbins=9):
    """
    Computes the HOG descriptor of an image.
    
    Args:
        image: A NumPy array representing the input image.
        cell_size: A tuple representing the size of each cell in the HOG descriptor.
        block_size: A tuple representing the size of each block in the HOG descriptor.
        nbins: An integer representing the number of orientation bins in the HOG descriptor.
        
    Returns:
        hog: A NumPy array representing the HOG descriptor of the input image.
    """
    # Compute the gradient magnitude and direction of the input image
    mag, angle = compute_gradients(image)

    # Define the number of cells and blocks in the HOG descriptor
    cells_per_block = (block_size[0] * cell_size[0],
                       block_size[1] * cell_size[1])
    n_cells_x = int(image.shape[1] // cell_size[1])
    n_cells_y = int(image.shape[0] // cell_size[0])
    n_blocks_x = n_cells_x - block_size[1] + 1
    n_blocks_y = n_cells_y - block_size[0] + 1

    # Initialize the HOG descriptor
    hog = np.zeros(
        (n_blocks_y, n_blocks_x, block_size[0], block_size[1], nbins))

    # Compute the orientation histograms for each cell
    for i in range(n_cells_y):
        for j in range(n_cells_x):
            cell_mag = mag[i * cell_size[0]:(i + 1) * cell_size[0],
                           j * cell_size[1]:(j + 1) * cell_size[1]]
            cell_angle = angle[i * cell_size[0]:(i + 1) * cell_size[0],
                               j * cell_size[1]:(j + 1) * cell_size[1]]
            hist = np.zeros(nbins)
            for k in range(nbins):
                hist[k] = np.sum(cell_mag * (cell_angle >= k * (180 / nbins)) *
                                 (cell_angle < (k + 1) * (180 / nbins)))
            hog[i:i + block_size[0], j:j + block_size[1], :, :, :] += hist

    # Normalize the blocks in the HOG descriptor
    eps = 1e-7
    hog /= np.sqrt(np.square(hog).sum(axis=-1, keepdims=True) + eps)

    # Reshape the HOG descriptor and flatten it
    hog = hog.reshape(-1, block_size[0] * block_size[1], nbins).flatten()

    return hog


threshold_values = {0}
h = [1]


def Hist(img):
    row, col = img.shape
    y = np.zeros(256)
    for i in range(0, row):
        for j in range(0, col):
            y[img[i, j]] += 1
    x = np.arange(0, 256)
    plt.bar(x, y, color='b', width=5, align='center', alpha=0.25)
    plt.show()
    return y


def regenerate_img(img, threshold):
    row, col = img.shape
    y = np.zeros((row, col))
    for i in range(0, row):
        for j in range(0, col):
            if img[i, j] >= threshold:
                y[i, j] = 255
            else:
                y[i, j] = 0
    return y


def countPixel(h):
    cnt = 0
    for i in range(0, len(h)):
        if h[i] > 0:
            cnt += h[i]
    return cnt


def wieght(s, e):
    w = 0
    for i in range(s, e):
        w += h[i]
    return w


def mean(s, e):
    m = 0
    w = wieght(s, e)
    for i in range(s, e):
        m += h[i] * i

    return m / float(w)


def variance(s, e):
    v = 0
    m = mean(s, e)
    w = wieght(s, e)
    for i in range(s, e):
        v += ((i - m)**2) * h[i]
    v /= w
    return v


def threshold(h):
    cnt = countPixel(h)
    for i in range(1, len(h)):
        vb = variance(0, i)
        wb = wieght(0, i) / float(cnt)
        mb = mean(0, i)

        vf = variance(i, len(h))
        wf = wieght(i, len(h)) / float(cnt)
        mf = mean(i, len(h))

        V2w = wb * (vb) + wf * (vf)
        V2b = wb * wf * (mb - mf)**2

        fw = open("trace.txt", "a")
        fw.write('T=' + str(i) + "\n")

        fw.write('Wb=' + str(wb) + "\n")
        fw.write('Mb=' + str(mb) + "\n")
        fw.write('Vb=' + str(vb) + "\n")

        fw.write('Wf=' + str(wf) + "\n")
        fw.write('Mf=' + str(mf) + "\n")
        fw.write('Vf=' + str(vf) + "\n")

        fw.write('within class variance=' + str(V2w) + "\n")
        fw.write('between class variance=' + str(V2b) + "\n")
        fw.write("\n")

        if not math.isnan(V2w):
            threshold_values[i] = V2w


def get_optimal_threshold():
    min_V2w = min(threshold_values.itervalues())
    optimal_threshold = [
        k for k, v in threshold_values.iteritems() if v == min_V2w
    ]
    print 'optimal threshold', optimal_threshold[0]
    return optimal_threshold[0]


img = cv.imread("./images/image_23.jpg")
''' skimage HOG
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
hog_features, hog_image = hog(gray,
                              orientations=8,
                              pixels_per_cell=(16, 16),
                              cells_per_block=(1, 1),
                              block_norm='L2-Hys',
                              visualize=True)
'''

hog_descriptor, hog_image = compute_hog(img)

threshold = 0.5 * hog_image.max()
if threshold < 2:
    threshold = 2
binary_image = hog_image > threshold
'''
threshold(h)
op_thres = get_optimal_threshold()
binary_image = regenerate_img(hog_image, op_thres)
'''
'''
hog_img = hog_image / hog_image.max()
(hog_img * 255).astype('uint8')
print(hog_img[1, 1])
#gray_img = cv2.cvtColor(hog_img, cv2.COLOR_GRAY2BGR)
otsu_threshold, binary_image = cv2.threshold(
    hog_img,
    0,
    255,
    cv2.THRESH_BINARY + cv2.THRESH_OTSU,
)
'''

plt.figure(1, figsize=(10, 10))
plt.imshow(hog_image)

plt.figure(2, figsize=(10, 10))
plt.imshow(binary_image)
plt.show()
