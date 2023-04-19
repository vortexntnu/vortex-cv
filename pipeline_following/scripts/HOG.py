import cv2
import numpy as np
"""
Implementation of Histogram of Orientated Gradients (HOG).
- Lasse Moen Guttormsen
"""


class HOG:

    def __init__(self, image, cell_size=(8, 8), block_size=(2, 2), nbins=9):
        self.image = image
        self.cell_size = cell_size
        self.block_size = block_size
        self.nbins = nbins
        self.features = None
        self.hog_image = None

    def compute_hog(self):
        # Convert image to grayscale
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Compute gradients in x and y directions
        gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0)
        gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1)

        # Compute gradient magnitude and direction
        mag, angle = cv2.cartToPolar(gx, gy)

        # Round the angle to the nearest orientation bin
        angle = np.int32(self.nbins * angle / (2 * np.pi))

        # Divide the image into cells
        height, width = gray.shape
        cell_height, cell_width = self.cell_size
        n_cells_y = height // cell_height
        n_cells_x = width // cell_width
        cells = np.zeros((n_cells_y, n_cells_x, self.nbins))

        # Accumulate the histograms of gradient directions in each cell
        for i in range(n_cells_y):
            for j in range(n_cells_x):
                cell_mag = mag[i * cell_height:(i + 1) * cell_height,
                               j * cell_width:(j + 1) * cell_width]
                cell_angle = angle[i * cell_height:(i + 1) * cell_height,
                                   j * cell_width:(j + 1) * cell_width]
                hist = np.bincount(cell_angle.ravel(),
                                   weights=cell_mag.ravel(),
                                   minlength=self.nbins)
                cells[i, j] = hist

        # Normalize the block of cells
        block_height, block_width = self.block_size
        eps = 1e-7
        self.features = np.zeros(
            (n_cells_y - block_height + 1, n_cells_x - block_width + 1,
             self.nbins * block_height * block_width))
        self.hog_image = np.zeros((height, width))
        for i in range(n_cells_y - block_height + 1):
            for j in range(n_cells_x - block_width + 1):
                block = cells[i:i + block_height, j:j + block_width]
                block_norm = np.linalg.norm(block) + eps
                block_features = block.ravel() / block_norm
                self.features[i, j] = block_features
                self.hog_image[i * cell_height:(i + block_height) *
                               cell_height, j * cell_width:(j + block_width) *
                               cell_width] = block_norm

        return self.features.ravel(), self.hog_image
