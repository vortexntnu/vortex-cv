import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from test_RANSAC import RANSAC, LinearRegressor

from feature_detection import ImageFeatureProcessing as ifp
from sympy import symbols, Eq, solve

img = cv.imread("./data/path_bendy_full.png")
img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)

plt.figure(1, figsize=(10, 10))
plt.imshow(img_rgb)

x = ifp(img)

hsv_hue_min = 5
hsv_hue_max = 55
hsv_sat_min = 5
hsv_sat_max = 55
hsv_val_min = 5
hsv_val_max = 55

img2, a, b = x.hsv_processor(img, hsv_hue_min, hsv_hue_max, hsv_sat_min,
                             hsv_sat_max, hsv_val_min, hsv_val_max)

#img2_rgb = cv.cvtColor(img2, cv.COLOR_BGR2RGB)

imgray = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(imgray, 127, 230, 0)
im2, contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE,
                                           cv.CHAIN_APPROX_SIMPLE)

#vx, vy, x0, y0 = cv2.fitLine(img2, cv2.DIST_L2, 0, 0.1, 0.1)
plt.figure(2, figsize=(10, 10))
plt.imshow(im2, cmap=plt.cm.gray)

# Convert the image to a set of 2D points
points = np.argwhere(im2 > 220)
X = points[:, 0].reshape(-1, 1)
y = points[:, 1].reshape(-1, 1)

#print(points)
plt.figure(3, figsize=(10, 10))
plt.axis('equal')
plt.xlim = (0, 1239)
plt.ylim = (0, 927)
plt.scatter(X, y)


# ------ RANSAC wikipedia
def square_error_loss(y_true, y_pred):
    return (y_true - y_pred)**2


def mean_square_error(y_true, y_pred):
    return np.sum(square_error_loss(y_true, y_pred)) / y_true.shape[0]


n = 10
k = 1000
t = 500
d = np.size(points) / 6
regressor = RANSAC(n,
                   k,
                   t,
                   d,
                   model=LinearRegressor(),
                   loss=square_error_loss,
                   metric=mean_square_error)

#X = np.array([-0.848,-0.800,-0.704,-0.632,-0.488,-0.472,-0.368,-0.336,-0.280,-0.200,-0.00800,-0.0840,0.0240,0.100,0.124,0.148,0.232,0.236,0.324,0.356,0.368,0.440,0.512,0.548,0.660,0.640,0.712,0.752,0.776,0.880,0.920,0.944,-0.108,-0.168,-0.720,-0.784,-0.224,-0.604,-0.740,-0.0440,0.388,-0.0200,0.752,0.416,-0.0800,-0.348,0.988,0.776,0.680,0.880,-0.816,-0.424,-0.932,0.272,-0.556,-0.568,-0.600,-0.716,-0.796,-0.880,-0.972,-0.916,0.816,0.892,0.956,0.980,0.988,0.992,0.00400]).reshape(-1,1)
#y = np.array([-0.917,-0.833,-0.801,-0.665,-0.605,-0.545,-0.509,-0.433,-0.397,-0.281,-0.205,-0.169,-0.0531,-0.0651,0.0349,0.0829,0.0589,0.175,0.179,0.191,0.259,0.287,0.359,0.395,0.483,0.539,0.543,0.603,0.667,0.679,0.751,0.803,-0.265,-0.341,0.111,-0.113,0.547,0.791,0.551,0.347,0.975,0.943,-0.249,-0.769,-0.625,-0.861,-0.749,-0.945,-0.493,0.163,-0.469,0.0669,0.891,0.623,-0.609,-0.677,-0.721,-0.745,-0.885,-0.897,-0.969,-0.949,0.707,0.783,0.859,0.979,0.811,0.891,-0.137]).reshape(-1,1)

regressor.fit(X, y)
a = regressor.points
plt.scatter(X[a], y[a])

# LinReg = LinearRegressor()
# line = LinReg.fit(X[a],y[a])
# line_params = line.params
# np.info(line_params)
# print(line_params)
# x = np.linspace(0,1239,100)
# y = line_params[0]*x + line_params[1]
# plt.plot(x,y, c="peru")

params = regressor.best_fit.params

line = np.linspace(0, 1239, num=100).reshape(-1, 1)
y = regressor.predict(line)
plt.plot(line, y, c="peru")

params = regressor.best_fit.params
alpha = float(params[1])
beta = float(params[0])
x, y = symbols('x y')

# alpha = 1.003
# print('hellow world')
# np.info(alpha)
# beta = 1.003

y = alpha * x + beta

sol = solve(Eq(y, 0))
x0 = float(sol[0])
y0 = beta
vx = 1
vy = alpha

colin_vec = np.ravel(np.array((vx, vy)))
p0 = np.ravel(np.array((x0, y0)))

colin_vec = [colin_vec[0], colin_vec[1], 0]
p0 = [p0[0], p0[1], 0]

print('params:', params)

plt.show()
