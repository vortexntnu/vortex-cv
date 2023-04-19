from copy import copy
import numpy as np

rng = np.random.RandomState(123)


class RANSAC:

    def __init__(self, n, k, t, d, model=None, loss=None, metric=None):
        self.n = n  # `n`: Minimum number of data points to estimate parameters
        self.k = k  # `k`: Maximum iterations allowed
        self.t = t  # `t`: Threshold value to determine if points are fit well
        self.d = d  # `d`: Number of close data points required to assert model fits well
        self.model = model  # `model`: class implementing `fit` and `predict`
        self.loss = loss  # `loss`: function of `y_true` and `y_pred` that returns a vector
        self.metric = metric  # `metric`: function of `y_true` and `y_pred` and returns a float
        self.best_fit = None
        self.best_error = np.inf
        self.points = None

    def fit(self, X, y):
        for _ in range(self.k):  #while a == False or i > self.k:
            ids = rng.permutation(X.shape[0])

            maybe_inliers = ids[:self.n]
            maybe_model = copy(self.model).fit(X[maybe_inliers],
                                               y[maybe_inliers])

            thresholded = (self.loss(y[ids][self.n:],
                                     maybe_model.predict(X[ids][self.n:]))
                           < self.t)

            inlier_ids = ids[self.n:][np.flatnonzero(thresholded).flatten()]

            if inlier_ids.size > self.d:
                inlier_points = np.hstack([maybe_inliers, inlier_ids])
                better_model = copy(self.model).fit(X[inlier_points],
                                                    y[inlier_points])

                this_error = self.metric(
                    y[inlier_points], better_model.predict(X[inlier_points]))

                if this_error < self.best_error:
                    self.points = inlier_points
                    self.best_error = this_error
                    self.best_fit = maybe_model

        return self

    def predict(self, X):
        return self.best_fit.predict(X)


class LinearRegressor:

    def __init__(self):
        self.params = None

    def fit(self, X, y):
        r, _ = X.shape
        X = np.hstack([np.ones((r, 1)), X])
        self.params = np.dot(np.linalg.inv(np.dot(X.T, X)), np.dot(X.T, y))
        return self

    def predict(self, X):
        r, _ = X.shape
        X = np.hstack([np.ones((r, 1)), X])
        return np.dot(X, self.params)
