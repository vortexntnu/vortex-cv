import numpy as np
from dataclasses import dataclass

"""
Single object tracking

Implementation based on chapter 7 in "Fundemental in Sensor Fusion" by Brekke, 2021 edition. 
Slides from PSU are nice for vizualization. https://www.cse.psu.edu/~rtc12/CSE598C/datassocPart2.pdf 

"""


@dataclass
class MultivariateGaussian:
    mean: np.ndarray
    covariance: np.ndarray
    timestamp: float


class PDAF:
    def __init__(self, config):
        # x = [x, y, x', y']

        self.time_step = config["pdaf"]["time_step"]  # can vary for each time step

        self.posterior_state_estimate = MultivariateGaussian(
            np.array(config["pdaf"]["state_post"]).reshape((4, 1)),
            np.array(config["pdaf"]["P_post"]).reshape((4, 4)),
            self.time_step,
        )

        self.prior_state_estimate = MultivariateGaussian(
            self.posterior_state_estimate.mean,
            self.posterior_state_estimate.covariance,
            self.time_step,
        )

        self.predited_observation = MultivariateGaussian(
            np.zeros((2, 1)), np.zeros((2, 2)), self.time_step
        )

        self.model_disturbance = MultivariateGaussian(
            np.zeros((4, 1)), np.array(config["pdaf"]["Q"]), self.time_step
        )

        self.measurment_noise = MultivariateGaussian(
            np.zeros((2, 1)), np.array(config["pdaf"]["R"]), self.time_step
        )

        self.C = np.array(  # C as in, y = C @ x
            [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]]
        )

        self.A = np.array(  # A as in, x' = A @ x
            [
                [1.0, 0.0, self.time_step, 0],
                [0, 1.0, 0, self.time_step],
                [0, 0, 1.0, 0],  # assuming constnat velocity
                [0, 0, 0, 1.0],  # assuming constnat velocity
            ]
        )

        self.validation_gate_scaling_param = config["pdaf"][
            "validation_gate_scaling_param"
        ]

        self.minimal_mahalanobis_distance = config["pdaf"][
            "minimal_mahalanobis_distance"
        ]  # observations that are closer then this, will be set to min distance

        self.p_no_match = config["pdaf"][
            "p_no_match"
        ]  # probabiity that no observations originates from the track

        self.o_within_gate_arr = None

    def step_once(self, o_arr, time_step):
        self.update_model(time_step)
        self.prediction_step()
        self.correction_step(o_arr)

    def prediction_step(self):

        self.prior_state_estimate.mean = self.A @ self.posterior_state_estimate.mean
        self.prior_state_estimate.covariance = (
            self.A @ self.posterior_state_estimate.covariance @ self.A.T
            + self.model_disturbance.covariance
        )

        self.predited_observation.mean = self.C @ self.prior_state_estimate.mean
        self.predited_observation.covariance = (
            self.C @ self.prior_state_estimate.covariance @ self.C.T
            + self.measurment_noise.covariance
        )

    def correction_step(self, o):

        L = self.compute_kalman_gain()

        self.filter_observations_outside_gate(o)

        if len(self.o_within_gate_arr) == 0:
            self.posterior_state_estimate = self.prior_state_estimate

        else:
            p_match_arr = self.compute_probability_of_matching_observations()
            residual_vector = self.compute_residual_vector(p_match_arr)

            self.correct_state_vector(L, residual_vector)
            self.correct_P(L, residual_vector, p_match_arr)

    def filter_observations_outside_gate(self, o):

        within_gate = []

        for o_i in o:
            o_i_arr = np.array(o_i).reshape(2, 1)
            mah_dist = self.compute_mah_dist(o_i_arr)
            if mah_dist < self.validation_gate_scaling_param**2:
                within_gate.append(o_i_arr)

        self.o_within_gate_arr = np.array(within_gate)

    def compute_mah_dist(self, o):
        "Compute mahaloanobis distance between observation and predicted observation."

        diff = o - self.predited_observation.mean
        mah_dist = diff.T @ np.linalg.inv(self.predited_observation.covariance) @ diff

        return mah_dist

    def compute_probability_of_matching_observations(self):

        score = np.zeros((len(self.o_within_gate_arr),))

        p_match_arr = np.zeros((len(self.o_within_gate_arr) + 1,))

        if len(self.o_within_gate_arr) == 0:
            p_match_arr[0] = 1.0
        else:
            p_match_arr[0] = self.p_no_match

        for i, o_i in enumerate(self.o_within_gate_arr):

            mah_distance = self.compute_mah_dist(o_i)
            if mah_distance <= self.minimal_mahalanobis_distance:
                score[i] = 1 / self.minimal_mahalanobis_distance
            else:
                score[i] = 1 / mah_distance

        score_sum = np.sum(score)
        for i in range(len(self.o_within_gate_arr)):
            p_match_arr[i + 1] = (score[i] / score_sum) * (1 - self.p_no_match)

        return p_match_arr

    def update_model(self, time_step):
        self.time_step = time_step
        self.A = np.array(
            [
                [1.0, 0.0, self.time_step, 0],
                [0, 1.0, 0, self.time_step],
                [0, 0, 1.0, 0],  # assuming constnat velocity
                [0, 0, 0, 1.0],  # assuming constnat velocity
            ]
        )

    def compute_residual_vector(self, p_match_arr):
        residual_vector = np.zeros((2, 1))
        for i in range(len(self.o_within_gate_arr)):
            residual_vector += p_match_arr[i + 1] * (
                self.o_within_gate_arr[i] - self.predited_observation.mean
            )

        return residual_vector

    def compute_kalman_gain(self):
        C_P_CT = self.C @ self.prior_state_estimate.covariance @ self.C.T
        L = (
            self.prior_state_estimate.covariance
            @ self.C.T
            @ np.linalg.inv(C_P_CT + self.measurment_noise.covariance)
        )
        return L

    def correct_state_vector(self, L, residual_vector):
        self.posterior_state_estimate.mean = (
            self.prior_state_estimate.mean + L @ residual_vector
        )

    def correct_P(self, L, residual_vector, p_match_arr):
        # qf - quadratic form
        qf_weighted_residual_vector = np.zeros((2, 2))
        for i, o_i in enumerate(self.o_within_gate_arr):
            conditional_innovations = o_i - self.predited_observation.mean

            qf_weighted_residual_vector += (
                p_match_arr[i + 1] * conditional_innovations @ conditional_innovations.T
            )

        qf_residual_vector = residual_vector @ residual_vector.T
        diff = qf_weighted_residual_vector - qf_residual_vector
        spread_of_innovations = L @ diff @ L.T  # given by (7.26) Brekke

        L_S_LT = L @ self.predited_observation.covariance @ L.T

        self.posterior_state_estimate.covariance = (
            self.prior_state_estimate.covariance
            - (1 - self.p_no_match) * L_S_LT
            + spread_of_innovations
        )  # given by (7.25) Brekke
