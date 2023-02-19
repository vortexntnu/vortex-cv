import numpy as np
import yaml
import sys

from pdaf import PDAF
from test_pdafTester import PDAFTester
"""

Unit tests for pdaf. 
ROS independent. 
Depends on monkey_tracking/data_generation. 
     !!!! OBS: must clone https://github.com/chrstrom/monkey_tracking and modify PATH_ variabels.

"""

# # ---- modify here: 
PATH_TO_DATA_GENERATION_REP = "/home/hannahcl/Documents/vortex/monkey_tracking/data_generation"


sys.path.insert(0, PATH_TO_DATA_GENERATION_REP)
from scenarios import BaseScenario
from load_config import load_yaml_into_dotdict


def test_pdaf_zero_velocity():

    x = 1
    y = 1
    tollerance = 0.5
    n_timesteps = 200

    pdafTester = PDAFTester()
    pdaf = pdafTester.create_pdaf_instance()

    pdaf.validation_gate_scaling_param = 4

    pdaf.prior_state_estimate.mean[0] = 0
    pdaf.prior_state_estimate.mean[1] = 0
    pdaf.prior_state_estimate.mean[2] = 10
    pdaf.prior_state_estimate.mean[3] = 5

    for i in range(len(pdaf.posterior_state_estimate.mean)):
        pdaf.model_disturbance.covariance[i, i] = 0.1

    for i in range(len(pdaf.C)):
        pdaf.measurment_noise.covariance[i, i] = 0.1

    for k in range(n_timesteps):

        o_time_k = pdafTester.create_observations_for_one_timestep_simple_version(x, y)

        pdaf.prediction_step()
        pdaf.correction_step(o_time_k)

    print(pdaf.posterior_state_estimate.mean)

    assert abs(pdaf.posterior_state_estimate.mean[0] - x) < tollerance
    assert abs(pdaf.posterior_state_estimate.mean[1] - y) < tollerance
    assert abs(pdaf.posterior_state_estimate.mean[2]) < tollerance
    assert abs(pdaf.posterior_state_estimate.mean[3]) < tollerance


def test_pdaf_constant_vel():
    """
    We simulate a boat with constant velocity in both x and y.
    """

    x = 5
    x_der = 0.9
    y = 1.2
    y_der = 0.8
    tollerance = 0.5
    n_timesteps = 200


    pdafTester = PDAFTester()
    pdaf = pdafTester.create_pdaf_instance()

    pdaf.validation_gate_scaling_param = 5

    pdaf.prior_state_estimate.mean[0] = 0
    pdaf.prior_state_estimate.mean[1] = 0
    pdaf.prior_state_estimate.mean[2] = 10
    pdaf.prior_state_estimate.mean[3] = 10

    for i in range(len(pdaf.posterior_state_estimate.mean)):
        pdaf.model_disturbance.covariance[i, i] = 0.1

    for i in range(len(pdaf.C)):
        pdaf.measurment_noise.covariance[i, i] = 0.1

    for k in range(n_timesteps):

        o_time_k = pdafTester.create_observations_for_one_timestep_simple_version(
            x + k * x_der * pdaf.time_step, y + k * y_der * pdaf.time_step
        )

        pdaf.prediction_step()
        pdaf.correction_step(o_time_k)


    print(
        "final true state: ",
        (x + x_der * (n_timesteps - 1) * pdaf.time_step),
        (y + y_der * (n_timesteps - 1) * pdaf.time_step),
        x_der,
        y_der,
    )

    print("final observations: ", o_time_k)

    print("final estimates: ", pdaf.posterior_state_estimate.mean)

    assert (
        abs(pdaf.posterior_state_estimate.mean[0] - (x + x_der * (n_timesteps - 1) * pdaf.time_step))
        < tollerance
    )
    assert (
        abs(pdaf.posterior_state_estimate.mean[1] - (y + y_der * (n_timesteps - 1) * pdaf.time_step))
        < tollerance
    )
    assert abs(pdaf.posterior_state_estimate.mean[2] - x_der) < tollerance
    assert abs(pdaf.posterior_state_estimate.mean[3] - y_der) < tollerance


def data_generation():

    config = load_yaml_into_dotdict("scenario.yaml")

    scenario = BaseScenario(config)

    measurements, ground_truths = scenario.run()

    return scenario, measurements, ground_truths


def test_filter_observations_outside_gate():


    pdafTester = PDAFTester()
    pdaf = pdafTester.create_pdaf_instance()

    n_obs = 10
    x = 1
    y = 0.5

    observations = np.ndarray((n_obs, 2), dtype=float)
    for i in range(n_obs):
        observations[i, 0] = x
        observations[i, 1] = y

    for i in range(n_obs - 5):
        observations[i, 0] = x + 2
        observations[i, 1] = y

    print("observations: ", observations)

    pdaf.compute_kalman_gain()
    pdaf.predited_observation.covariance = pdaf.C @ pdaf.prior_state_estimate.covariance @ pdaf.C.T + pdaf.measurment_noise.covariance
    pdaf.filter_observations_outside_gate(observations)

    print("observations within gate: ", pdaf.o_within_gate_arr)


def test_compute_probability_of_matching_observations():


    pdafTester = PDAFTester()
    pdaf = pdafTester.create_pdaf_instance()

    n_obs = 10
    x = 1
    y = 0.5

    observations = np.ndarray((n_obs, 2), dtype=float)

    for i in range(n_obs):
        observations[i, 0] = x + i * 0.1
        observations[i, 1] = y

    pdaf.compute_kalman_gain()
    pdaf.predited_observation.covariance = pdaf.C @ pdaf.prior_state_estimate.covariance @ pdaf.C.T + pdaf.measurment_noise.covariance
    pdaf.filter_observations_outside_gate(observations)
    p_arr = pdaf.compute_probability_of_matching_observations()

    print("p of matches: ", p_arr)
    print("p sum: ", np.sum(p_arr))

    assert abs(np.sum(p_arr) - 1) < 0.00001
    assert p_arr[0] == pdaf.p_no_match or len(pdaf.o_within_gate_arr) == 0




def test_correct_P():


    pdafTester = PDAFTester()
    pdaf = pdafTester.create_pdaf_instance()

    x = 4
    y = 0.5

    pdaf.posterior_state_estimate.mean[0] = x
    pdaf.posterior_state_estimate.mean[1] = y

    for i in range(20):
        observations = pdafTester.create_observations_for_one_timestep(x, y)

        pdaf.prediction_step()

        L = pdaf.compute_kalman_gain()
        pdaf.predited_observation.covariance = pdaf.C @ pdaf.prior_state_estimate.covariance @ pdaf.C.T + pdaf.measurment_noise.covariance

        pdaf.filter_observations_outside_gate(observations)
        p_arr = pdaf.compute_probability_of_matching_observations()
        res_vec = pdaf.compute_residual_vector(p_arr)
        pdaf.correct_state_vector(L, res_vec)

        pdaf.correct_P(L, res_vec, p_arr)

