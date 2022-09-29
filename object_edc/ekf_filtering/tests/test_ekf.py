import pickle
from numpy.core.numeric import isscalar
import pytest
from copy import deepcopy
import sys
from pathlib import Path
import numpy as np
import os
from dataclasses import is_dataclass

assignment_name = "ekf"

this_file = Path(__file__)
tests_folder = this_file.parent
test_data_file = tests_folder.joinpath("test_data.pickle")
project_folder = tests_folder.parent
code_folder = project_folder.joinpath(assignment_name)

sys.path.insert(0, str(code_folder))

import solution  # nopep8
import ekf  # nopep8


@pytest.fixture
def test_data():
    with open(test_data_file, "rb") as file:
        test_data = pickle.load(file)
    return test_data


def compare(a, b):
    if (
        isinstance(a, np.ndarray)
        or isinstance(b, np.ndarray)
        or np.isscalar(a)
        or np.isscalar(b)
    ):
        return np.allclose(a, b)
    elif is_dataclass(a) or is_dataclass(b):
        return str(a) == str(b)
    else:
        return a == b


class TestOutput:

    def test_output__EKF_predict(self, test_data):
        for finput in test_data["ekf.EKF.predict"]:
            params = tuple(finput.values())

            self_1, state_upd_prev_gauss_1, Ts_1 = deepcopy(params)

            self_2, state_upd_prev_gauss_2, Ts_2 = deepcopy(params)

            state_pred_gauss_1 = ekf.EKF.predict(
                self_1, state_upd_prev_gauss_1, Ts_1)

            state_pred_gauss_2 = solution.ekf.EKF.predict(
                self_2, state_upd_prev_gauss_2, Ts_2)

            assert compare(state_pred_gauss_1, state_pred_gauss_2)

            assert compare(self_1, self_2)
            assert compare(state_upd_prev_gauss_1, state_upd_prev_gauss_2)
            assert compare(Ts_1, Ts_2)

    def test_output__EKF_predict_measurement(self, test_data):
        for finput in test_data["ekf.EKF.predict_measurement"]:
            params = tuple(finput.values())

            self_1, state_pred_gauss_1 = deepcopy(params)

            self_2, state_pred_gauss_2 = deepcopy(params)

            measure_pred_gauss_1 = ekf.EKF.predict_measurement(
                self_1, state_pred_gauss_1)

            measure_pred_gauss_2 = solution.ekf.EKF.predict_measurement(
                self_2, state_pred_gauss_2)

            assert compare(measure_pred_gauss_1, measure_pred_gauss_2)

            assert compare(self_1, self_2)
            assert compare(state_pred_gauss_1, state_pred_gauss_2)

    def test_output__EKF_update(self, test_data):
        for finput in test_data["ekf.EKF.update"]:
            params = tuple(finput.values())

            self_1, z_1, state_pred_gauss_1, measurement_gauss_1 = deepcopy(
                params)

            self_2, z_2, state_pred_gauss_2, measurement_gauss_2 = deepcopy(
                params)

            state_upd_gauss_1 = ekf.EKF.update(
                self_1, z_1, state_pred_gauss_1, measurement_gauss_1)

            state_upd_gauss_2 = solution.ekf.EKF.update(
                self_2, z_2, state_pred_gauss_2, measurement_gauss_2)

            assert compare(state_upd_gauss_1, state_upd_gauss_2)

            assert compare(self_1, self_2)
            assert compare(z_1, z_2)
            assert compare(state_pred_gauss_1, state_pred_gauss_2)
            assert compare(measurement_gauss_1, measurement_gauss_2)

    def test_output__EKF_step_with_info(self, test_data):
        for finput in test_data["ekf.EKF.step_with_info"]:
            params = tuple(finput.values())

            self_1, state_upd_prev_gauss_1, z_1, Ts_1 = deepcopy(params)

            self_2, state_upd_prev_gauss_2, z_2, Ts_2 = deepcopy(params)

            state_pred_gauss_1, measurement_pred_gauss_1, state_upd_gauss_1 = ekf.EKF.step_with_info(
                self_1, state_upd_prev_gauss_1, z_1, Ts_1)

            state_pred_gauss_2, measurement_pred_gauss_2, state_upd_gauss_2 = solution.ekf.EKF.step_with_info(
                self_2, state_upd_prev_gauss_2, z_2, Ts_2)

            assert compare(state_pred_gauss_1, state_pred_gauss_2)
            assert compare(measurement_pred_gauss_1, measurement_pred_gauss_2)
            assert compare(state_upd_gauss_1, state_upd_gauss_2)

            assert compare(self_1, self_2)
            assert compare(state_upd_prev_gauss_1, state_upd_prev_gauss_2)
            assert compare(z_1, z_2)
            assert compare(Ts_1, Ts_2)


class TestSolutionUsage:

    def test_solution_usage__EKF_predict(self, test_data):
        for finput in test_data["ekf.EKF.predict"][:1]:
            params = finput

            solution.used["ekf.EKF.predict"] = False

            ekf.EKF.predict(**params)

            assert not solution.used["ekf.EKF.predict"], "The function uses the solution"

    def test_solution_usage__EKF_predict_measurement(self, test_data):
        for finput in test_data["ekf.EKF.predict_measurement"][:1]:
            params = finput

            solution.used["ekf.EKF.predict_measurement"] = False

            ekf.EKF.predict_measurement(**params)

            assert not solution.used["ekf.EKF.predict_measurement"], "The function uses the solution"

    def test_solution_usage__EKF_update(self, test_data):
        for finput in test_data["ekf.EKF.update"][:1]:
            params = finput

            solution.used["ekf.EKF.update"] = False

            ekf.EKF.update(**params)

            assert not solution.used["ekf.EKF.update"], "The function uses the solution"

    def test_solution_usage__EKF_step_with_info(self, test_data):
        for finput in test_data["ekf.EKF.step_with_info"][:1]:
            params = finput

            solution.used["ekf.EKF.step_with_info"] = False

            ekf.EKF.step_with_info(**params)

            assert not solution.used["ekf.EKF.step_with_info"], "The function uses the solution"


if __name__ == "__main__":
    os.environ["_PYTEST_RAISE"] = "1"
    pytest.main()
