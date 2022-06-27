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
import dynamicmodels  # nopep8


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

    def test_output__WhitenoiseAcceleration2D_f(self, test_data):
        for finput in test_data["dynamicmodels.WhitenoiseAcceleration2D.f"]:
            params = tuple(finput.values())

            self_1, x_1, Ts_1 = deepcopy(params)

            self_2, x_2, Ts_2 = deepcopy(params)

            x_kp1_1 = dynamicmodels.WhitenoiseAcceleration2D.f(
                self_1, x_1, Ts_1)

            x_kp1_2 = solution.dynamicmodels.WhitenoiseAcceleration2D.f(
                self_2, x_2, Ts_2)

            assert compare(x_kp1_1, x_kp1_2)

            assert compare(self_1, self_2)
            assert compare(x_1, x_2)
            assert compare(Ts_1, Ts_2)

    def test_output__WhitenoiseAcceleration2D_F(self, test_data):
        for finput in test_data["dynamicmodels.WhitenoiseAcceleration2D.F"]:
            params = tuple(finput.values())

            self_1, x_1, Ts_1 = deepcopy(params)

            self_2, x_2, Ts_2 = deepcopy(params)

            F_1 = dynamicmodels.WhitenoiseAcceleration2D.F(self_1, x_1, Ts_1)

            F_2 = solution.dynamicmodels.WhitenoiseAcceleration2D.F(
                self_2, x_2, Ts_2)

            assert compare(F_1, F_2)

            assert compare(self_1, self_2)
            assert compare(x_1, x_2)
            assert compare(Ts_1, Ts_2)

    def test_output__WhitenoiseAcceleration2D_Q(self, test_data):
        for finput in test_data["dynamicmodels.WhitenoiseAcceleration2D.Q"]:
            params = tuple(finput.values())

            self_1, x_1, Ts_1 = deepcopy(params)

            self_2, x_2, Ts_2 = deepcopy(params)

            Q_1 = dynamicmodels.WhitenoiseAcceleration2D.Q(self_1, x_1, Ts_1)

            Q_2 = solution.dynamicmodels.WhitenoiseAcceleration2D.Q(
                self_2, x_2, Ts_2)

            assert compare(Q_1, Q_2)

            assert compare(self_1, self_2)
            assert compare(x_1, x_2)
            assert compare(Ts_1, Ts_2)


class TestSolutionUsage:

    def test_solution_usage__WhitenoiseAcceleration2D_f(self, test_data):
        for finput in test_data["dynamicmodels.WhitenoiseAcceleration2D.f"][:1]:
            params = finput

            solution.used["dynamicmodels.WhitenoiseAcceleration2D.f"] = False

            dynamicmodels.WhitenoiseAcceleration2D.f(**params)

            assert not solution.used["dynamicmodels.WhitenoiseAcceleration2D.f"], "The function uses the solution"

    def test_solution_usage__WhitenoiseAcceleration2D_F(self, test_data):
        for finput in test_data["dynamicmodels.WhitenoiseAcceleration2D.F"][:1]:
            params = finput

            solution.used["dynamicmodels.WhitenoiseAcceleration2D.F"] = False

            dynamicmodels.WhitenoiseAcceleration2D.F(**params)

            assert not solution.used["dynamicmodels.WhitenoiseAcceleration2D.F"], "The function uses the solution"

    def test_solution_usage__WhitenoiseAcceleration2D_Q(self, test_data):
        for finput in test_data["dynamicmodels.WhitenoiseAcceleration2D.Q"][:1]:
            params = finput

            solution.used["dynamicmodels.WhitenoiseAcceleration2D.Q"] = False

            dynamicmodels.WhitenoiseAcceleration2D.Q(**params)

            assert not solution.used["dynamicmodels.WhitenoiseAcceleration2D.Q"], "The function uses the solution"


if __name__ == "__main__":
    os.environ["_PYTEST_RAISE"] = "1"
    pytest.main()
