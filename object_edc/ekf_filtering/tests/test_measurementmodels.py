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
import measurementmodels  # nopep8


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

    def test_output__CartesianPosition2D_h(self, test_data):
        for finput in test_data["measurementmodels.CartesianPosition2D.h"]:
            params = tuple(finput.values())

            self_1, x_1 = deepcopy(params)

            self_2, x_2 = deepcopy(params)

            x_h_1 = measurementmodels.CartesianPosition2D.h(self_1, x_1)

            x_h_2 = solution.measurementmodels.CartesianPosition2D.h(
                self_2, x_2)

            assert compare(x_h_1, x_h_2)

            assert compare(self_1, self_2)
            assert compare(x_1, x_2)

    def test_output__CartesianPosition2D_H(self, test_data):
        for finput in test_data["measurementmodels.CartesianPosition2D.H"]:
            params = tuple(finput.values())

            self_1, x_1 = deepcopy(params)

            self_2, x_2 = deepcopy(params)

            H_1 = measurementmodels.CartesianPosition2D.H(self_1, x_1)

            H_2 = solution.measurementmodels.CartesianPosition2D.H(self_2, x_2)

            assert compare(H_1, H_2)

            assert compare(self_1, self_2)
            assert compare(x_1, x_2)

    def test_output__CartesianPosition2D_R(self, test_data):
        for finput in test_data["measurementmodels.CartesianPosition2D.R"]:
            params = tuple(finput.values())

            self_1, x_1 = deepcopy(params)

            self_2, x_2 = deepcopy(params)

            R_1 = measurementmodels.CartesianPosition2D.R(self_1, x_1)

            R_2 = solution.measurementmodels.CartesianPosition2D.R(self_2, x_2)

            assert compare(R_1, R_2)

            assert compare(self_1, self_2)
            assert compare(x_1, x_2)


class TestSolutionUsage:

    def test_solution_usage__CartesianPosition2D_h(self, test_data):
        for finput in test_data["measurementmodels.CartesianPosition2D.h"][:1]:
            params = finput

            solution.used["measurementmodels.CartesianPosition2D.h"] = False

            measurementmodels.CartesianPosition2D.h(**params)

            assert not solution.used["measurementmodels.CartesianPosition2D.h"], "The function uses the solution"

    def test_solution_usage__CartesianPosition2D_H(self, test_data):
        for finput in test_data["measurementmodels.CartesianPosition2D.H"][:1]:
            params = finput

            solution.used["measurementmodels.CartesianPosition2D.H"] = False

            measurementmodels.CartesianPosition2D.H(**params)

            assert not solution.used["measurementmodels.CartesianPosition2D.H"], "The function uses the solution"

    def test_solution_usage__CartesianPosition2D_R(self, test_data):
        for finput in test_data["measurementmodels.CartesianPosition2D.R"][:1]:
            params = finput

            solution.used["measurementmodels.CartesianPosition2D.R"] = False

            measurementmodels.CartesianPosition2D.R(**params)

            assert not solution.used["measurementmodels.CartesianPosition2D.R"], "The function uses the solution"


if __name__ == "__main__":
    os.environ["_PYTEST_RAISE"] = "1"
    pytest.main()
