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
import analysis  # nopep8


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

    def test_output__get_NIS(self, test_data):
        for finput in test_data["analysis.get_NIS"]:
            params = tuple(finput.values())

            z_pred_gauss_1, z_1 = deepcopy(params)

            z_pred_gauss_2, z_2 = deepcopy(params)

            NIS_1 = analysis.get_NIS(z_pred_gauss_1, z_1)

            NIS_2 = solution.analysis.get_NIS(z_pred_gauss_2, z_2)

            assert compare(NIS_1, NIS_2)

            assert compare(z_pred_gauss_1, z_pred_gauss_2)
            assert compare(z_1, z_2)

    def test_output__get_NEES(self, test_data):
        for finput in test_data["analysis.get_NEES"]:
            params = tuple(finput.values())

            x_gauss_1, x_gt_1 = deepcopy(params)

            x_gauss_2, x_gt_2 = deepcopy(params)

            NEES_1 = analysis.get_NEES(x_gauss_1, x_gt_1)

            NEES_2 = solution.analysis.get_NEES(x_gauss_2, x_gt_2)

            assert compare(NEES_1, NEES_2)

            assert compare(x_gauss_1, x_gauss_2)
            assert compare(x_gt_1, x_gt_2)

    def test_output__get_ANIS(self, test_data):
        for finput in test_data["analysis.get_ANIS"]:
            params = tuple(finput.values())

            z_pred_gauss_data_1, z_data_1 = deepcopy(params)

            z_pred_gauss_data_2, z_data_2 = deepcopy(params)

            ANIS_1 = analysis.get_ANIS(z_pred_gauss_data_1, z_data_1)

            ANIS_2 = solution.analysis.get_ANIS(z_pred_gauss_data_2, z_data_2)

            assert compare(ANIS_1, ANIS_2)

            assert compare(z_pred_gauss_data_1, z_pred_gauss_data_2)
            assert compare(z_data_1, z_data_2)

    def test_output__get_ANEES(self, test_data):
        for finput in test_data["analysis.get_ANEES"]:
            params = tuple(finput.values())

            x_upd_gauss_data_1, x_gt_data_1 = deepcopy(params)

            x_upd_gauss_data_2, x_gt_data_2 = deepcopy(params)

            ANEES_1 = analysis.get_ANEES(x_upd_gauss_data_1, x_gt_data_1)

            ANEES_2 = solution.analysis.get_ANEES(
                x_upd_gauss_data_2, x_gt_data_2)

            assert compare(ANEES_1, ANEES_2)

            assert compare(x_upd_gauss_data_1, x_upd_gauss_data_2)
            assert compare(x_gt_data_1, x_gt_data_2)


class TestSolutionUsage:

    def test_solution_usage__get_NIS(self, test_data):
        for finput in test_data["analysis.get_NIS"][:1]:
            params = finput

            solution.used["analysis.get_NIS"] = False

            analysis.get_NIS(**params)

            assert not solution.used["analysis.get_NIS"], "The function uses the solution"

    def test_solution_usage__get_NEES(self, test_data):
        for finput in test_data["analysis.get_NEES"][:1]:
            params = finput

            solution.used["analysis.get_NEES"] = False

            analysis.get_NEES(**params)

            assert not solution.used["analysis.get_NEES"], "The function uses the solution"

    def test_solution_usage__get_ANIS(self, test_data):
        for finput in test_data["analysis.get_ANIS"][:1]:
            params = finput

            solution.used["analysis.get_ANIS"] = False

            analysis.get_ANIS(**params)

            assert not solution.used["analysis.get_ANIS"], "The function uses the solution"

    def test_solution_usage__get_ANEES(self, test_data):
        for finput in test_data["analysis.get_ANEES"][:1]:
            params = finput

            solution.used["analysis.get_ANEES"] = False

            analysis.get_ANEES(**params)

            assert not solution.used["analysis.get_ANEES"], "The function uses the solution"


if __name__ == "__main__":
    os.environ["_PYTEST_RAISE"] = "1"
    pytest.main()
