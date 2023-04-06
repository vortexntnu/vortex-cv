import sys
from test_pdaf_test_setup import PDAFTester

import yaml

from track_manager_multiple_tracks import MultiTargetTrackManager, TrackStatus
import test_plots

import numpy as np

# ---- modify here:
PATH_TO_DATA_GENERATION_REP = "/home/hannahcl/Documents/vortex/monkey_tracking"
PATH_TO_CONFIG_TRACKING_SYS = (
    "/home/hannahcl/Documents/vortex/ros_ws/src/vortex-cv/tracking/config")

sys.path.insert(0, PATH_TO_DATA_GENERATION_REP + "/data_generation")
sys.path.insert(0, PATH_TO_DATA_GENERATION_REP + "/config")

from scenarios import BaseScenario
from load_config import load_yaml_into_dotdict


def data_generation():

    config = load_yaml_into_dotdict(PATH_TO_DATA_GENERATION_REP +
                                    "/config/scenario.yaml")

    scenario = BaseScenario(config)

    measurements, ground_truths = scenario.run()

    return scenario, measurements, ground_truths


def test_cb():

    with open(
            PATH_TO_CONFIG_TRACKING_SYS + "/config_traking_sys.yaml",
            "r",
    ) as stream:
        config_loaded = yaml.safe_load(stream)

    manager = MultiTargetTrackManager(config_loaded)
    pdafTester = PDAFTester()

    x1 = 5
    y1 = 7

    x2 = 0
    y2 = -7
    n_timesteps = 15
    time_step = 0.1

    for i in range(n_timesteps):
        print("\n timestep", i, "\n")
        o_arr1 = pdafTester.create_observations_for_one_timestep(x1, y1)
        o_arr2 = pdafTester.create_observations_for_one_timestep(x2, y2)
        o_arr = np.concatenate((o_arr1, o_arr2))
        manager.step_once(o_arr, time_step)

        for track in manager.tentative_tracks:
            print("\n tentive tracks \n")
            print("state: ", track.pdaf.prior_state_estimate.mean[:2])
            print("n: ", track.n, "m: ", track.m)

        for track in manager.confirmed_tracks:
            print("\n confirmed \n")
            print("state: ", track.pdaf.prior_state_estimate.mean[:2])
            print("n: ", track.n, "m: ", track.m)

    for track in manager.confirmed_tracks:
        print("final estimates: ", track.pdaf.posterior_state_estimate.mean)


# @pytest.mark.plot
def test_plot():
    with open(
            PATH_TO_CONFIG_TRACKING_SYS + "/config_traking_sys.yaml",
            "r",
    ) as stream:
        config_loaded = yaml.safe_load(stream)

    wait_for_btn_press = False

    manager = MultiTargetTrackManager(config_loaded)

    scenario, measurements, ground_truths = data_generation()

    tentative_estimates = []
    conf_estimates = []

    time_step = 0.1

    for i in range(len(measurements)):

        o_list = []
        for o in measurements[i]:
            o_list.append(o.pos)
        o_arr = np.array(o_list)

        # update
        manager.step_once(o_arr, time_step)

        # add updates to lists that will be plotted

        # add tentative tracks
        last_addition_to_tentative_tracks = []
        for track in manager.tentative_tracks:
            last_addition_to_tentative_tracks.append(
                track.pdaf.posterior_state_estimate.mean)
        tentative_estimates.append(last_addition_to_tentative_tracks)

        # add confirmed tracks
        last_addition_to_conf_tracks = []
        for track in manager.confirmed_tracks:
            last_addition_to_conf_tracks.append(
                track.pdaf.posterior_state_estimate.mean)
        conf_estimates.append(last_addition_to_tentative_tracks)

    test_plots.plot_pos_and_vel_multiple_tracks(
        scenario,
        measurements,
        ground_truths,
        tentative_estimates,
        conf_estimates,
        wait_for_btn_press,
    )

    assert True
