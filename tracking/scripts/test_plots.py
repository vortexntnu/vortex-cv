import sys

sys.path.insert(0, "/home/hannahcl/Documents/vortex/monkey_tracking/data_generation")
from scenarios import BaseScenario
from utility import time_from_step

from track_manager_single_track import TrackStatus

import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np


"""
3 plots to choose from: 
Plot position of sigle track, 
Plot position and velocity of single track, or
Plot position and velocity of multiple tracks. 


"""


def plot_pos_single_track(
    scenario,
    measurements,
    ground_truths,
    tentative_estimates,
    conf_estimates,
    tentative_del_estimates,
    estimate_status,
    wait_for_btn_press,
):

    plt.ion()

    end_time = time_from_step(scenario.k, scenario.config.dt)
    min_alpha = 0.5

    # Ground truths
    for target in ground_truths:
        track = target.track
        x = track[:, 1]
        y = track[:, 2]
        # alpha = 1 - np.vectorize(max)(min_alpha, 1 - track[:, 5] / end_time)
        alpha = None
        plt.scatter(x, y, alpha=alpha)

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

    plt.gca().set_aspect("equal")
    plt.grid()
    plt.show()

    # Measurements
    k_ten = 0
    k_conf = 0
    k_del = 0
    for k in range(len(measurements)):

        # opacity based on time
        for measurement in measurements[k]:
            alpha = 1 - max(min_alpha, 1 - measurement.t / end_time)
            color = "r" if measurement.is_clutter else "k"

            plt.scatter(
                measurement.pos[0],
                measurement.pos[1],
                marker="x",
                color=color,
                alpha=alpha,
            )

        if estimate_status[k] == TrackStatus.tentative_confirm:
            for tentative_estimate in tentative_estimates[k_ten]:
                # alpha = 1 - max(min_alpha, 1 - measurement.t / end_time)
                color = "y"

                plt.scatter(
                    tentative_estimate[0],
                    tentative_estimate[1],
                    marker="+",
                    color=color,
                    # alpha=alpha,
                )
            k_ten += 1

        if estimate_status[k] == TrackStatus.confirmed:
            estimates_at_t = conf_estimates[k_conf]
            color = "g"

            plt.scatter(
                estimates_at_t[0],
                estimates_at_t[1],
                marker="+",
                color=color,
                # alpha=alpha,
            )
            k_conf += 1

        if estimate_status[k] == TrackStatus.tentative_delete:
            del_estimates_at_t = tentative_del_estimates[k_del]
            color = "r"
            plt.scatter(
                del_estimates_at_t[0],
                del_estimates_at_t[1],
                marker="+",
                color=color,
                # alpha=alpha,
            )
            k_del += 1

        plt.draw()
        if wait_for_btn_press:
            plt.waitforbuttonpress()
        else:
            plt.pause(0.01)

    plt.ioff()
    plt.show()


def plot_pos_and_vel_single_track(
    scenario,
    measurements,
    ground_truths,
    tentative_estimates,
    conf_estimates,
    tentative_del_estimates,
    estimate_status,
    wait_for_btn_press,
):

    # ----------init pos plot
    plt.subplot(1, 2, 1)
    plt.ion()

    end_time = time_from_step(scenario.k, scenario.config.dt)
    min_alpha = 0.5

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

    plt.gca().set_aspect("equal")
    plt.grid()
    plt.show()

    # Ground truths
    for target in ground_truths:
        track = target.track

        x = track[:, 1]
        y = track[:, 2]
        # alpha = 1 - np.vectorize(max)(min_alpha, 1 - track[:, 5] / end_time)
        alpha = None
        plt.scatter(x, y, alpha=alpha)

    # -----------init vel plot
    plt.subplot(1, 2, 2)
    plt.ion()

    plt.xlabel("x'[m/s]")
    plt.ylabel("y' [m/s]")

    plt.gca().set_aspect("equal")
    plt.grid()
    plt.show()

    # Ground truths
    for target in ground_truths:
        track = target.track

        x = track[:, 3]
        y = track[:, 4]
        alpha = 1 - np.vectorize(max)(min_alpha, 1 - track[:, 5] / end_time)
        plt.scatter(x, y, alpha=alpha)

    # Measurements
    k_ten = 0
    k_conf = 0
    k_del = 0
    for k in range(len(measurements)):

        # opacity based on time
        for measurement in measurements[k]:
            alpha = 1 - max(min_alpha, 1 - measurement.t / end_time)
            color = "r" if measurement.is_clutter else "k"

            plt.subplot(1, 2, 1)
            plt.scatter(
                measurement.pos[0],
                measurement.pos[1],
                marker="x",
                color=color,
                alpha=alpha,
            )

        if estimate_status[k] == TrackStatus.tentative_confirm:
            for tentative_estimate in tentative_estimates[k_ten]:
                # alpha = 1 - max(min_alpha, 1 - measurement.t / end_time)
                color = "y"

                plt.subplot(1, 2, 1)
                plt.scatter(
                    tentative_estimate[0],
                    tentative_estimate[1],
                    marker="+",
                    color=color,
                    # alpha=alpha,
                )
            k_ten += 1

        if estimate_status[k] == TrackStatus.confirmed:
            estimates_at_t = conf_estimates[k_conf]
            color = "g"

            plt.subplot(1, 2, 1)
            plt.scatter(
                estimates_at_t[0],
                estimates_at_t[1],
                marker="+",
                color=color,
                # alpha=alpha,
            )

            plt.subplot(1, 2, 2)
            plt.scatter(
                estimates_at_t[2],
                estimates_at_t[3],
                color=color,
                # alpha=alpha,
            )
            k_conf += 1

        if estimate_status[k] == TrackStatus.tentative_delete:
            del_estimates_at_t = tentative_del_estimates[k_del]
            color = "r"

            plt.subplot(1, 2, 1)
            plt.scatter(
                del_estimates_at_t[0],
                del_estimates_at_t[1],
                marker="+",
                color=color,
                # alpha=alpha,
            )
            k_del += 1

        plt.subplot(1, 2, 1)
        plt.draw()
        if wait_for_btn_press:
            plt.waitforbuttonpress()
        else:
            plt.pause(0.01)

        plt.subplot(1, 2, 2)
        plt.draw()
        if wait_for_btn_press:
            plt.waitforbuttonpress()
        else:
            plt.pause(0.0001)

    plt.ioff()
    plt.show()


def plot_pos_and_vel_multiple_tracks(
    scenario,
    measurements,
    ground_truths,
    tentative_estimates,
    conf_estimates,
    wait_for_btn_press,
):

    # ----------init pos plot
    plt.subplot(1, 2, 1)
    plt.ion()

    end_time = time_from_step(scenario.k, scenario.config.dt)
    min_alpha = 0.5

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

    plt.gca().set_aspect("equal")
    plt.grid()
    plt.show()

    # Ground truths
    for target in ground_truths:
        track = target.track

        x = track[:, 1]
        y = track[:, 2]
        # alpha = 1 - np.vectorize(max)(min_alpha, 1 - track[:, 5] / end_time)
        alpha = None
        plt.scatter(x, y, alpha=alpha)

    # -----------init vel plot
    plt.subplot(1, 2, 2)
    plt.ion()

    plt.xlabel("x'[m]")
    plt.ylabel("y' [m]")

    plt.gca().set_aspect("equal")
    plt.grid()
    plt.show()

    # Ground truths
    for target in ground_truths:
        track = target.track

        x = track[:, 3]
        y = track[:, 4]
        alpha = 1 - np.vectorize(max)(min_alpha, 1 - track[:, 5] / end_time)
        plt.scatter(x, y, alpha=alpha)

    # ------------- plot each step
    for k in range(len(measurements)):

        for measurement in measurements[k]:
            alpha = 1 - max(min_alpha, 1 - measurement.t / end_time)
            color = "r" if measurement.is_clutter else "k"

            plt.subplot(1, 2, 1)
            plt.scatter(
                measurement.pos[0],
                measurement.pos[1],
                marker="x",
                color=color,
                alpha=alpha,
            )

        for tentative_estimate in tentative_estimates[k]:
            # alpha = 1 - max(min_alpha, 1 - measurement.t / end_time)
            color = "y"

            plt.subplot(1, 2, 1)
            plt.scatter(
                tentative_estimate[0],
                tentative_estimate[1],
                marker="+",
                color=color,
                # alpha=alpha,
            )

            plt.subplot(1, 2, 2)
            plt.scatter(
                tentative_estimate[2],
                tentative_estimate[3],
                color=color,
                # alpha=alpha,
            )

        for conf_estimate in conf_estimates[k]:

            color = "g"

            plt.subplot(1, 2, 1)
            plt.scatter(
                conf_estimate[0],
                conf_estimate[1],
                marker="+",
                color=color,
                # alpha=alpha,
            )

            plt.subplot(1, 2, 2)
            plt.scatter(
                conf_estimate[2],
                conf_estimate[3],
                color=color,
                # alpha=alpha,
            )

        # draw update to bloth plots
        plt.subplot(1, 2, 1)
        plt.draw()
        if wait_for_btn_press:
            plt.waitforbuttonpress()
        else:
            plt.pause(0.01)

        plt.subplot(1, 2, 2)
        plt.draw()
        if wait_for_btn_press:
            plt.waitforbuttonpress()
        else:
            plt.pause(0.0001)

    plt.ioff()
    plt.show()
