import numpy as np
from enum import Enum
from typing import List
from dataclasses import dataclass

from pdaf import PDAF
"""

Track manager - Manage single object tracking based on M/N method. 

Implementation based on section 7.4.3 in chapter 7 in "Fundementals in Sensor Fusion" by Brekke, 2021 edition.
"""


class TrackStatus(Enum):
    tentative_confirm = 1
    confirmed = 2
    tentative_delete = 3


class PDAF2MN:

    def __init__(self, config):
        self.pdaf = PDAF(config)
        self.m = 0
        self.n = 0
        self.track_status = TrackStatus.tentative_confirm


class SingleTargetTrackManager:

    def __init__(self, config):
        # subscribe to topic with detections from point cloud
        # publish state of main track if status is confirmed

        self.config = config

        self.prev_observations: List[np.ndarray] = []
        self.observations_not_incorporated_in_track: List[np.ndarray] = []
        self.tentative_tracks: List[PDAF2MN] = []

        self.N_resurrect = config["manager"]["N_resurrect"]
        self.M_resurrect = config["manager"]["M_resurrect"]

        self.N_kill = config["manager"]["N_kill"]
        self.M_kill = config["manager"]["M_resurrect"]

        self.main_track = PDAF2MN(config)

        self.max_vel = config["manager"]["max_vel"]  # [m/s]
        self.initial_measurement_covariance = config["manager"][
            "initial_measurement_covariance"]

        self.time_step = 0

    def step_once(self, o_arr, time_step):
        self.time_step = time_step

        if self.main_track.track_status == TrackStatus.tentative_confirm:

            # print(
            #     "\n ------------ tentative confirm with ",
            #     len(self.tentative_tracks),
            #     " tracks.",
            # )

            # for track in self.tentative_tracks:
            #     print("state: ", track.pdaf.state_pri[:2])
            #     print("n: ", track.n, "m: ", track.m)
            #     # print("S: ", track.S)

            # print("update status on tentative tracks")

            self.update_status_on_tentative_tracks(o_arr)
            self.remove_o_incorporated_in_tracks(o_arr)
            self.add_tentative_tracks()
            self.prev_observations = self.observations_not_incorporated_in_track

        elif self.main_track.track_status == TrackStatus.confirmed:

            # print("\n ------------track still confirmed")
            # print("state: ", self.main_track.pdaf.state_post)

            self.main_track.pdaf.step_once(o_arr, self.time_step)

            if len(self.main_track.pdaf.o_within_gate_arr) == 0:
                self.main_track.track_status = TrackStatus.tentative_delete
                self.main_track.m = 0
                self.main_track.n = 0

        elif self.main_track.track_status == TrackStatus.tentative_delete:

            # print("\n ------------tentative delete")
            # print("state: ", self.main_track.pdaf.state_post)
            # print("n: ", self.main_track.n, "m: ", self.main_track.m)

            self.main_track.pdaf.step_once(o_arr, self.time_step)

            self.main_track.m += 1
            if len(self.main_track.pdaf.o_within_gate_arr) > 0:
                self.main_track.n += 1

            if self.main_track.n == self.N_kill:
                self.main_track.track_status = TrackStatus.confirmed

            elif self.main_track.m == self.M_kill:
                self.main_track.track_status = TrackStatus.tentative_confirm

    def update_status_on_tentative_tracks(self, o_arr):
        for track in self.tentative_tracks:

            track.n, track.m = self.update_confirmation_count(track, o_arr)

            track.pdaf.step_once(o_arr, self.time_step)

            if track.n == self.N_resurrect:
                self.main_track = track
                self.main_track.track_status = TrackStatus.confirmed
                self.tentative_tracks = []
            elif track.m == self.M_resurrect:
                self.tentative_tracks.remove(track)

    def remove_o_incorporated_in_tracks(self, o_arr):
        remaining_o = o_arr.tolist()

        for track in self.tentative_tracks:
            for i, o in enumerate(remaining_o):

                dist = np.sqrt(
                    (o[0] - track.pdaf.prior_state_estimate.mean[0])**2 +
                    (o[1] - track.pdaf.prior_state_estimate.mean[1])**2)
                if (dist < self.max_vel * self.main_track.pdaf.time_step +
                        self.initial_measurement_covariance):
                    remaining_o.pop(i)

        self.observations_not_incorporated_in_track = remaining_o

    def add_tentative_tracks(self):
        for prev_o in self.prev_observations:
            n = self.n_observations_inside_max_size_gate(
                prev_o, self.observations_not_incorporated_in_track)
            if n > 0:

                tentative_track = PDAF2MN(self.config)

                self.tentative_tracks.append(tentative_track)

    def update_confirmation_count(self, track: PDAF2MN, o_arr):
        m = track.m + 1

        predicted_o = track.pdaf.C @ track.pdaf.prior_state_estimate.mean
        if self.n_observations_inside_max_size_gate(predicted_o, o_arr) > 0:
            n = track.n + 1
        else:
            n = track.n

        return n, m

    def n_observations_inside_max_size_gate(self, predicted_position, o_arr):
        n = 0

        for o in o_arr:

            dist = np.sqrt((o[0] - predicted_position[0])**2 +
                           (o[1] - predicted_position[1])**2)

            if (dist < self.max_vel * self.main_track.pdaf.time_step +
                    self.initial_measurement_covariance):
                n += 1

        return n
