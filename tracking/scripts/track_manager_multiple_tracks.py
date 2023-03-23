import numpy as np
from enum import Enum
from typing import List
from dataclasses import dataclass

from pdaf import PDAF

"""

Track manager - Manage multiple object tracking based on M/N method. Assuming tracks are sufficiently far apart so that pdaf is sufficient. 

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


class MultiTargetTrackManager:
    def __init__(self, config):

        self.config = config

        self.prev_observations: List[np.ndarray] = []
        self.observations_not_incorporated_in_track: List[np.ndarray] = []
        self.tentative_tracks: List[PDAF2MN] = []
        self.confirmed_tracks: List[PDAF2MN] = []

        self.N = config["manager"]["N"]
        self.M = config["manager"]["M"]

        self.max_vel = config["manager"]["max_vel"]  # [m/s]
        self.initial_measurement_covariance = config["manager"][
            "initial_measurement_covariance"
        ]

        self.time_step = 0

    def step_once(self, o_arr, time_step):
        self.time_step = time_step
        self.observations_not_incorporated_in_track = o_arr.tolist()

        #print('\n ---- step once --------- \n')

        #print('obs pre udapte conf tracks \n', self.observations_not_incorporated_in_track)

        self.update_status_on_confirmed_tracks()

        #print('obs post udapte conf tracks \n', self.observations_not_incorporated_in_track)

        self.remove_o_incorporated_in_tracks(self.confirmed_tracks)

        self.update_status_on_tentative_tracks()

        self.remove_o_incorporated_in_tracks(self.tentative_tracks)

        self.add_tentative_tracks()

        self.prev_observations = self.observations_not_incorporated_in_track

        # print('traking', len(self.confirmed_tracks), 'objects')
        
        # for track in self.tentative_tracks:
        #     print("status", track.track_status)
        #     print("state: ", track.pdaf.prior_state_estimate.mean[:2])
        #     print("n: ", track.n, "m: ", track.m)

        # for track in self.confirmed_tracks:
        #     print("status", track.track_status)
        #     print("state: ", track.pdaf.prior_state_estimate.mean[:2])
        #     print("n: ", track.n, "m: ", track.m)




    def update_status_on_confirmed_tracks(self):

        for track in self.confirmed_tracks:
            track.pdaf.step_once(
                self.observations_not_incorporated_in_track, self.time_step
            )

            # #Remove observation incorporated in this confirmed track
            # for o in track.pdaf.o_within_gate_arr:
            #     for obs in self.observations_not_incorporated_in_track:
            #         if (abs(o[0] - obs[0]) < 0.01) and (abs(o[1] - obs[1]) < 0.01):
            #             self.observations_not_incorporated_in_track.remove(obs)  

            if track.track_status == TrackStatus.confirmed:

                if len(track.pdaf.o_within_gate_arr) == 0:
                    track.track_status = TrackStatus.tentative_delete
                    track.m = 0
                    track.n = 0

            elif track.track_status == TrackStatus.tentative_delete:
                track.m += 1

                if len(track.pdaf.o_within_gate_arr) > 0:
                    track.n += 1

                if track.n == self.N:
                    track.track_status = TrackStatus.confirmed

                elif track.m == self.M:
                    self.confirmed_tracks.remove(track)

    def update_status_on_tentative_tracks(self):
        for track in self.tentative_tracks:

            track.n, track.m = self.update_confirmation_count(
                track, self.observations_not_incorporated_in_track
            )

            track.pdaf.step_once(
                self.observations_not_incorporated_in_track, self.time_step
            )

            # #remove obs incorporated in track
            # temp_list = self.observations_not_incorporated_in_track

            # for i, o in enumerate(temp_list):

            #     dist = np.sqrt(
            #         (o[0] - track.pdaf.prior_state_estimate.mean[0]) ** 2
            #         + (o[1] - track.pdaf.prior_state_estimate.mean[1]) ** 2
            #     )
            #     if (
            #         dist
            #         < self.max_vel * self.time_step
            #         + self.initial_measurement_covariance
            #     ):
            #         temp_list.pop(i)

            # self.observations_not_incorporated_in_track = temp_list

            if track.n == self.N:
                track.track_status = TrackStatus.confirmed
                self.confirmed_tracks.append(track)
                self.tentative_tracks.remove(track)

            elif track.m == self.M:
                self.tentative_tracks.remove(track)

    def remove_o_incorporated_in_tracks(self, tracks):

        temp_list = self.observations_not_incorporated_in_track

        for track in tracks:
            for i, o in enumerate(temp_list):

                dist = np.sqrt(
                    (o[0] - track.pdaf.prior_state_estimate.mean[0]) ** 2
                    + (o[1] - track.pdaf.prior_state_estimate.mean[1]) ** 2
                )
                if (
                    dist
                    < self.max_vel * self.time_step
                    + self.initial_measurement_covariance
                ):
                    temp_list.pop(i)

        self.observations_not_incorporated_in_track = temp_list

    def add_tentative_tracks(self):
        for prev_o in self.prev_observations:
            (
                there_are_obs_within_gate,
                an_obs_within_gate,
            ) = self.are_there_observations_inside_max_size_gate(
                prev_o, self.observations_not_incorporated_in_track
            )
            if there_are_obs_within_gate:

                tentative_track = PDAF2MN(self.config)

                tentative_track.pdaf.posterior_state_estimate.mean[
                    0
                ] = an_obs_within_gate[0]
                tentative_track.pdaf.posterior_state_estimate.mean[
                    1
                ] = an_obs_within_gate[1]

                self.tentative_tracks.append(tentative_track)

                #Remove observation incorporated in new tentative track
                self.observations_not_incorporated_in_track.remove(an_obs_within_gate)

    def update_confirmation_count(self, track: PDAF2MN, o_arr):
        m = track.m + 1

        predicted_o = track.pdaf.C @ track.pdaf.prior_state_estimate.mean
        (
            there_are_obs_within_gate,
            an_obs_within_gate,
        ) = self.are_there_observations_inside_max_size_gate(predicted_o, o_arr)
        if there_are_obs_within_gate:
            n = track.n + 1
        else:
            n = track.n

        return n, m

    def are_there_observations_inside_max_size_gate(self, predicted_position, o_arr):

        for o in o_arr:

            dist = np.sqrt(
                (o[0] - predicted_position[0]) ** 2
                + (o[1] - predicted_position[1]) ** 2
            )

            if (
                dist
                < self.max_vel * self.time_step + self.initial_measurement_covariance
            ):
                return True, o

        return False, np.nan
