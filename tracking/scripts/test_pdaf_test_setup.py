import numpy as np
from dataclasses import dataclass
import yaml

from pdaf import PDAF

# Modify here:
PATH_TO_CONFIG_TRACKING_SYS = (
    "/home/hannahcl/Documents/vortex/ros_ws/src/vortex-cv/tracking/config"
)


class PDAFTester:
    def __init__(self):

        self.pdaf = self.create_pdaf_instance()

    def create_pdaf_instance(self):

        with open(
            PATH_TO_CONFIG_TRACKING_SYS + "/config_traking_sys.yaml",
            "r",
        ) as stream:
            config_loaded = yaml.safe_load(stream)

        pdaf = PDAF(config_loaded)

        return pdaf

    def create_observations_for_one_timestep(self, x, y):

        n_obs = 5  # np.random.randint(0, 10)

        obs = np.ndarray((n_obs, 2), dtype=float)

        # Add obs that are scaterd far apart
        for i in range(n_obs):
            obs[i, 0] = x + np.random.randn(1) * 1
            obs[i, 1] = y + np.random.randn(1) * 1

        # Add obs that corresponds to the acctual track (1-p_no_match)*100 prosent of the time.

        # Can replace with:
        # random_int = np.random.randint(0, 100)
        # if (random_int < 100 * (1 - self.pdaf.p_no_match)) and (n_obs > 0):
        if True:
            obs[-1, 0] = (
                x + np.random.randn(1) * self.pdaf.measurment_noise.covariance[0, 0]
            )
            obs[-1, 1] = (
                y + np.random.randn(1) * self.pdaf.measurment_noise.covariance[1, 1]
            )

        return obs

    def create_observations_for_one_timestep_simple_version(self, x, y):

        n_obs = np.random.randint(0, 10)

        obs = np.ndarray((n_obs, 2), dtype=float)
        for i in range(n_obs):
            obs[i, 0] = (
                x + np.random.randn(1) * self.pdaf.measurment_noise.covariance[0, 0]
            )
            obs[i, 1] = (
                y + np.random.randn(1) * self.pdaf.measurment_noise.covariance[1, 1]
            )

        return obs
