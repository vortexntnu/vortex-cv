
from gaussparams_py2 import MultiVarGaussian
import numpy as np


def test_gate_hypotheses():
    
    # Generate inputs
    z_mean = np.zeros((6,))
    sigma_z = 2*np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
    z = MultiVarGaussian(z_mean, np.diag(sigma_z))
    
    # TODO: add fake predicted measurements and see how the gating works


test_gate_hypotheses()