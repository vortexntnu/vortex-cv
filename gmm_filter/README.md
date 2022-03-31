Simple Kalman Filter for estimating objects in odom frame. Includes a general ekf implementation which can be configured for
use with generic process and sensor model.

Kalman Filter:

state vector:       x = [x, y, z, phi, theta, psi]
measurement vector: y = [x, y, z, phi, theta, psi]

process model:     x_dot = 0 + v
measurement model: y = I_6x6 * x


Important Files in ekf_python2:

dynamicmodels_py2.py: contains a parent class for dynamic models and as many different dynamic models as need to be defined 
as subclasses

measurementmodels_py2.py: contains a parent class for sensor models and as many different sensor models as need to be defined 
as subclasses

ekf_py2.py: contains a class implementing the ekf equations. In the case of linear model this will collapse to a standard Kalman Filter

gaussparams_py2.py: contains a class which implements multivariate gaussians

Pip3/pip requirement:
pip install dataclasses



## Work-flow pseudocode for the algorithm:
###ToDo: change this pseudocode for a flow-chart

global (self) variables:

    boost_prob: the probability boost a hypothesis gets by getting associated to a measurement. This is a tuning parameter.
    
    gmm_weights: list of length N containing the probabilities of hypotheses:
        gmm_weights = [w_H0, w_H1, ... , w_HN-1]
    
    active_hypotheses: list of MultiVarGauss which are the active hypotheses

    active_hypotheses_count: the number of hypotheses which are not the null hypothesis
        active_hypotheses_count = len(gmm_weights) - 1
    

0) Measurement comes in

1) Gate function: takes in a measurement (MultiVar Gaussian), returns a list of gated indices
    How the fuck was gating done again?????

2) Associate measurement: takes in a list of gated indices, returns two MultiVar Gaussians and an ass_ind

    if gated = 0:
        associate with a new KF initialized at the measurement with the sensor R
    
    if gated = 1:
        associate with that one
    
    if gated = 2 or more:
        merge mixture of these indices into a single one
        give it weight that is sum of original weights

        associate with that mixture

    
3) Update associated measurement
    
    if associated mean = measurement mean
        updated = associated
        go to weight update
    else
        updated = kf update with associated as prev and measurement as z
        go to weight update
    
4) Weight update

    if updated mean = measurement mean:
        weight append initialization weight (tuning parameter)
        normalize weights

    else 
        weigth[ass_ind] += weight boost (tuning parameter)'
        normalize weights

5) Decide convergence
    if there is a weight over 0.95 %
        terminate node and publish the hypothesis
    else
        publish number of hypotheses (maybe for auto, ASK FINN AND T-MAN !!!)
        keep going
        