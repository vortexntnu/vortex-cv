GMF scheme for evaluating detection validity and obtaining robust search.

![Rsz_1gmf_bruh](https://user-images.githubusercontent.com/69768720/167249805-1a8c74ae-1f0c-473b-ab07-197ca5600d53.jpg)

##Description of the Algorithm:

### During Search:
Perform the GMF Workflow. The model for the KF updating the hypotheses is an LTV 3 dimensional model where positions in odom are states. The measurements are modeled in camera frame.

    A measurement comes in, assumed a Gaussian. Gate the measurements through Mahalanobis Distance.
    Associate measurement to a hypothesis:
        If there is only 1 hypothesis within gate, associate with that.
        If there are more than one, perform mixture reduction and associate with the resultant Gaussian.
        If there are no measurements, instantiate a new hypothesis.
    Update Associated hypothesis with measurement through an EKF step.
    Update hypotheses probabilities by boosting the associated one and normalizing the probabilities vector to sum to 1.
    Evaluate the convergence of the scheme.

The GMF scheme is evaluated in step 5 above in the following way. If a hypothesis goes over a convergence threshold, say 85%, it is declared a reliable detection, the bool transition_sc is set to True and we move into the converge state.

### During Converge:
In converge state, use a 6 DoF EKF which has the position and euler angles of the object in odom as states. The positional measurement is still modeled in camera frame, while the orientation is mapped to odom. The filter is instantiated with the hypothesis which was declared convergent in the GMF algorithm, and is updated from there with incoming measurements. The convergence of the EKF is evaluated. If the norm of the covariance matrix does not go under a certain threshold in a certain number of updates, the original GMF convergence is considered a mistake, and the bool transition_isFucked is set True (isFucked takes fsm from converge or execute back to search). If the EKF converges, the bool transition_x is set True, at which point execution is happening. 

### During Execute:
Do Nothing

### Kalman Filters:

#### In search:
state vector:       x = [x_odom, y_odom, z_odom]
measurement vector: y = [x_cam, y_cam, z_cam]

process model:     x_dot = 0
measurement model: y = Rot_cw * x + v

#### In converge:
state vector:       x = [x_odom, y_odom, z_odom, phi, theta, psi]
measurement vector: y = [x_cam, y_cam, z_cam, phi, theta, psi]

process model:     x_dot = 0
measurement model: y = diag(Rot_cw, I_3x3) * x + v

Rot_cw is the SO(3) matrix rotating a vector from w(orld = odom) to c(amera) frame.

### Important Files in ekf_python2:

dynamicmodels_py2.py: contains a parent class for dynamic models and as many different dynamic models as need to be defined 
as subclasses

measurementmodels_py2.py: contains a parent class for sensor models and as many different sensor models as need to be defined 
as subclasses

ekf_py2.py: contains a class implementing the ekf equations. In the case of linear model this will collapse to a standard Kalman Filter

gaussparams_py2.py: contains a class which implements multivariate gaussians


