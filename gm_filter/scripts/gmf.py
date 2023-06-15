import numpy as np
from ekf_python2.gaussparams_py2 import MultiVarGaussian


## TODO: Finish moving the gmf code here
class GMF:

    def __init__(self, ekf, z):
        self.ekf = ekf
        self.z = z

    def reduce_mixture(self, gated_hypotheses, gated_weights):
        """
        Reduces a Gaussian Mixture to a single Gaussian

        Inputs:     gated_hypotheses - a list of MultiVarGauss who have been gateded with the measurement
                    gated_weights - a list of the corresponding weights which have been gated to the measurement
        
        Outputs:    reduced_hypothesis - a MultiVarGaussian

        """

        M = len(gated_weights)

        reduced_mean = 0
        cov_internal = np.zeros(np.shape(gated_hypotheses[0].cov))
        cov_external = np.zeros(np.shape(gated_hypotheses[0].cov))

        for i in range(M):
            reduced_mean += gated_hypotheses[i].mean * gated_weights[i]

        for i in range(M):
            weight_n = gated_weights[i]
            diff = np.array(gated_hypotheses[i].mean - reduced_mean)

            cov_internal += weight_n * gated_hypotheses[i].cov
            cov_external = weight_n * np.matmul(diff, diff.T)

        reduced_cov = cov_internal + cov_external

        return MultiVarGaussian(reduced_mean, reduced_cov)

    def predict_states(self, active_hypotheses, active_hypotheses_count):
        """
        Predicts y from x
        returns a list of predicted means
        """
        predicted_states = np.empty((active_hypotheses_count, ), dtype=object)

        for i in range(active_hypotheses_count):
            predicted_states[i] = active_hypotheses[i].mean
        return predicted_states

    def predict_measurements(self, states_pred):

        predicted_measurements = np.empty_like(states_pred)
        for i, x_pred in enumerate(states_pred):
            #predicted_measurements[i] = np.matmul(Rot_wc.T, (x_pred[0:3] - pos_wc[0:3]))
            predicted_measurements[i] = self.ekf.sensor_model.h(x_pred)

        return predicted_measurements

    def gate_hypotheses(self, active_hypotheses, gmf_weights, z_gauss,
                        predicted_zs, active_hypotheses_count):
        # Tested, works
        """
        Inputs: z - MultiVarGauss of measurement
                predicted_zs - array of predicted measurement locations

        Outputs: hypothesis indices gated with measurement
        """
        gated_inds = []

        for i in range(active_hypotheses_count):
            mahalanobis_distance = z_gauss.mahalanobis_distance_sq(
                predicted_zs[i])
            if mahalanobis_distance <= self.gate_size_sq:
                gated_inds.append(i)
                #m_distances.append(mahalanobis_distance)
        g = len(gated_inds)
        gated_hypotheses = [active_hypotheses[gated_inds[j]] for j in range(g)]
        gated_weights = np.array(
            [gmf_weights[gated_inds[k] + 1] for k in range(g)])

        return gated_hypotheses, gated_weights, gated_inds

    def associate_and_update(self, gated_hypotheses, gated_weights, gated_inds,
                             z, pos_wc, Rot_wc):
        # Tested, works
        """
        Inputs: 
            gated_hypotheses: a list of MultiVarGauss of the active hypotheses which have been gated.
            gated_weights: a list of weights associated to the active hypotheses gated
            gated_inds: a list of indices in the self.active_hypotheses variable which have been gated.
                        Notice this is NOT indices in the gmf_weights (that is, nothing here has anythin
                        to do with the null hypothesis)

        Performs logic to associate and update the active hypotheses and their weights.

        3 valid cases for the logic here:
            a) 1 gated hypothesis: associate to that hypothesis and perform a KF update with the measurement
            b) more than 1 gated hypotheses: perform a mixture reduction on gated hypotheses associate and 
            update with reduced mixture.
            c) 0 associated hypotheses: initiate a hypothesis with the mean of the measurement and P_hat0 cov
            matrix
        """
        #search_measurement_model = LTV_search_measurement_model(self.sigma_z[:3], pos_wc, Rot_wc)
        #search_ekf = EKF(self.search_dynamic_model, search_measurement_model)

        if len(gated_hypotheses) == 1:
            ass_ind = gated_inds[0]
            _, _, upd_hypothesis = self.kf_function(
                z, self.active_hypotheses[ass_ind], search_ekf)
            self.active_hypotheses[ass_ind] = upd_hypothesis

            self.gmf_weights[ass_ind +
                             1] = self.gmf_weights[ass_ind +
                                                   1] + self.boost_prob

        elif len(gated_hypotheses) > 1:

            # Perform mixture reduction on gated hypotheses to produce associated hypothesis. Sum weights, boost
            ass_weight = np.sum(gated_weights)
            ass_weight += self.boost_prob
            ass_hypothesis = self.reduce_mixture(gated_hypotheses,
                                                 gated_weights)
            _, _, upd_hypothesis = self.kf_function(z, ass_hypothesis,
                                                    search_ekf)

            # Remove the gated hypotheses from the list, replace with reduced associated hypothesis. normalize
            for ind in sorted(gated_inds, reverse=True):
                self.gmf_weights = np.delete(self.gmf_weights, ind + 1)
                del self.active_hypotheses[ind]

            self.active_hypotheses.append(upd_hypothesis)
            self.gmf_weights = np.append(self.gmf_weights, ass_weight)
