
from os import remove
from gaussparams_py2 import MultiVarGaussian
import numpy as np
from scipy.stats import chi2


def test_function_template(args):

    #### GIVE SOME INPUTS ####
    test_inputs = 1

    #### PREDICT SOME OUTPUTS ####
    ref_outputs = 2

    #### CODE TO BE TESTED ####


    #### RESULTS ####
    print(f"The inputs are: {test_inputs}")
    print(f"The oututs are: {ref_outputs}")


def test_gate_hypotheses(active_hypotheses, active_hypotheses_count, gmf_weights):
    
    #### Parameters ####

    ndim = 6
    gate_percentile = 0.5

    gate_size_sq = chi2.ppf(gate_percentile, ndim)

    #### GIVE SOME INPUTS ####

    ref_weights = gmf_weights[2] # Hypothesis 2 is the only gated one, giving the third index in weights
    ref_hyps = active_hypotheses[1]

    #### CODE TO BE TESTED ####
    gated_inds = []
        

    for i in range(active_hypotheses_count):
        mahalanobis_distance = z.mahalanobis_distance_sq(predicted_zs[i])
        if mahalanobis_distance <= gate_size_sq:
            gated_inds.append(i)
            #m_distances.append(mahalanobis_distance)
    g = len(gated_inds)
    gated_hypotheses = [active_hypotheses[gated_inds[j]] for j in range(g)]
    gated_weights = np.array([gmf_weights[gated_inds[k] + 1] for k in range(g)])

    #### RESULTS ####
    return gated_hypotheses, gated_weights, gated_inds


def test_associate_and_update(gated_hypotheses, gated_weights, gated_inds, gmf_weights, active_hypotheses):
    #### PARAMETERS ####
    boost_prob = 0.1
    init_weight = 0.2
    #### GIVE SOME INPUTS ####

    #### PREDICT SOME OUTPUTS ####
    ref_outputs = 2

    #### CODE TO BE TESTED ####

    if len(gated_hypotheses) == 1:
        ass_ind = gated_inds[0]
        #active_hypotheses[ass_ind] = kf_function(z, active_hypotheses[ass_ind])
        gmf_weights[ass_ind + 1] = gmf_weights[ass_ind + 1] + boost_prob
        bruh = 1
        
    elif len(gated_hypotheses) > 1:
        
        # Perform mixture reduction on gated hypotheses to produce associated hypothesis. Sum weights, boost
        ass_weight = np.sum(gated_weights)
        ass_weight += boost_prob
        #ass_hypothesis = reduce_mixture(gated_hypotheses, gated_weights)
        #_, _, upd_hypothesis = kf_function(z, ass_hypothesis)
        # Remove the gated hypotheses from the list, replace with reduced associated hypothesis. normalize
        upd_hypothesis = MultiVarGaussian(0, 5)
        for ind in sorted(gated_inds, reverse = True): 
            gmf_weights = np.delete(gmf_weights, ind + 1)
            del active_hypotheses[ind]
        
        active_hypotheses.append(upd_hypothesis)
        gmf_weights = np.append(gmf_weights, ass_weight)
        bruh = 1
        
    else:
        # In case of no gated hypotheses initiate a new KF with mean of measurement
        ass_hypothesis = MultiVarGaussian(0, 5)
        ass_weight = init_weight
        active_hypotheses.append(ass_hypothesis)
        gmf_weights = np.append(gmf_weights, ass_weight)
        bruh = 1
    
    gmf_weights = gmf_weights / sum(gmf_weights)
    active_hypotheses_count = len(active_hypotheses)


def test_gmf_eval():
    ### PARAMETERS ###
    termination_criterion = 0.95
    max_nr_hypotheses = 25
    survival_threshold = 0.5
    termination_bool = False
    k = 15
    ### Tests ###
    # Test 1: termination of search
    active_hypotheses_count = 3
    gmf_weights = [0.001, 0.002, 0.97, 0.01]
    active_hypotheses = [MultiVarGaussian(0, 5) for i in range(active_hypotheses_count)]

    # Test 2: termination of many hypotheses
    active_hypotheses_count = 35
    gmf_weights = abs(np.random.normal(0,2, active_hypotheses_count))
    gmf_weights = gmf_weights / sum(gmf_weights)
    active_hypotheses = [MultiVarGaussian(0, 5) for i in range(active_hypotheses_count)]    




    #gmf_weights = gmf_weights / sum(gmf_weights)
    ### CODE ###
    #remove_inds = []
#
    #if active_hypotheses_count == 0:
    #        gmf_weights = gmf_weights / sum(gmf_weights)
    #        return None
#
    #for i in range(len(gmf_weights)):
    #    if gmf_weights[i] >= termination_criterion and i != 0:
    #        termination_bool = True
    #        bruh = 1
    #    if gmf_weights[i] <= survival_threshold and i != 0:
    #        remove_inds.append(i)
    #        #del active_hypotheses[i-1]

    #for remove_ind in sorted(remove_inds, reverse=True):
    #    gmf_weights = np.delete(gmf_weights, remove_ind)
    #    del active_hypotheses[remove_ind - 1]

    if len(active_hypotheses) >= max_nr_hypotheses:
        # Find the smallest 15 values, sum them up and add them to the null hypothesis
        hypotheses_weights = gmf_weights.copy()
        hypotheses_weights = np.delete(hypotheses_weights, 0)

        k_smallest_inds = np.argpartition(hypotheses_weights, k)
        termination_weights = hypotheses_weights[k_smallest_inds[:k]]
        new_null_weight = gmf_weights[0] + sum(termination_weights)
        
        # TODO: fix this np thing when I get to this function
        termination_weights = np.delete(hypotheses_weights, k_smallest_inds[:k])
        termination_weights = np.concatenate(([new_null_weight], termination_weights))
        gmf_weights = termination_weights

    gmf_weights = gmf_weights / sum(gmf_weights)
    active_hypotheses_count = len(active_hypotheses)
    best_ind = np.argmax(gmf_weights)

def test_reduce_mixture(gated_hypotheses, gated_weights):

    M = len(gated_weights)

    reduced_mean = 0
    cov_internal = np.zeros(np.shape(gated_hypotheses[0].cov))
    cov_external = np.zeros(np.shape(gated_hypotheses[0].cov))

    for i in range(M):
        reduced_mean += gated_hypotheses[i].mean * gated_weights[i]

    for i in range(M):
        weight_n = gated_weights[i]
        diff = np.array(gated_hypotheses[i].mean - reduced_mean)

        cov_internal += weight_n*gated_hypotheses[i].cov
        cov_external = weight_n * np.matmul(diff, diff.T)

    reduced_cov = cov_internal + cov_external

    return MultiVarGaussian(reduced_mean, reduced_cov)

def test_callback_logic():

    mission_topic = "pole_search"
    mission_topic_old = "gate_search"

    current_action = mission_topic.split("_")[1]
    old_action = mission_topic_old.split("_")[1]
    
    bruh = 1
    # Reset GMF on going from converge back to search
    if old_action == "converge" and current_action == "search":
        bruh = 1
        return None
    # Reset GMF on going from either execute or converge to search
    elif old_action == "execute" and current_action == "search":
        bruh = 1
        return None
    else:
        pass
    


z_mean = np.zeros((6,))
sigma_z = 2*np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
z = MultiVarGaussian(z_mean, np.diag(sigma_z))
hyp1_mean = 0.1 * np.ones((6,))
hyp2_mean = 0.2 * np.ones((6,))
hyp3_mean = 0.22 * np.ones((6,))
hyp_cov = np.diag(np.ones((6,)))
means = [hyp1_mean, hyp2_mean, hyp3_mean]

active_hypotheses = [MultiVarGaussian(means[i], hyp_cov) for i in range(len(means))]
active_hypotheses_count = len(active_hypotheses)
predicted_zs = means
gmf_weights = np.array([0.5, 0.2, 0.1, 0.2])


# TESTED, WORKS
#gated_hypotheses, gated_weights, gated_inds = test_gate_hypotheses(active_hypotheses, active_hypotheses_count, gmf_weights)
#
#test_associate_and_update(gated_hypotheses=gated_hypotheses, gated_inds=gated_inds, gated_weights=gated_weights, gmf_weights=gmf_weights, 
#                        active_hypotheses=active_hypotheses)


#test_gmf_eval()

#reduced_mixture = test_reduce_mixture(active_hypotheses, gmf_weights[1:])

test_callback_logic()