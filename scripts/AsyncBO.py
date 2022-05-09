
from warnings import catch_warnings
from warnings import simplefilter
import numpy as np
import scipy.stats


np.random.seed(2135)

sensing_noise = 0.1
# sampling radius
sensing_radius = 1.0  # m
num_sample_eval = 50

# simulation parameters
target_area = [0, 2, 0, 2]


def random(size, bound):
    '''
    :param size: number of samples
    :param bound: [Xmin, Xmax, Ymin, Ymax]
    :return: uniform random samples within the bound
    '''
    x_rand = np.random.uniform(bound[0], bound[1], size=size)
    y_rand = np.random.uniform(bound[2], bound[3], size=size)
    X = np.asarray([(x, y) for x, y in zip(x_rand, y_rand)])
    return X

# surrogate or approximation for the measurement function
def surrogate(model, X):
    # catch any warning generated when making a prediction
    with catch_warnings():
        # ignore generated warnings
        simplefilter("ignore")
        return model.predict(X, return_std=True)

# probability of improvement acquisition function
def acquisition(X, Xsamples, model):
    # calculate the best surrogate score found so far
    yhat, _ = surrogate(model, X)
    best = max(yhat)
    # calculate mean and stdev via surrogate function
    mu, std = surrogate(model, Xsamples)
    # calculate the probability of improvement
    probs = scipy.stats.norm.cdf((mu - best) / (std+1E-9))
    return probs

def opt_acquisition(pos, X, model):

    #sampling bound
    bound = [pos[0]-sensing_radius, pos[0]+sensing_radius, pos[1]-sensing_radius, pos[1]+sensing_radius]
    # random search, generate random samples
    Xsamples = random(num_sample_eval, bound)
    # point in target area
    in_target = lambda x: target_area[0] < x[0] < target_area[1] and target_area[2] < x[1] < target_area[3]
    # prune samples outside of sampling sensing_radius
    Xsamples = np.asarray([x for x in Xsamples if np.linalg.norm(x-pos)<sensing_radius and in_target(x)])
    # calculate the acquisition function for each sample
    scores = acquisition(X, Xsamples, model)
    # locate the index of the largest scores
    ix = np.argmax(scores)
    return Xsamples[ix]