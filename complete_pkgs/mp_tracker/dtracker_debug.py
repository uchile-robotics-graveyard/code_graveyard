import matplotlib.pyplot as plt
from matplotlib.pyplot import figure, plot, subplot, savefig
import numpy as np
import math as m
from matplotlib.patches import Ellipse, Arrow

def debug_plot(num, weights, means, covs, laser=None, measurements=None):
    # Create the figure
    figure()
    ax = subplot(111, aspect='equal')
    # Convert to numpy
    weights = np.array(weights)
    means = np.array(means)
    covs = np.array(covs)

    # Display the laser
    if laser:
        laser = np.array(laser)
        laser_points, = plot(laser[:,0], laser[:,1], 'r.', mec='r')
    # Display the measurements
    if measurements:
        measurements = np.array(measurements)
        plot(measurements[:,0], measurements[:,1], 'bo', mec='b')

    # Display the gaussians
    for i in range(weights.shape[0]):
        (w,v) = np.linalg.eig(covs[i,:,:])
        angle = m.atan2(v[0,1], v[0,0])
        e = Ellipse(xy=means[i,0:2],
                    width=2.0*m.sqrt(w[0]),
                    height=2.0*m.sqrt(w[1]),
                    angle=m.degrees(angle),
                    alpha=0.5,
                    fc=plt.cm.jet(weights[i]))
        ax.add_artist(e)
        a = Arrow(x=means[i,0], y=means[i,1], dx=means[i,2], dy=means[i,3])
        ax.add_artist(a)
    total =  np.sum(weights)
    savefig("image_%05d.png" % num)
    return total
