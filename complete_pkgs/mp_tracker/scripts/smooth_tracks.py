import numpy as np
import yaml
from infpy.gp import SquaredExponentialKernel, noise_kernel, gp_learn_hyperparameters
from infpy.gp import GaussianProcess


paths = yaml.load(open('data/depth_paths.yaml'))
new_paths = []
for k, path in enumerate(paths):
    print "Path ", k
    data = np.array(path)
    t = [[p[0]] for p in path]
    x = data[:,1]
    y = data[:,2]
    gp_x = GaussianProcess(t, x, SquaredExponentialKernel([12.0]) + noise_kernel(0.1))
    gp_y = GaussianProcess(t, y, SquaredExponentialKernel([12.0]) + noise_kernel(0.1))
#    gp_learn_hyperparameters(gp_x)
#    gp_learn_hyperparameters(gp_y)
    xs,_,_ =  gp_x.predict(t)
    ys,_,_ =  gp_y.predict(t)
    new_path = [[t[i][0], float(xs[i][0]), float(ys[i][0])] for i in range(len(t))]
    new_paths.append(new_path)
yaml.dump(new_paths, open('smooth_paths.yaml', 'w'))
