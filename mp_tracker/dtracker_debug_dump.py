import yaml

outfile = yaml(open('outfile.yaml'))
def debug_plot(num, time, T, weights, means, covs, laser=None, measurements=None, as_=None, bs_=None):
    if num % 5:
        return
    global laser_points, measurement_points, ground_truth_points, arrows

    title("Time %f" % (time-start_time))
    # Convert to numpy
    weights = np.array(weights)
    means = np.array(means)
    covs = np.array(covs)

    T = np.array(T)

    dt = np.array([abs(t0 - time) for t0 in points[:,0]])
    match = np.nonzero(dt < 0.1)[0]
    if len(match) > 0:
        _points = np.transpose(np.dot(T, np.transpose(points[match,1:])))
        if ground_truth_points is None:
            ground_truth_points, = plot(_points[:,0], _points[:,1], 'mo', markersize=15, alpha=0.5)
        else:
            ground_truth_points.set_data(_points[:,0], _points[:,1])
    elif ground_truth_points is not None:
        ground_truth_points.set_data(None, None)
    # Display the laser
    if laser:
        laser = np.array(laser)
        if laser_points is None:
            laser_points, = plot(laser[:,0], laser[:,1], 'r.', mec='r')
        else:
            laser_points.set_data(laser[:,0], laser[:,1])
            xlim(np.min(laser[:,0]), np.max(laser[:,0]))
            ylim(np.min(laser[:,1]), np.max(laser[:,1]))
    # Display the measurements
#    if measurements:
#        measurements = np.array(measurements)
#        if measurement_points is None:
#            measurement_points, = plot(measurements[:,0], measurements[:,1], 'bo', mec='b')
#        else:
#            measurement_points.set_data(measurements[:,0], measurements[:,1])

    # Display the gaussians
    for i,e in enumerate(ellipses):
        e.remove()
        del ellipses[i]
    real_targets = np.nonzero(weights > 0.5)[0]
    if len(real_targets) > 0 :
        if measurement_points is None:
            measurement_points, = plot(means[real_targets,0], means[real_targets,1], 'bo')
        else:
            measurement_points.set_data(means[real_targets,0], means[real_targets,1])
        if arrows is None:
            arrows = quiver(means[real_targets,0], means[real_targets,1], means[real_targets,2], means[real_targets,3], scale=10)
        else:
            arrows.set_offsets(means[real_targets,0:2])
            arrows.set_UVC(means[real_targets,2], means[real_targets,3])
    elif measurement_points is not None:
        measurement_points.set_data(None, None)
#    for i in real_targets:
#        try:
#            (w,v) = np.linalg.eig(covs[i,0:2,0:2])
#            angle = m.atan2(v[1,0], v[0,0])
#            colour = weights[i];
#            e = Ellipse(xy=means[i,0:2],
#                        width=3.0*m.sqrt(w[0]),
#                        height=3.0*m.sqrt(w[1]),
#                        angle=m.degrees(angle),
#                        alpha=0.5,
#                        fc=plt.cm.jet(colour))
#            ax.add_artist(e)
#            ellipses.append(e)
#            if as_ is not None and bs_ is not None and as_[i] > bs_[i]:
#                continue
#            if np.linalg.norm(means[i,2:4]) < 0.1:
#                continue
#            a = Arrow(x=means[i,0], y=means[i,1], dx=means[i,2], dy=means[i,3])
#            ax.add_artist(a)
#            arrows.append(a)
#        except Exception as e:
#            print "Something happend: " + str(e)
#    valid = np.array(weights) > 0.5
#    print "Valid estimations: ", np.sum(valid), " total weight:", np.sum(weights)
#    print np.sum(valid * (np.array(as_) < np.array(bs_))), " are dynamic and ", np.sum(valid * (np.array(as_) > np.array(bs_))), " are static"
    total =  np.sum(weights)
    draw()
    savefig('tracking_%05d.pdf' % (num/5), bbox_inches='tight')
    return total
