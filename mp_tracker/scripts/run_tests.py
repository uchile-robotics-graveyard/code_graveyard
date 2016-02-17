#!/usr/bin/env python
import psutil
import time

devnull = open('/dev/null', 'w')
bagfiles = ('/data/bagfiles/annotated_01.bag', '/data/bagfiles/annotated_03.bag', '/data/bagfiles/annotated_04.bag', '/data/bagfiles/annotated_02.bag')
ground_truth = ('laser_paths_1.yaml', 'laser_paths_2.yaml', 'laser_paths_3.yaml', 'laser_paths_crowded.yaml')

launch_files = ('base_jpda.launch', 'base_nn.launch', 'mine_laser.launch', 'mine_leg_laser.launch', 'mine_vision.launch', 'mine_kinect.launch', 'mine_beta_visual.launch')
base_name = ('base_', 'base_nn_', 'error_', 'error_', 'error_', 'error_', 'error_')
base_end = ('_laser', '_laser', '_laser', '_leg_laser', '_vision', '_kinect', '_beta_vision')

for j,bag in enumerate(bagfiles):
    for i,l in enumerate(launch_files):
        if i <> 2:
            continue
        # Launch the base roslaunch
        kinect = psutil.Popen(['/opt/ros/electric/stacks/ros_comm/tools/roslaunch/bin/roslaunch', 'icl_robot', 'record_temp.launch'])
        time.sleep(10)

        # Launch the tracker
        tracker = psutil.Popen(['/opt/ros/electric/stacks/ros_comm/tools/roslaunch/bin/roslaunch', 'mp_tracker', l])

        # Run the evaluator
        name = str(j+1) if j < 3 else 'crowded'
        eval_params = ['_output:='+base_name[i] + name + base_end[i], '_ground_truth:=/data/ros/people_detect_and_tracking/mp_tracker/data/' + ground_truth[j]]
        if i >= 4:
            eval_params += ['_camera_fov:=true']
        evaluator = psutil.Popen(['/usr/bin/python', '/data/ros/people_detect_and_tracking/mp_tracker/scripts/evaluator.py'] + eval_params)

        # Wait for it to launch all nodes
        time.sleep(5)

        # Run ROSBAG
        bagprocess = psutil.Popen(['/opt/ros/electric/stacks/ros_comm/tools/rosbag/bin/rosbag', 'play', '--clock', bag])
        time.sleep(460)
        while bagprocess.is_running() and bagprocess.status <> psutil.STATUS_ZOMBIE:
            time.sleep(10)

        # Restart everything
        tracker.terminate()
        evaluator.terminate()
        kinect.terminate()
        while tracker.is_running() and tracker.status <> psutil.STATUS_ZOMBIE:
            print "Waiting for %s to finish (status %d)" % (str(tracker), tracker.status)
            time.sleep(5)
        while evaluator.is_running() and evaluator.status <> psutil.STATUS_ZOMBIE:
            print "Waiting for %s to finish (status %d)" % (str(evaluator), evaluator.status)
            time.sleep(5)
        while kinect.is_running() and kinect.status <> psutil.STATUS_ZOMBIE:
            print "Waiting for %s to finish (status %d)" % (str(kinect), kinect.status)
            time.sleep(5)
