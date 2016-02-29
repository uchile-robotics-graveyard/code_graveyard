import roslib
roslib.load_manifest('mp_tracker')
import tf
import rospy
from pc_utils import read_pc2
from sensor_msgs.msg import PointCloud2
from munkres import Munkres
from math import sqrt, fabs, cos, radians
import csv
import threading
import Queue as queue
import pickle
import yaml
import numpy as np

c = 1.0
class EvaluatorNode:
    def __init__(self):
        rospy.init_node('evaluator', anonymous=True)
        rospy.Subscriber('/tracked', PointCloud2, self.estimation)
        self.output_file = rospy.get_param('~output', 'error')
        self.ground_truth_file =  rospy.get_param('~ground_truth', 'file.yaml')
        self.camera_fov =  rospy.get_param('~camera_fov', False)
        self.cosfov = cos(radians(57.0 / 2.0))
        gt_data = yaml.load(open(self.ground_truth_file, 'r'))
        self.paths = gt_data['paths']
        # Variables used
        self.estimated_points = []
        self.num_estimated = []
        self.times = []
        self.distances = []
        self.complete = []
        self.queue = queue.Queue()
        self.processed = 0
        self.created = 0
        self.work_thread = threading.Thread(target=self.calculate)
        self.work_thread.daemon = True
        self.work_thread.start()
        self.transformer = tf.TransformListener()
        # For the MOTP and MOTD metric
        self.sum_dist_t = 0
        self.sum_matches_t = 0
        self.sum_fp_t = 0
        self.sum_miss_t = 0
        self.sum_miss_match_t = 0
        self.sum_objects_t = 0

        self.gt_mapping = dict()

#        rospy.Timer(rospy.Duration(0.1), self.calc_error, False)
        rospy.on_shutdown(self.flush)
        rospy.spin()

    def flush(self):
        print "A total of ", self.created, " messages were created and ", self.processed, " were processed"
        f = open(self.output_file + '.csv', 'w')
        outfile = csv.writer(f)
        for d in self.distances:
            outfile.writerow(d)
        f.close()
        f = open(self.output_file + '.pickle', 'w')
        pickle.dump(self.complete, f)
        f.close()
        self.running = False
        
    def calc_error(self, event):
        a = list(self.estimated_points)
        b = list(self.times)
        c = list(self.num_estimated)
        self.queue.put((rospy.Time.now().to_sec(), a, b, c))
        self.estimated_points = []
        self.ground_truth_points = []
        self.num_estimated = []
        self.times = []
        self.created += 1

    def calculate(self):
        while True:
            time, estimated_points, times, num_estimated = self.queue.get()
            # Find the points to compare against
            ground_truth_points = []
            for i,path in enumerate(self.paths):
                ti = float(path[0][0]) / 1000000000.0
                tf = float(path[-1][0]) / 1000000000.0
                if time >= ti and time <= tf:
                    dt = [fabs(time - p[0] / 1000000000.0) for p in path]
                    min_pt = path[dt.index(min(dt))]
                    if self.camera_fov:
                        if min_pt[1] / sqrt(min_pt[1]**2 + min_pt[2]**2) < self.cosfov \
                           or sqrt(min_pt[1]**2 + min_pt[2]**2) > 10.0 \
                           or sqrt(min_pt[1]**2 + min_pt[2]**2) < 1.0:
                            continue
                    ground_truth_points.append([min_pt[1], min_pt[2], i])
            # Calculate the OSPA1/2 distance between the estimation and the ground truth
            self.processed += 1
            mnk = Munkres()
            cost = []
            if len(estimated_points) > 0 and len(ground_truth_points) > 0:
                for e in estimated_points:
                    row = []
                    for g in ground_truth_points:
                        row.append(sqrt((e[0] - g[0]) ** 2 + (e[1] - g[1]) ** 2))
                    cost.append(row)
                indexes = mnk.compute(cost)
            else:
                indexes = []
            n = float(max(len(estimated_points), len(ground_truth_points)))
            m = float(min(len(estimated_points), len(ground_truth_points)))
            card_error_1 = c * (n - m)
            card_error_2 = (c ** 2) * (n - m)
            loc_error_1 = 0.0
            loc_error_2 = 0.0
            loc_errors = []
            matches = 0.0
            for (x,y) in indexes:
                d = min(cost[x][y], c)
                loc_error_1 +=  d
                loc_error_2 += d ** 2
                loc_errors.append(d)
                if d < c:
                    indx = ground_truth_points[y][2]
                    if indx in self.gt_mapping.keys():
                        label = self.gt_mapping[indx]
                        if label <> estimated_points[x][2]:
                            self.sum_miss_match_t += 1.0
                    self.gt_mapping[indx] = estimated_points[x][2]
                    self.sum_dist_t += d
                    matches += 1.0
            self.sum_objects_t += float(len(ground_truth_points))
            self.sum_miss_t += len(ground_truth_points) - matches
            self.sum_fp_t += len(estimated_points) - matches
            self.sum_matches_t += matches
            ospa_dist_1 = (card_error_1 + loc_error_1) / n if n > 0 else 0
            ospa_dist_2 = sqrt((card_error_2 + loc_error_2) / n) if n > 0 else 0
            dt = 0.0
            for (i,t) in enumerate(times):
                if i > 0:
                    dt += fabs(t - times[i-1])
            if len(times) > 0:
                dt /= len(times)
            else:
                dt = 'nan'
            if len(num_estimated) > 0:
                num_targets = float(sum(num_estimated)) / len(num_estimated)
            else:
                num_targets = 0.0
            self.complete.append((time, n, m, loc_errors, dt, num_targets))
            self.distances.append([time,
                                   dt,
                                   ospa_dist_2,
                                   sqrt(card_error_2 / n) if n > 0 else 0,
                                   sqrt(loc_error_2 / n) if n > 0 else 0,
                                   ospa_dist_1,
                                   card_error_1 / n if n > 0 else 0,
                                   loc_error_1 / n if n > 0 else 0,
                                   num_targets,
                                   # MOTP
                                   self.sum_dist_t / self.sum_matches_t if self.sum_matches_t > 0 else 1.0,
                                   # MOTA
                                   1.0 - (self.sum_miss_t + self.sum_miss_match_t + self.sum_fp_t) / self.sum_objects_t if self.sum_objects_t > 0 else 1.0,
                                   # Components of MOTA
                                   self.sum_miss_t / self.sum_objects_t if self.sum_objects_t > 0 else 1.0,
                                   self.sum_fp_t / self.sum_objects_t if self.sum_objects_t > 0 else 1.0,
                                   self.sum_miss_match_t / self.sum_objects_t if self.sum_objects_t > 0 else 1.0,
                                   # The RAW data for MOTA and MOTP
                                   self.sum_matches_t,
                                   self.sum_objects_t,
                                   self.sum_dist_t,
                                   self.sum_miss_t,
                                   self.sum_fp_t,
                                   self.sum_miss_match_t])

    def estimation(self, msg):
        fields, pc = read_pc2(msg)
        self.times.append(msg.header.stamp.to_sec())
        if pc is None:
            self.num_estimated.append(0)
            return
        self.num_estimated.append(pc.shape[0])
        msg.header.stamp = rospy.Time()
        try:
            mat44 = self.transformer.asMatrix('/laser', msg.header)
            for point in pc:
                xyz = tuple(np.dot(mat44, np.array([point[0], point[1], 0.0, 1.0])))[:3]
                # Dont forget about the id of the point
                self.estimated_points.append([xyz[0], xyz[1], point[6]])
            self.calc_error(None)
        except Exception as e:
            print "PROBLEM:", str(e)

if __name__ == "__main__":
    e = EvaluatorNode()
