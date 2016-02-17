import roslib
roslib.load_manifest('mp_tracker')
from munkres import Munkres
from math import sqrt
import csv
import pickle
import yaml
import sys

c = 1.0
class EvaluatorNode:
    def __init__(self, infile, outfile):
        self.output_file = 'data/' + outfile
        ground_truth_file =  'data/depth_paths.yaml'
        data_file = infile+'.yaml'
        self.paths = yaml.load(open(ground_truth_file, 'r'))
        self.raw_data = yaml.load(open(data_file, 'r'))
        # Variables used
        self.estimated_points = []
        self.num_estimated = []
        self.times = []
        self.distances = []
        self.complete = []
        self.processed = 0
        self.created = 0
        # For the MOTP and MOTD metric
        self.sum_dist_t = 0
        self.sum_matches_t = 0
        self.sum_fp_t = 0
        self.sum_miss_t = 0
        self.sum_miss_match_t = 0
        self.sum_objects_t = 0

        self.gt_mapping = dict()

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
        
    def calculate(self):
        for time,data in enumerate(self.raw_data):
            if data is False:
                data = []
            # Find the points to compare against
            ground_truth_points = []
            for i,path in enumerate(self.paths):
                ti = float(path[0][0])
                tf = float(path[-1][0])
                if time >= ti and time <= tf:
                    dt = [abs(time - p[0]) for p in path]
                    if min(dt) == 0:
                        min_pt = path[dt.index(min(dt))]
                        ground_truth_points.append([i, min_pt[1], min_pt[2]])
            # Calculate the OSPA1/2 distance between the estimation and the ground truth
            self.processed += 1
            mnk = Munkres()
            cost = []
            if len(data) > 0 and len(ground_truth_points) > 0:
                for e in data:
                    row = []
                    for g in ground_truth_points:
                        row.append(sqrt((e[2] - g[2]) ** 2 + (e[1] - g[1]) ** 2))
                    cost.append(row)
                indexes = mnk.compute(cost)
            else:
                indexes = []
            n = float(max(len(data), len(ground_truth_points)))
            m = float(min(len(data), len(ground_truth_points)))
            card_error_1 = c * (n - m)
            card_error_2 = (c ** 2) * (n - m)
            loc_error_1 = 0.0
            loc_error_2 = 0.0
            loc_errors = []
            matches = 0
            for (x,y) in indexes:
                d = min(cost[x][y], c)
                loc_error_1 +=  d
                loc_error_2 += d ** 2
                loc_errors.append(d)
                if d < c:
                    # A match was made
                    self.sum_dist_t += d
                    matches += 1.0
                    # Check if there is a change of id
                    indx = ground_truth_points[y][0]
                    if indx in self.gt_mapping.keys():
                        label = self.gt_mapping[indx]
                        if label <> data[x][0]:
                            self.sum_miss_match_t += 1.0
                    self.gt_mapping[indx] = data[x][0]
            # The rest of the metric for CLEAR MOT
            self.sum_objects_t += float(len(ground_truth_points))
            self.sum_miss_t += len(ground_truth_points) - matches
            self.sum_fp_t += len(data) - matches
            self.sum_matches_t += matches
            ospa_dist_1 = (card_error_1 + loc_error_1) / n if n > 0 else 0
            ospa_dist_2 = sqrt((card_error_2 + loc_error_2) / n) if n > 0 else 0
            dt = 0.0
            num_targets = len(data)
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

if __name__ == "__main__":
    infile = 'data/error_depth_dataset'
    outfile = 'rgbd_people_dataset'
    if len(sys.argv) == 3:
        infile = sys.argv[1]
        outfile = sys.argv[2]
    e = EvaluatorNode(infile, outfile)
    e.calculate()
    e.flush()
