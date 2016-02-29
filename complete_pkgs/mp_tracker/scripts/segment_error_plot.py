import csv
import numpy as np
from matplotlib.pyplot import figure, subplot, title, bar, xlim, xticks, legend, xlabel, ylabel, savefig, plot, grid
import yaml
from matplotlib import rc
rc('font',family='serif', size=10)
rc('font',family='serif')
rc('text', usetex=True)

times_segments = dict(moving=[[], [], []], notmoving=[[], [], []])

def segment_ospa_error(segment_data, time, data, estimation=False):
    group_error = dict(moving=[[], [], []], notmoving=[[], [], []])
    segment_num = 0
    t0 = -1
    for i,t in enumerate(time):
        if t0 == -1:
            t0 = t
        if segment_num < len(segment_data) and t > segment_data[segment_num]['end']:
            if estimation:
                state = 'moving' if segment_data[segment_num]['moving'] else 'notmoving'
                complexity = segment_data[segment_num]['complexity']-1
                times_segments[state][complexity].append(t - t0)
                t0 = t
            segment_num += 1
        if segment_num < len(segment_data):
            state = 'moving' if segment_data[segment_num]['moving'] else 'notmoving'
            complexity = segment_data[segment_num]['complexity']-1
            if complexity < 0: continue
            group_error[state][complexity].append(data[i])
        if estimation and i == len(time) - 1 and segment_num < len(segment_data):
            print i, t
            state_ = 'moving' if segment_data[segment_num]['moving'] else 'notmoving'
            complexity_ = segment_data[segment_num]['complexity']-1
            times_segments[state_][complexity_].append(t - t0)
    return group_error

def segment_ospa_error_by_number(times, count, time, data, group_error):
    segment_num = 0
    for i,t in enumerate(time):
        if segment_num < len(times) and t > times[segment_num]:
            segment_num += 1
        complexity = count[segment_num]
        tmp = group_error.get(complexity, [])
        tmp.append(data[i])
        group_error[complexity] = tmp

def draw_bars(x, labels, data_base, data_comp):
    width = 0.35
    bar(x, data_base, width, color='r', hatch='.', label=label_base)
    bar(x+width, data_comp, width, color='b', hatch='x', label=label_comp)
    xticks(x+width, labels)
    xlim((np.min(x) - width, np.max(x) + 3*width))

def complete_grouped(x, grouped):
    for k in x:
        if len(grouped.get(k, [0.0])) > 4:
            grouped[k] = np.mean(grouped.get(k, [0.0]))
        else:
            grouped[k] = 0.0

scenarios_ospa_base = []
scenarios_ospa_comp = []
scenarios_ospa_base_card = []
scenarios_ospa_comp_card = []
scenarios_ospa_base_loc = []
scenarios_ospa_comp_loc = []

scenarios_ospa_base_grouped = {}
scenarios_ospa_comp_grouped = {}
scenarios_ospa_base_card_grouped = {}
scenarios_ospa_comp_card_grouped = {}
scenarios_ospa_base_loc_grouped = {}
scenarios_ospa_comp_loc_grouped = {}

#label_base = 'JPDA-based tracker'
#base_files = ('base_1_laser.csv', 'base_2_laser.csv', 'base_3_laser.csv', 'base_crowded_laser.csv')
#extra_filename = ''

#label_base = 'KF with NN association tracker'
#base_files = ('base_nn_1_laser.csv', 'base_nn_2_laser.csv', 'base_nn_3_laser.csv', 'base_nn_crowded_laser.csv')
#extra_filename = '_nn'

label_base = 'Standard PHD tracker'
base_files = ('error_1_leg_laser.csv', 'error_2_leg_laser.csv', 'error_3_leg_laser.csv', 'error_crowded_leg_laser.csv')
extra_filename = '_plainphd'

label_comp = 'PHD-based tracker'
titles = ('Long corridor scenario', 'Large hall scenario', 'Entrance scenario', 'Very crowded scenario')
extras_pdf = ('_1_laser', '_2_laser', '_3_laser', '_crowded_laser')
comp_files = ('error_1_laser.csv', 'error_2_laser.csv', 'error_3_laser.csv', 'error_crowded_laser.csv')
segmentation_files = ('segmentation_1.yaml', 'segmentation_3.yaml', 'segmentation_4.yaml', 'segmentation_2.yaml')
groud_truth_files = ('laser_paths_1.yaml', 'laser_paths_2.yaml', 'laser_paths_3.yaml', 'laser_paths_crowded.yaml')
use_ospa1 = True
if use_ospa1:
    oi = 5
    ci = 6
    li = 7
else:
    oi = 2
    ci = 3
    li = 4
ospa_label = 'OSPA$(1)$' if use_ospa1 else 'OSPA$(2)$'

max_number_people = -1
for i in range(4):
    # Open the files
    data_base = np.array([[float(x) for x in row] for row in csv.reader(open('data/' + base_files[i], 'r'))])
    data_comp = np.array([[float(x) for x in row] for row in csv.reader(open('data/' + comp_files[i], 'r'))])
    # Fix incorrect calues (nan)
    bad_points_base = np.nonzero(np.isnan(data_base[:,1]))[0]
    bad_points_comp = np.nonzero(np.isnan(data_comp[:,1]))[0]
    data_base[bad_points_base, 2] = 1.0
    data_base[bad_points_base, 3] = 1.0
    data_base[bad_points_base, 5] = 1.0
    data_base[bad_points_base, 6] = 1.0
    data_comp[bad_points_comp, 2] = 1.0
    data_comp[bad_points_comp, 3] = 1.0
    data_comp[bad_points_comp, 5] = 1.0
    data_comp[bad_points_comp, 6] = 1.0
    segment_data = yaml.load(open('data/' + segmentation_files[i], 'r'))
    raw_gt_data = yaml.load(open('data/' + groud_truth_files[i], 'r'))

    # Segmentation is especified in real world time, thus the start time should not be removed
    time_base = data_base[:,0]
    time_comp = data_comp[:,0]
    # The OSPA errors
    scenarios_ospa_base.append(segment_ospa_error(segment_data, time_base, data_base[:,oi]))
    scenarios_ospa_comp.append(segment_ospa_error(segment_data, time_comp, data_comp[:,oi], True))
    scenarios_ospa_base_card.append(segment_ospa_error(segment_data, time_base, data_base[:,ci]))
    scenarios_ospa_comp_card.append(segment_ospa_error(segment_data, time_comp, data_comp[:,ci]))
    scenarios_ospa_base_loc.append(segment_ospa_error(segment_data, time_base, data_base[:,li]))
    scenarios_ospa_comp_loc.append(segment_ospa_error(segment_data, time_comp, data_comp[:,li]))

    # The number of people in time
    start_time = min(np.min(time_base), np.min(time_comp))
    time_base -= start_time
    time_comp -= start_time
    people_count_gt = np.array(sum([[float(x[0]/1000000000) + float(x[0]%1000000000)/1000000000 for x in path] for path in raw_gt_data['paths']], [])) - start_time
    min_gt_time = np.min(people_count_gt)
    max_gt_time = np.max(people_count_gt)
    times_gt = np.arange(min_gt_time, max_gt_time, 0.05)
    count_gt = np.zeros(times_gt.shape)
    for j,t in enumerate(times_gt):
        for p in raw_gt_data['paths']:
            t_start = float(p[0][0]/1000000000) + float(p[0][0]%1000000000)/1000000000 - start_time
            t_end = float(p[-1][0]/1000000000) + float(p[-1][0]%1000000000)/1000000000 - start_time
            if t >= t_start and t <= t_end:
                count_gt[j] += 1
    changes = times_gt[np.nonzero(np.diff(count_gt))[0]]
    number = count_gt[np.nonzero(np.diff(count_gt))[0]-1]
    number = np.hstack((number, count_gt[-1]))
    max_number_people = max(max_number_people, np.max(number))

    # OSPA error group by number of people in the environment
    segment_ospa_error_by_number(changes, number, time_base, data_base[:,oi], scenarios_ospa_base_grouped)
    segment_ospa_error_by_number(changes, number, time_comp, data_comp[:,oi], scenarios_ospa_comp_grouped)
    segment_ospa_error_by_number(changes, number, time_base, data_base[:,ci], scenarios_ospa_base_card_grouped)
    segment_ospa_error_by_number(changes, number, time_comp, data_comp[:,ci], scenarios_ospa_comp_card_grouped)
    segment_ospa_error_by_number(changes, number, time_base, data_base[:,li], scenarios_ospa_base_loc_grouped)
    segment_ospa_error_by_number(changes, number, time_comp, data_comp[:,li], scenarios_ospa_comp_loc_grouped)

    # OSPA error group by number of people in the environment for each scenario
    tmp_base_grouped = {}
    tmp_comp_grouped = {}
    tmp_base_card_grouped = {}
    tmp_comp_card_grouped = {}
    tmp_base_loc_grouped = {}
    tmp_comp_loc_grouped = {}
    segment_ospa_error_by_number(changes, number, time_base, data_base[:,oi], tmp_base_grouped)
    segment_ospa_error_by_number(changes, number, time_comp, data_comp[:,oi], tmp_comp_grouped)
    segment_ospa_error_by_number(changes, number, time_base, data_base[:,ci], tmp_base_card_grouped)
    segment_ospa_error_by_number(changes, number, time_comp, data_comp[:,ci], tmp_comp_card_grouped)
    segment_ospa_error_by_number(changes, number, time_base, data_base[:,li], tmp_base_loc_grouped)
    segment_ospa_error_by_number(changes, number, time_comp, data_comp[:,li], tmp_comp_loc_grouped)
    x = np.arange(0, np.max(number) + 1, 1.0)
    complete_grouped(x, tmp_base_grouped)
    complete_grouped(x, tmp_comp_grouped)
    complete_grouped(x, tmp_base_card_grouped)
    complete_grouped(x, tmp_comp_card_grouped)
    complete_grouped(x, tmp_base_loc_grouped)
    complete_grouped(x, tmp_comp_loc_grouped)

    figure()
    title('Grouped by number of people in ' + titles[i])
    labels = [str(int(k)) for k in tmp_base_grouped.keys()]
    draw_bars(x, labels , [v for (k,v) in tmp_base_grouped.iteritems()], [v for (k,v) in tmp_comp_grouped.iteritems()])
    legend(loc='lower right')
    xlabel('Scenarios')
    ylabel(ospa_label + ' metric')
    savefig('group_by_number' + extras_pdf[i] + extra_filename + '.pdf', bbox_inches='tight')

    figure()
    labels = [str(int(k)) for k in tmp_base_grouped.keys()]
    subplot(211)
    draw_bars(x, labels , [v for (k,v) in tmp_base_card_grouped.iteritems()], [v for (k,v) in tmp_comp_card_grouped.iteritems()])
    ylabel(ospa_label + ' cardinality error')
    subplot(212)
    draw_bars(x, labels , [v for (k,v) in tmp_base_loc_grouped.iteritems()], [v for (k,v) in tmp_comp_loc_grouped.iteritems()])
    ylabel(ospa_label + ' location error')
    legend(loc='upper left')
    xlabel('Scenarios')
    savefig('details_group_by_number' + extras_pdf[i] + extra_filename +  '.pdf', bbox_inches='tight')

times_segments = dict(moving=[\
                        np.sum(times_segments['moving'][0]),\
                        np.sum(times_segments['moving'][1]),\
                        np.sum(times_segments['moving'][2])],\
                      notmoving=[\
                        np.sum(times_segments['notmoving'][0]),\
                        np.sum(times_segments['notmoving'][1]),\
                        np.sum(times_segments['notmoving'][2])])

def sumarise(errors):
    new_errors = dict(moving=[\
                          np.mean(sum([sc['moving'][0] for sc in errors], [])),\
                          np.mean(sum([sc['moving'][1] for sc in errors], [])),\
                          np.mean(sum([sc['moving'][2] for sc in errors], []))],\
                      notmoving=[\
                          np.mean(sum([sc['notmoving'][0] for sc in errors], [])),\
                          np.mean(sum([sc['notmoving'][1] for sc in errors], [])),\
                          np.mean(sum([sc['notmoving'][2] for sc in errors], []))])
    moving_err = np.array(new_errors['moving']) * np.array(times_segments['moving'])
    static_err = np.array(new_errors['notmoving']) * np.array(times_segments['notmoving'])
    averag_err = (moving_err + static_err) / (np.array(times_segments['moving'])+ np.array(times_segments['notmoving']))
    return new_errors, averag_err

scenarios_ospa_base, average_ospa_base = sumarise(scenarios_ospa_base)
scenarios_ospa_comp, average_ospa_comp = sumarise(scenarios_ospa_comp)
scenarios_ospa_base_card, average_ospa_base_card = sumarise(scenarios_ospa_base_card)
scenarios_ospa_comp_card, average_ospa_comp_card = sumarise(scenarios_ospa_comp_card)
scenarios_ospa_base_loc, average_ospa_base_loc = sumarise(scenarios_ospa_base_loc)
scenarios_ospa_comp_loc, average_ospa_comp_loc = sumarise(scenarios_ospa_comp_loc)

print "OSPA"
print label_base, scenarios_ospa_base
print label_comp, scenarios_ospa_comp
print
print label_base, average_ospa_base
print label_comp, average_ospa_comp
print
print "OSPA cardinality"
print label_base, scenarios_ospa_base_card
print label_comp, scenarios_ospa_comp_card
print
print label_base, average_ospa_base_card
print label_comp, average_ospa_comp_card
print
print "OSPA location"
print label_base, scenarios_ospa_base_loc
print label_comp, scenarios_ospa_comp_loc
print
print label_base, average_ospa_base_loc
print label_comp, average_ospa_comp_loc
print
print times_segments

figure()
x = np.array([0, 1, 2])
labels = ('Simple', 'Medium', 'Complex')
plot(scenarios_ospa_base['notmoving'], 'b->', label=label_base + ' and static robot')
plot(scenarios_ospa_base['moving'], 'r-<', label=label_base + ' and moving robot')
plot(scenarios_ospa_comp['notmoving'], 'b*--', label=label_comp + ' and static robot')
plot(scenarios_ospa_comp['moving'], 'rd--', label=label_comp + ' and moving robot')
xticks(x, labels)
xlim((np.min(x) - 0.5, np.max(x) + 0.5))
legend(loc='lower right')
xlabel('Scenarios')
ylabel(ospa_label + ' metric')
grid()
savefig('group_together' + extra_filename + '.pdf', bbox_inches='tight')

x = np.array([0, 1, 2])
figure()
title('Scenarios with static robot')
draw_bars(x, ('Simple', 'Medium', 'Complex'), scenarios_ospa_base['notmoving'], scenarios_ospa_comp['notmoving'])
legend(loc='lower right')
xlabel('Scenarios')
ylabel(ospa_label + ' metric')
savefig('static_robot' + extra_filename + '.pdf', bbox_inches='tight')

x = np.array([0, 1, 2])
figure()
title('Scenarios with moving robot')
draw_bars(x, ('Simple', 'Medium', 'Complex'), scenarios_ospa_base['moving'], scenarios_ospa_comp['moving'])
legend(loc='lower right')
xlabel('Scenarios')
ylabel(ospa_label + ' metric')
savefig('moving_robot' + extra_filename + '.pdf', bbox_inches='tight')

x = np.array([0, 1])
figure()
title('Moving vs static')
draw_bars(x, ('Moving', 'Not moving'), [np.mean(scenarios_ospa_base['moving']), np.mean(scenarios_ospa_base['notmoving'][0::2])], \
                                       [np.mean(scenarios_ospa_comp['moving']), np.mean(scenarios_ospa_comp['notmoving'][0::2])])
legend(loc='lower right')
xlabel('Scenarios')
ylabel(ospa_label + ' metric')
savefig('moving_vs_static' + extra_filename + '.pdf', bbox_inches='tight')

figure()
subplot(211)
title('Details errors for scenarios with moving robot')
x = np.array([0, 1, 2])
draw_bars(x, ('Simple', 'Medium', 'Complex'), scenarios_ospa_base_card['moving'], scenarios_ospa_comp_card['moving'])
ylabel(ospa_label + ' cardinality error')
legend(loc='lower right')
subplot(212)
draw_bars(x, ('Simple', 'Medium', 'Complex'), scenarios_ospa_base_loc['moving'], scenarios_ospa_comp_loc['moving'])
ylabel(ospa_label + ' location error')
xlabel('Scenarios')
savefig('details_moving_robot' + extra_filename + '.pdf', bbox_inches='tight')

figure()
subplot(211)
title('Details errors for scenarios with static robot')
x = np.array([0, 1, 2])
draw_bars(x, ('Simple', 'Medium', 'Complex'), scenarios_ospa_base_card['notmoving'], scenarios_ospa_comp_card['notmoving'])
ylabel(ospa_label + ' cardinality error')
legend(loc='lower right')
subplot(212)
draw_bars(x, ('Simple', 'Complex'), scenarios_ospa_base_loc['notmoving'], scenarios_ospa_comp_loc['notmoving'])
ylabel(ospa_label + ' location error')
xlabel('Scenarios')
savefig('details_static_robot' + extra_filename + '.pdf', bbox_inches='tight')

x = np.arange(0, 10, 1.0)
complete_grouped(x, scenarios_ospa_base_grouped)
complete_grouped(x, scenarios_ospa_comp_grouped)
complete_grouped(x, scenarios_ospa_base_card_grouped)
complete_grouped(x, scenarios_ospa_comp_card_grouped)
complete_grouped(x, scenarios_ospa_base_loc_grouped)
complete_grouped(x, scenarios_ospa_comp_loc_grouped)

print "OSPA grouped"
print label_base, [scenarios_ospa_base_grouped[k] for k in x]
print label_comp, [scenarios_ospa_comp_grouped[k] for k in x]
print
print "OSPA cardinality"
print label_base, [scenarios_ospa_base_card_grouped[k] for k in x]
print label_comp, [scenarios_ospa_comp_card_grouped[k] for k in x]
print
print "OSPA location"
print label_base, [scenarios_ospa_base_loc_grouped[k] for k in x]
print label_comp, [scenarios_ospa_comp_loc_grouped[k] for k in x]
print

figure()
title('Grouped by number of people')
labels = [str(int(k)) for k in x]
draw_bars(x, labels , [scenarios_ospa_base_grouped[v] for v in x], [scenarios_ospa_comp_grouped[v] for v in x])
legend(loc='lower right')
xlabel('Number of people')
ylabel(ospa_label + ' metric')
savefig('group_by_number' + extra_filename + '.pdf', bbox_inches='tight')

figure()
labels = [str(int(k)) for k in scenarios_ospa_base_grouped.keys()]
subplot(211)
draw_bars(x, labels , [scenarios_ospa_base_card_grouped[v] for v in x], [scenarios_ospa_comp_card_grouped[v] for v in x])
ylabel(ospa_label + ' cardinality error')
legend(loc='lower right')
subplot(212)
draw_bars(x, labels , [scenarios_ospa_base_loc_grouped[v] for v in x], [scenarios_ospa_comp_loc_grouped[v] for v in x])
ylabel(ospa_label + ' location error')
xlabel('Number of people')
savefig('details_group_by_number' + extra_filename + '.pdf', bbox_inches='tight')
