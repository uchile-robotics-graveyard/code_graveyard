import csv
import numpy as np
from matplotlib.pyplot import figure, grid, subplot, plot, legend, xlabel, ylabel, fill_between, savefig, xlim, bar, axvspan, twinx, gca
from matplotlib.ticker import FuncFormatter
import yaml
from matplotlib import rc
rc('font',family='serif', size=10)
rc('text', usetex=True)

table_template = '\
\\begin{{tabular}}{{lcccccccc}}\n\
Method & OSPA$(2)$ & OSPA$(2)$ cardinality & OSPA$(2)$ location & OSPA$(1)$ & OSPA$(1)$ cardinality & OSPA$(1)$ location & Calculation time (s) & Output frequency (Hz) & MOTP(m) & MOTA & Miss rate & False pos. rate & Missmatches\\\\\n\
{base_name} & {base_ospa_2:.2f} & {base_ospa_card_2:.2f} & {base_ospa_loc_2:.2f} & {base_ospa_1:.2f} & {base_ospa_card_1:.2f} & {base_ospa_loc_1:.2f} & {base_calc_time:.2f} & {base_output_freq:.2f} & {base_motp:.2f} & {base_mota:.2f}\\% & {base_miss_rate:.2f}\\% & {base_false_rate:.2f}\\% & {base_miss_match}\\\\\n\
{comp_name}& {compare_ospa_2:.2f} & {compare_ospa_card_2:.2f} & {compare_ospa_loc_2:.2f} & {compare_ospa_1:.2f} & {compare_ospa_card_1:.2f} & {compare_ospa_loc_1:.2f} & {compare_calc_time:.2f} & {compare_output_freq:.2f} & {compare_motp:.2f} & {compare_mota:.2f}\\% & {compare_miss_rate:.2f}\\% & {compare_false_rate:.2f}\\% & {compare_miss_match}\\\\\n\
\\end{{tabular}}'

titles = ('Long corridor scenario',
          'Large hall scenario',
          'Entrance scenario',
          'Very crowded scenario',

          'Long corridor scenario',
          'Large hall scenario',
          'Entrance scenario',
          'Very crowded scenario',

          'Long corridor scenario',
          'Large hall scenario',
          'Entrance scenario',
          'Very crowded scenario',

          'Long corridor using colour camera',
          'Large hall scenario using colour camera',
          'Very crowded scenario with colour camera',

          'Long corridor using depth camera',
          'Large hall scenario using depth camera',
          'Entrance scenario using depth camera',
          'Very crowded scenario with depth camera',

          'Long corridor using GB mixture',
          'Large hall scenario using GB mixture',
          'Entrance scenario using GB mixture',
          'Very crowded scenario with GB mixture',
          
          'RGB-D People Dataser')

base_files = ('base_1_laser.csv',
              'base_2_laser.csv',
              'base_3_laser.csv',
              'base_crowded_laser.csv',

              'base_nn_1_laser.csv',
              'base_nn_2_laser.csv',
              'base_nn_3_laser.csv',
              'base_nn_crowded_laser.csv',

              'base_1_laser.csv',
              'base_2_laser.csv',
              'base_3_laser.csv',
              'base_crowded_laser.csv',

              'error_1_laser.csv',
              'error_2_laser.csv',
              'error_crowded_laser.csv',

              'error_1_laser.csv',
              'error_2_laser.csv',
              'error_3_laser.csv',
              'error_crowded_laser.csv',

              'error_1_laser.csv',
              'error_2_laser.csv',
              'error_3_laser.csv',
              'error_crowded_laser.csv',
              
              'error_depth_dataset.csv')

base_labels = ('JPDA-based tracker',
               'JPDA-based tracker',
               'JPDA-based tracker',
               'JPDA-based tracker',

               'KF with NN association tracker',
               'KF with NN association tracker',
               'KF with NN association tracker',
               'KF with NN association tracker',

               'JPDA-based tracker',
               'JPDA-based tracker',
               'JPDA-based tracker',
               'JPDA-based tracker',

               'PHD and laser based tracker',
               'PHD and laser based tracker',
               'PHD and laser based tracker',
               'PHD and laser based tracker',

               'PHD and laser based tracker',
               'PHD and laser based tracker',
               'PHD and laser based tracker',

               'PHD and laser based tracker',
               'PHD and laser based tracker',
               'PHD and laser based tracker',
               'PHD and laser based tracker',
               
               'Proposed PHD tracker with depth camera')
comp_files = ('error_1_laser.csv',
              'error_2_laser.csv',
              'error_3_laser.csv',
              'error_crowded_laser.csv',

              'error_1_laser.csv',
              'error_2_laser.csv',
              'error_3_laser.csv',
              'error_crowded_laser.csv',
              
              'error_1_leg_laser.csv',
              'error_2_leg_laser.csv',
              'error_3_leg_laser.csv',
              'error_crowded_leg_laser.csv',

              'error_1_vision.csv',
              'error_2_vision.csv',
              'error_crowded_vision.csv',

              'error_1_kinect.csv',
              'error_2_kinect.csv',
              'error_3_kinect.csv',
              'error_crowded_kinect.csv',

              'error_1_beta_vision.csv',
              'error_2_beta_vision.csv',
              'error_3_beta_vision.csv',
              'error_crowded_beta_vision.csv',
              
              'error_depth_dataset.csv')

comp_labels = ('Proposed PHD tracker',
               'Proposed PHD tracker',
               'Proposed PHD tracker',
               'Proposed PHD tracker',

               'Proposed PHD tracker',
               'Proposed PHD tracker',
               'Proposed PHD tracker',
               'Proposed PHD tracker',
               
               'Standard PHD-based tracker',
               'Standard PHD-based tracker',
               'Standard PHD-based tracker',
               'Standard PHD-based tracker',

               'Proposed PHD tracker with colour camera',
               'Proposed PHD tracker with colour camera',
               'Proposed PHD tracker with colour camera',

               'Proposed PHD tracker with depth camera',
               'Proposed PHD tracker with depth camera',
               'Proposed PHD tracker with depth camera',
               'Proposed PHD tracker with depth camera',

               'Proposed BetaGaussian PHD tracker',
               'Proposed BetaGaussian PHD tracker',
               'Proposed BetaGaussian PHD tracker',
               'Proposed BetaGaussian PHD tracker',
               
               'Proposed PHD tracker with depth camera')

groud_truth_files = ('laser_paths_1.yaml',
                     'laser_paths_2.yaml',
                     'laser_paths_3.yaml',
                     'laser_paths_crowded.yaml',
                     
                     'laser_paths_1.yaml',
                     'laser_paths_2.yaml',
                     'laser_paths_3.yaml',
                     'laser_paths_crowded.yaml',
                     
                     'laser_paths_1.yaml',
                     'laser_paths_2.yaml',
                     'laser_paths_3.yaml',
                     'laser_paths_crowded.yaml',

                     'laser_paths_1.yaml',
                     'laser_paths_2.yaml',
                     'laser_paths_crowded.yaml',

                     'laser_paths_1.yaml',
                     'laser_paths_2.yaml',
                     'laser_paths_3.yaml',
                     'laser_paths_crowded.yaml',

                     'laser_paths_1.yaml',
                     'laser_paths_2.yaml',
                     'laser_paths_3.yaml',
                     'laser_paths_crowded.yaml',
                     
                     'depth_paths.yaml')

extras_pdf = ('_1_laser',
              '_2_laser',
              '_3_laser',
              '_crowded_laser',
              
              '_nn_1_laser',
              '_nn_2_laser',
              '_nn_3_laser',
              '_nn_crowded_laser',
              
              '_1_leg_laser',
              '_2_leg_laser',
              '_3_leg_laser',
              '_crowded_leg_laser',

              '_1_vision',
              '_2_vision',
              '_crowded_vision',

              '_1_kinect',
              '_2_kinect',
              '_3_kinect',
              '_crowded_kinect',

              '_1_beta_vision',
              '_2_beta_vision',
              '_3_beta_vision',
              '_crowded_beta_vision',
              
              '_rgdb_people_dataset')

zoom = ((50, 100),
        (150, 200),
        (150, 200),
        (90, 150),

        (50, 100),
        (150, 200),
        (150, 200),
        (90, 150),

        (50, 100),
        (150, 200),
        (150, 200),
        (90, 150),

        (50, 100),
        (150, 200),
        (90, 150),

        (50, 100),
        (150, 200),
        (150, 200),
        (90, 150),

        (50, 100),
        (150, 200),
        (150, 200),
        (90, 150),
        
        (0, 1))
dt = 2.0

def to_percentage(y, pos):
    return "${:.1f}\%$".format(y*100.0)

def plot_error(i, time_base, error_base, time_comp, error_comp):
    plot(time_base, error_base, style1, label=base_labels[i])
    plot(time_comp, error_comp, style2, label=comp_labels[i])

def plot_accum_error(i, time_base, error_base, time_comp, error_comp):
    plot(time_base, np.add.accumulate(error_base), style1, label=base_labels[i])
    plot(time_comp, np.add.accumulate(error_comp), style2, label=comp_labels[i])

def plot_accum2_error(i, time_base, error_base, time_comp, error_comp):
    interval = 0.1
    times = np.arange(min(np.min(time_base), np.min(time_comp)),\
                      max(np.max(time_base), np.max(time_comp)), interval)
#    base_final = np.diff([np.sum(error_base[np.nonzero(time_base < t)[0]]) for t in times]) / dt
#    comp_final = np.diff([np.sum(error_comp[np.nonzero(time_comp < t)[0]]) for t in times]) / dt
    window = np.ones(dt / interval) * interval / dt
    base_final = np.convolve(np.interp(times, time_base, error_base), window, 'same')
    comp_final = np.convolve(np.interp(times, time_comp, error_comp), window, 'same')
    plot(times, base_final, style1, label=base_labels[i])
    plot(times, comp_final, style2, label=comp_labels[i])

for i in range(8):
    base_file = 'data/' + base_files[i]
    comp_file = 'data/' + comp_files[i]
    print base_file, comp_file
    groud_truth_file = 'data/' + groud_truth_files[i]
    extra_pdf = extras_pdf[i]

    data_base = np.array([[float(x) for x in row] for row in csv.reader(open(base_file, 'r'))])
    data_comp = np.array([[float(x) for x in row] for row in csv.reader(open(comp_file, 'r'))])

    pos_base = np.nonzero(data_base[:,1]>0)[0]
    pos_comp = np.nonzero(data_comp[:,1]>0)[0]

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

    start_time = min(np.min(data_base[:,0]), np.min(data_comp[:,0]))
    time_base = data_base[:,0] - start_time
    time_comp = data_comp[:,0] - start_time

    raw_gt_data = yaml.load(open(groud_truth_file, 'r'))
    people_count_gt = (np.array(sum([[float(x[0]/1000000000) + float(x[0]%1000000000)/1000000000 for x in path] for path in raw_gt_data['paths']], [])) - start_time) \
                       if i <> 23 else\
                       np.array(sum([[float(x[2]) for x in path] for path in raw_gt_data], [])) - start_time
    min_gt_time = np.min(people_count_gt)
    max_gt_time = np.max(people_count_gt)
    interval = 0.1 if i <> 23 else 1.0
    times_gt = np.arange(min_gt_time, max_gt_time, interval)
    count_gt = np.zeros(times_gt.shape)
    if i <> 23:
        for j,t in enumerate(times_gt):
            for p in raw_gt_data['paths']:
                t_start = float(p[0][0]/1000000000) + float(p[0][0]%1000000000)/1000000000 - start_time
                t_end = float(p[-1][0]/1000000000) + float(p[-1][0]%1000000000)/1000000000 - start_time
                if t >= t_start and t <= t_end:
                    count_gt[j] += 1
    else:
        for j,t in enumerate(times_gt):
            for p in raw_gt_data:
                t_start = float(p[0][0]) - start_time
                t_end = float(p[-1][0])  - start_time
                if t >= t_start and t <= t_end:
                    count_gt[j] += 1
    window = np.ones(dt / interval) * interval / dt
    count_gt = np.convolve(count_gt, window, 'same')
    style1 = 'r--'
    style2 = 'b-'
    style3 = 'm:'
    style4 = 'y-.'

#    figure()
##    title('OSPA$(2)$ metric for ' + titles[i])
#    fill_between(times_gt, 0, count_gt, color='m', alpha = 0.5, label='Number of people')
#    bar((0), (0), 0, color="m", alpha=0.5, label='Number of people')
#    ylabel('Number of people')
#    twinx()
#    plot_accum2_error(i, time_base, data_base[:,2], time_comp, data_comp[:,2])
#    xlabel('Time (s)')
#    ylabel('OSPA$(2)$ metric')
#    xlim(np.min(time_comp), np.max(time_comp))
#    legend(loc='best')
#    grid()
#    savefig('ospa2'+extra_pdf+'.pdf', bbox_inches='tight')
#
#    figure()
##    title('OSPA$(1)$ metric for ' + titles[i])
#    fill_between(times_gt, 0, count_gt, color='m', alpha = 0.5, label='Number of people')
#    bar((0), (0), 0, color="m", alpha=0.5, label='Number of people')
#    ylabel('Number of people')
#    twinx()
#    plot_accum2_error(i, time_base, data_base[:,5], time_comp, data_comp[:,5])
#    xlabel('Time (s)')
#    ylabel('OSPA$(1)$ metric')
#    xlim(np.min(time_comp), np.max(time_comp))
#    legend(loc='best')
#    grid()
#    savefig('ospa1'+extra_pdf+'.pdf', bbox_inches='tight')
#
#    figure()
#    subplot(211)
#    plot_accum2_error(i, time_base, data_base[:,3], time_comp, data_comp[:,3])
#    ylabel('OSPA$(2)$ cardinality error')
#    grid()
#    legend(loc='best')
#    subplot(212)
#    plot_accum2_error(i, time_base, data_base[:,4], time_comp, data_comp[:,4])
#    xlabel('Time (s)')
#    ylabel('OSPA$(2)$ location error')
#    grid()
#    savefig('ospa2_details'+extra_pdf+'.pdf', bbox_inches='tight')
#
#    figure()
#    subplot(211)
#    plot_accum2_error(i, time_base, data_base[:,6], time_comp, data_comp[:,6])
#    ylabel('OSPA$(1)$ cardinality error')
#    grid()
#    legend(loc='best')
#    subplot(212)
#    plot_accum2_error(i, time_base, data_base[:,7], time_comp, data_comp[:,7])
#    xlabel('Time (s)')
#    ylabel('OSPA$(1)$ location error')
#    grid()
#    savefig('ospa1_details'+extra_pdf+'.pdf', bbox_inches='tight')
#
#    #figure()
#    #title('Calculation time')
#    #subplot(211)
#    #plot(time_base[pos_base], data_base[pos_base,1], style1, label='JPDA-based tracker')
#    #plot(time_comp[pos_comp], data_comp[pos_comp,1], style2, label='PHD-based tracker')
#    #xlabel('Time (s)')
#    #ylabel('Calculation time (s)')
#    #subplot(212)
#    #plot(time_base[pos_base], 1.0 / data_base[pos_base,1], style1, label='JPDA-based tracker')
#    #plot(time_comp[pos_comp], 1.0 / data_comp[pos_comp,1], style2, label='PHD-based tracker')
#    #xlabel('Time (s)')
#    #ylabel('Output frequency (Hz)')
#    #legend(loc='best')
#
#    figure()
#    fill_between(times_gt, 0, count_gt, color='m', alpha = 0.5)
#    plot(time_base, data_base[:,8], style1, label=base_labels[i])
#    plot(time_comp, data_comp[:,8], style2, label=comp_labels[i])
#    axvspan(zoom[i][0], zoom[i][1], alpha=0.2, fc='k')
#    # Dummy bar to display the label
#    bar((0), (0), 0, color="m", alpha=0.5, label='Ground truth')
#    grid()
#    xlabel('Time (s)')
#    ylabel('Number of people')
#    xlim(np.min(time_comp), np.max(time_comp))
#    legend(loc='best')
#    savefig('estimated_number'+extra_pdf+'.pdf', bbox_inches='tight')
#
#    figure()
#    fill_between(times_gt, 0, count_gt, color='m', alpha = 0.5)
#    plot(time_base, data_base[:,8], style1, label=base_labels[i])
#    plot(time_comp, data_comp[:,8], style2, label=comp_labels[i])
#    # Dummy bar to display the label
#    bar((0), (0), 0, color="m", alpha=0.5, label='Ground truth')
#    xlim(zoom[i][0], zoom[i][1])
#    grid()
#    xlabel('Time (s)')
#    ylabel('Number of people')
#    legend(loc='best')
#    savefig('estimated_number_zoom'+extra_pdf+'.pdf', bbox_inches='tight')
#
#    figure()
#    motp1, = plot(time_base, data_base[:,9], style1, label='MOTP ' + base_labels[i])
#    motp2, = plot(time_comp, data_comp[:,9], style2, label='MOTP ' + comp_labels[i])
#    ylabel('MOTP(m)')
#    grid()
#    twinx()
#    mota1, = plot(time_base, data_base[:,10], style3, label='MOTA ' + base_labels[i])
#    mota2, = plot(time_comp, data_comp[:,10], style4, label='MOTA ' + comp_labels[i])
#    formatter = FuncFormatter(to_percentage)
#    gca().yaxis.set_major_formatter(formatter)
#    ylabel('MOTA($\%$)')
#    xlabel('Time (s)')
#    legend((motp1, motp2, mota1, mota2), ('MOTP ' + base_labels[i], 'MOTP ' + comp_labels[i], 'MOTA ' + base_labels[i], 'MOTA ' + comp_labels[i]),  loc='best')
#    savefig('motp_over_time'+extra_pdf+'.pdf', bbox_inches='tight')

    # Print the table
    print titles[i]
    print table_template.format(base_name=base_labels[i],
                                base_ospa_2=np.average(data_base[:,2]),
                                base_ospa_card_2=np.average(data_base[:,3]),
                                base_ospa_loc_2=np.average(data_base[:,4]),
                                base_ospa_1=np.average(data_base[:,5]),
                                base_ospa_card_1=np.average(data_base[:,6]),
                                base_ospa_loc_1=np.average(data_base[:,7]),
                                base_calc_time=np.average(data_base[pos_base,1]),
                                base_output_freq=np.average(1.0 / data_base[pos_base,1]),
                                base_motp=data_base[-1,9],
                                base_mota=data_base[-1,10]*100.0,
                                base_miss_rate=data_base[-1,11]*100.0,
                                base_false_rate=data_base[-1,12]*100.0,
                                base_miss_match=data_base[-1,19],
                                comp_name=comp_labels[i],
                                compare_ospa_2=np.average(data_comp[:,2]),
                                compare_ospa_card_2=np.average(data_comp[:,3]),
                                compare_ospa_loc_2=np.average(data_comp[:,4]),
                                compare_ospa_1=np.average(data_comp[:,5]),
                                compare_ospa_card_1=np.average(data_comp[:,6]),
                                compare_ospa_loc_1=np.average(data_comp[:,7]),
                                compare_calc_time=np.average(data_comp[pos_comp,1]),
                                compare_output_freq=np.average(1.0 / data_comp[pos_comp,1]),
                                compare_motp=data_comp[-1,9],
                                compare_mota=data_comp[-1,10]*100.0,
                                compare_miss_rate=data_comp[-1,11]*100.0,
                                compare_false_rate=data_comp[-1,12]*100.0,
                                compare_miss_match=data_comp[-1,19])
    print
