from matplotlib.pyplot import *
from matplotlib import rc

rc('font',family='serif', size=9, serif='Computer Modern Roman')
rc('text', usetex=True)

#figsize=(7.74,4.78)
figsize=(3.87,2.39)
bbox_to_anchor_1=(0.5, -0.2)
bbox_to_anchor_2=(0.5, -0.4)
import numpy as np

base_method_1 = np.array([0.75291454, 0.69201085, 0.64707548])
base_method_2 = np.array([0.75357416, 0.71536391, 0.65221479])
base_method_3 = np.array([0.74474761, 0.67120264, 0.70054554])
proposed_meth = np.array([0.59947596, 0.56988317, 0.64936067])

base_method_card_1 = np.array([0.58710708, 0.46152791, 0.45945124])
base_method_card_2 = np.array([0.58148579, 0.4952721,  0.46782334])
base_method_card_3 = np.array([0.64547372, 0.51588289, 0.47563141])
proposed_meth_card = np.array([0.43118755, 0.36344463, 0.34429787])

base_method_loc_1 = np.array([0.16580746, 0.23048294, 0.18762424])
base_method_loc_2 = np.array([0.17208837, 0.22009181, 0.18439145])
base_method_loc_3 = np.array([0.09927389, 0.15531975, 0.22491413])
proposed_meth_loc = np.array([0.16828841, 0.20643854, 0.3050628 ])

base_method_grouped_1 = np.array([0.99813384041527475, 0.77752051702789138, 0.69670027639825405, 0.64628097327275014, 0.64981180581021247, 0.58847907667180677, 0.56338300121002616, 0.48672444500046674, 0.56109625297184951, 0.6006403201411471])
base_method_grouped_2 = np.array([0.99841711703487224, 0.75973983943788848, 0.6869652967585288, 0.64910093450745199, 0.65132255249097648, 0.60951374245441192, 0.56180157578893075, 0.50617465233066494, 0.56763735322243491, 0.58836473778452103])
base_method_grouped_3 = np.array([0.99937027040551074, 0.77817810426597167, 0.66746694449489263, 0.63339610396111368, 0.61043712189672994, 0.61966366939660056, 0.58941930711059287, 0.5448930126584457, 0.53460594896950808, 0.54129650313093158])
proposed_method_group = np.array([0.99625871052913795, 0.61209864433305161, 0.55755395598934021, 0.51135805841675264, 0.52147460644938914, 0.52775181803912152, 0.53239093399243431, 0.47442895135561802, 0.4870541181026235, 0.49344076404852943])

base_method_grouped_card_1 = np.array([0.9972879973907377, 0.63840740941820084, 0.46161880146092976, 0.41362139634896034, 0.41099010235494593, 0.37812106781757254, 0.29578700391049839, 0.20344925529450927, 0.14911057477423015, 0.094437017231044112])
base_method_grouped_card_2 = np.array([0.99744114447947207, 0.60405096068467456, 0.45591913457778965, 0.41199737653805019, 0.40403456508301849, 0.42252303721780216, 0.30002010828665648, 0.22167898652266421, 0.17729468599025217, 0.098161189358281664])
base_method_grouped_card_3 = np.array([0.99885109313031284, 0.68882977909012977, 0.52686824783487285, 0.46482332101260493, 0.43074673634194793, 0.4405572698789334, 0.35830702478902782, 0.29648581392757428, 0.17587837936187714, 0.1395194607522465])
proposed_method_group_card = np.array([0.99504410344117766, 0.43900182666768112, 0.35117992594820963, 0.30147671768669126, 0.28991684011826563, 0.29090248672683466, 0.25965104941491285, 0.20874614134097441, 0.16670399648656525, 0.26095420102764721])

base_method_grouped_loc_1 = np.array([0.00084584302453753421, 0.13911310760973614, 0.23508147493731638, 0.23265957692382641, 0.23882170345526105, 0.21035800885416112, 0.26759599729944494, 0.28327518970587434, 0.41198567819760162, 0.50620330290999982])
base_method_grouped_loc_2 = np.array([0.00097597255539990258, 0.15568887875325574, 0.23104616218074478, 0.23710355796943627, 0.24728798740794533, 0.18699070523659381, 0.26178146750216702, 0.28449566580793462, 0.39034266723210431, 0.49020354842612673])
base_method_grouped_loc_3 = np.array([0.0005191772751981001, 0.089348325175875959, 0.14059869666003078, 0.16857278294857589, 0.17969038555478051, 0.1791063995176384, 0.23111228232147671, 0.24840719873076264, 0.3587275696075195, 0.4017770423785727])
proposed_method_group_loc = np.array([0.0012146070879612569, 0.17309681766541032, 0.20637403004108765, 0.20988134073005568, 0.23155776633107095, 0.23684933131227051, 0.27273988457743897, 0.26568281001454758, 0.32035012161597209, 0.23248656302078521])

def do_plot(x, labels, data_1, data_2, data_3, data_4):
    mkr_size = 15
    plot(data_1, 'r-->', markersize=mkr_size, label='JPDA-based tracker')
    plot(data_2, 'm--<', markersize=mkr_size, label='KF with NN association tracker')
    plot(data_3, 'y--*', markersize=mkr_size, label='Standard PHD tracker')
    plot(data_4, 'b-o', markersize=mkr_size, label='PHD-based tracked')
    grid()
    xticks(x, labels)
    xlim((np.min(x) - 0.5, np.max(x) + 0.5))
    ylim(0,1)

def do_bars(x, labels, data_1, data_2, data_3, data_4):
    width = 1. / 6.;
    bar(x-2*width, data_1, width, color='r', hatch='.', label='JPDA-based tracker')
    bar(x-width, data_2, width, color='m', hatch='/', label='KF with NN association tracker')
    bar(x, data_3, width, color='y', hatch='\\', label='Standard PHD tracker')
    bar(x+width, data_4, width, color='b', hatch='x', label='PHD-based tracked')
    grid()
    xticks(x, labels)
    xlim((np.min(x) - 0.5, np.max(x) + 0.5))
    ylim(0,1)
    yticks([0,0.25,0.5,0.75,1])

figure(figsize=figsize)
x = np.array([0, 1, 2])
labels = ('Simple', 'Medium', 'Complex')
do_plot(x, labels, base_method_1, base_method_2, base_method_3, proposed_meth)
legend(loc='upper right')
xlabel('Scenarios')
ylabel('OSPA(1) metric')
savefig('all_together_lines.pdf', bbox_inches='tight')

figure(figsize=figsize)
x = np.array([0, 1, 2])
labels = ('Simple', 'Medium', 'Complex')
do_bars(x, labels, base_method_1, base_method_2, base_method_3, proposed_meth)
#legend(loc='lower right')
legend(loc='upper center', bbox_to_anchor=bbox_to_anchor_1,
       fancybox=True, ncol=2)
xlabel('Scenarios')
ylabel('OSPA(1) metric')
savefig('all_together_bar.pdf', bbox_inches='tight')

figure(figsize=figsize)
x = np.array([0, 1, 2])
labels = ('Simple', 'Medium', 'Complex')
subplot(211)
do_plot(x, labels, base_method_card_1, base_method_card_2, base_method_card_3, proposed_meth_card)
ylabel('Cardinality')
subplot(212)
do_plot(x, labels, base_method_loc_1, base_method_loc_2, base_method_loc_3, proposed_meth_loc)
ylabel('Location')
xlabel('Scenarios')
legend(loc='upper left')
savefig('all_together_details_lines.pdf', bbox_inches='tight')

figure(figsize=figsize)
x = np.array([0, 1, 2])
labels = ('Simple', 'Medium', 'Complex')
subplot(211)
do_bars(x, labels, base_method_card_1, base_method_card_2, base_method_card_3, proposed_meth_card)
ylabel('Cardinality')
subplot(212)
do_bars(x, labels, base_method_loc_1, base_method_loc_2, base_method_loc_3, proposed_meth_loc)
#legend(loc='upper center')
legend(loc='upper center', bbox_to_anchor=bbox_to_anchor_2,
       fancybox=True, ncol=2)
ylabel('Location')
xlabel('Scenarios')
savefig('all_together_details_bar.pdf', bbox_inches='tight')

figure(figsize=figsize)
x = np.arange(1, 10, 1.0)
labels = [str(int(k)) for k in x]
do_plot(x, labels, base_method_grouped_1[1:], base_method_grouped_2[1:], base_method_grouped_3[1:], proposed_method_group[1:])
legend(loc='upper center')
xlabel('Number of people')
ylabel('OSPA(1) metric')
savefig('all_together_gruoped_number_lines.pdf', bbox_inches='tight')

figure(figsize=figsize)
do_bars(x, labels, base_method_grouped_1[1:], base_method_grouped_2[1:], base_method_grouped_3[1:], proposed_method_group[1:])
#legend(loc='lower right')
legend(loc='upper center', bbox_to_anchor=bbox_to_anchor_1,
       fancybox=True, ncol=2)
xlabel('Number of people')
ylabel('OSPA(1) metric')
savefig('all_together_gruoped_number_bars.pdf', bbox_inches='tight')

figure(figsize=figsize)
subplot(211)
do_plot(x, labels, base_method_grouped_card_1[1:], base_method_grouped_card_2[1:], base_method_grouped_card_3[1:], proposed_method_group_card[1:])
ylabel('OSPA(1) metric cardinality')
subplot(212)
do_plot(x, labels, base_method_grouped_loc_1[1:], base_method_grouped_loc_2[1:], base_method_grouped_loc_3[1:], proposed_method_group_loc[1:])
legend(loc='best')
xlabel('Number of people')
ylabel('Location')
savefig('all_together_gruoped_number_detail_lines.pdf', bbox_inches='tight')


figure(figsize=figsize)
subplot(211)
do_bars(x, labels, base_method_grouped_card_1[1:], base_method_grouped_card_2[1:], base_method_grouped_card_3[1:], proposed_method_group_card[1:])
ylabel('Cardinality')
subplot(212)
do_bars(x, labels, base_method_grouped_loc_1[1:], base_method_grouped_loc_2[1:], base_method_grouped_loc_3[1:], proposed_method_group_loc[1:])
#legend(loc='upper center')
legend(loc='upper center', bbox_to_anchor=bbox_to_anchor_2,
       fancybox=True, ncol=2)
xlabel('Number of people')
ylabel('Location')
savefig('all_together_gruoped_number_detail_bar.pdf', bbox_inches='tight')
