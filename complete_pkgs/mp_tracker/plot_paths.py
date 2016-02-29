import numpy as np
from matplotlib.pyplot import figure, plot, show

path_0=[
[1.3309, -50.2249, -85.8747, -0.826837, -0.282489, 1.74662], 
[1.44164, -50.2843, -85.9181, -0.770452, -0.363046, 1.7149], 
[1.54225, -50.3145, -85.965, -0.520486, -0.421498, 1.70098], 
]
path_1=[
[1.54225, -50.3145, -85.965, -0.520486, -0.421498, 1.70098], 
[1.68321, -50.3088, -86.01, -0.230818, -0.319002, 1.67899], 
[1.80407, -50.257, -86.1054, -0.0564104, -0.419625, 1.67292], 
[1.92523, -50.2426, -86.14, -0.0420514, -0.408977, 1.6391], 
[2.00614, -50.1945, -86.23, 0.0481354, -0.497991, 1.62226], 
[2.10673, -50.151, -86.3079, 0.124022, -0.569563, 1.5943], 
[2.19784, -50.1223, -86.4056, 0.131363, -0.624832, 1.57286], 
[2.32036, -50.0794, -86.5107, 0.164857, -0.671431, 1.55693], 
[2.40109, -50.0124, -86.6422, 0.238647, -0.78749, 1.54138], 
[2.49203, -49.8414, -86.8575, 0.46521, -0.992984, 1.53235], 
[2.6028, -49.6601, -87.1149, 0.610854, -1.19821, 1.51996], 
[2.68341, -49.5993, -87.2002, 0.620722, -1.16624, 1.51045], 
[2.76393, -49.5523, -87.2801, 0.622124, -1.1408, 1.49163], 
[2.85444, -49.4587, -87.4107, 0.656294, -1.179, 1.48448], 
[2.96559, -49.3392, -87.569, 0.708531, -1.19138, 1.4759], 
[3.04616, -49.2809, -87.6406, 0.723111, -1.14905, 1.46558], 
[3.13713, -49.2161, -87.7362, 0.737487, -1.14124, 1.4521], 
[3.2176, -49.151, -87.8311, 0.740529, -1.13847, 1.44583], 
[3.31821, -49.0508, -87.9612, 0.783025, -1.17657, 1.44471], 
[3.39869, -48.9737, -88.0509, 0.824495, -1.15819, 1.43972], 
[3.52051, -48.8593, -88.1735, 0.866479, -1.11896, 1.43311], 
[3.6011, -48.7586, -88.2783, 0.908946, -1.13759, 1.43082], 
[3.68167, -48.642, -88.3951, 1.00686, -1.20437, 1.42889], 
[3.76212, -48.5633, -88.4908, 0.988737, -1.19254, 1.42132], 
[3.83259, -48.4818, -88.5743, 1.03507, -1.20036, 1.41603], 
[3.91315, -48.4058, -88.656, 1.02424, -1.18044, 1.40637], 
[4.00663, -48.3259, -88.7491, 0.994221, -1.13674, 1.39913], 
[4.07982, -48.267, -88.8138, 0.986033, -1.11552, 1.39014], 
[4.16036, -48.2069, -88.8827, 0.94957, -1.07475, 1.38196], 
[4.23555, -48.1489, -88.9559, 0.928409, -1.05907, 1.36983], 
[4.31911, -48.0779, -89.0506, 0.898426, -1.06654, 1.36233], 
[4.38965, -48.0022, -89.1324, 0.910793, -1.07937, 1.36261], 
[4.46038, -47.8207, -89.3527, 1.11946, -1.43954, 1.36105], 
[4.53083, -47.7495, -89.4108, 1.12727, -1.34124, 1.35815], 
[4.60138, -47.6778, -89.4929, 1.11927, -1.31885, 1.34962], 
[4.69429, -47.5931, -89.5984, 1.07802, -1.29146, 1.33853], 
[4.79489, -47.5083, -89.701, 1.03777, -1.23063, 1.32253], 
[4.88546, -47.4855, -89.7922, 0.86934, -1.17758, 1.29809], 
[4.98689, -47.5171, -89.8588, 0.664605, -1.10347, 1.27049], 
[5.06744, -47.6547, -89.7942, 0.340288, -0.869019, 1.24626], 
[5.16831, -47.649, -89.8488, 0.243825, -0.794245, 1.2347], 
[5.24918, -47.7109, -89.9242, 0.135795, -0.945766, 1.21884], 
]
path_2=[
[14.6295, -51.7986, -77.79, -0.0205411, 0.112915, 1.71319], 
]
path_3=[
[22.4125, -55.4816, -69.8308, 0.341092, 0.0250988, 1.65092], 
]
path_4=[
[24.9478, -54.6866, -71.6171, -0.295664, 0.107949, 1.67338], 
[25.0586, -54.9317, -71.3109, -0.53446, 0.0812787, 1.60156], 
]
path_5=[
[27.9002, -45.3316, -76.1058, -0.693682, 0.591336, 1.6866], 
]
path_6=[
[45.6659, -54.1924, -74.4916, -0.144558, -0.0132358, 1.71661], 
[45.7665, -54.3392, -74.6899, -0.906418, -0.474854, 1.65353], 
[45.8573, -54.3494, -74.6514, -0.633866, -0.218034, 1.61899], 
[45.9479, -54.3893, -74.658, -0.65082, -0.174071, 1.56875], 
[46.0294, -54.4453, -74.7618, -0.752294, -0.26333, 1.53284], 
]
path_7=[
[46.0294, -54.4453, -74.7618, -0.752294, -0.26333, 1.53284], 
[46.1905, -54.3229, -74.7675, -0.320273, -0.0851766, 1.5035], 
[46.3516, -54.2284, -74.8195, -0.0999872, -0.137952, 1.47851], 
[46.4826, -54.1641, -74.8727, -0.00886252, -0.135928, 1.43789], 
[46.5741, -54.0915, -74.94, 0.049344, -0.176235, 1.40388], 
[46.6751, -54.048, -75.0078, 0.0281383, -0.185279, 1.36333], 
[46.7558, -53.9864, -75.1061, 0.0580888, -0.26283, 1.32638], 
[46.8466, -53.9921, -75.1329, 0.0188806, -0.244285, 1.28711], 
[46.9478, -53.9988, -75.1582, -0.0518934, -0.195837, 1.25975], 
[47.0205, -53.9632, -75.2303, -0.0402686, -0.250498, 1.22009], 
[47.1312, -53.9784, -75.2719, -0.0630832, -0.258851, 1.17951], 
[47.2017, -54.0149, -75.3182, -0.121804, -0.272203, 1.13921], 
[47.2822, -54.0471, -75.3442, -0.155399, -0.265701, 1.11133], 
[47.4231, -54.1092, -75.3507, -0.198219, -0.189042, 1.08327], 
[47.4835, -54.101, -75.3846, -0.182553, -0.222917, 1.0704], 
[47.564, -54.119, -75.4325, -0.19697, -0.248657, 1.03359], 
[47.6444, -54.1353, -75.4732, -0.201379, -0.291915, 1.00617], 
[47.7153, -54.1044, -75.5478, -0.164024, -0.353395, 0.990414], 
[47.8214, -54.0797, -75.591, -0.115728, -0.35894, 0.98034], 
[47.9021, -54.0155, -75.6642, -0.0369337, -0.417269, 0.981575], 
[47.9841, -53.95, -75.7253, 0.0498377, -0.45802, 0.974542], 
[48.0951, -53.8016, -75.874, 0.207865, -0.627023, 0.970481], 
[48.1755, -53.68, -76.1611, 0.232094, -0.912649, 0.974829], 
[48.2461, -53.5669, -76.4379, 0.260705, -1.15459, 0.974232], 
[48.3424, -53.5837, -76.771, 0.0611874, -1.35404, 0.957165], 
[48.433, -53.4889, -77.1673, 0.056227, -1.64665, 0.949007], 
[48.5436, -53.493, -77.3395, 0.0467733, -1.62596, 0.935296], 
[48.6341, -53.6374, -77.321, -0.171825, -1.35414, 0.936146], 
[48.7852, -53.5905, -77.3178, -0.0813972, -1.06323, 0.915503], 
[48.8459, -53.5875, -77.4476, -0.105357, -1.16429, 0.897469], 
[48.9164, -53.5797, -77.581, -0.149342, -1.21336, 0.883783], 
]
path_8=[
[49.8789, -48.4198, -84.5924, 0.138523, -0.084894, 1.6568], 
[50.0773, -48.3163, -84.7045, 0.273139, -0.552855, 1.64167], 
]
path_9=[
[50.9748, -46.4211, -83.8326, 0.0112278, 0.0673665, 1.75401], 
]
path_10=[
[53.4488, -46.4304, -83.6817, -0.273401, 0.0695378, 1.74156], 
[53.5136, -46.4451, -83.6824, -0.262111, 0.0834251, 1.75123], 
[53.6488, -46.386, -83.6392, 0.056852, 0.161958, 1.76685], 
[53.7252, -46.3518, -83.6073, 0.268468, 0.290297, 1.77499], 
[53.8202, -46.3546, -83.5886, 0.132401, 0.278339, 1.77053], 
[53.888, -46.3576, -83.5822, 0.0613182, 0.249405, 1.76683], 
]
path_11=[
[59.5413, -47.3527, -76.38, -0.166681, 0.635041, 1.62043], 
[59.7801, -47.2817, -76.5351, -0.114153, -0.0749472, 1.58949], 
[59.9174, -47.3442, -76.483, -0.218478, 0.0823306, 1.58093], 
[60.7558, -48.3991, -76.0078, -0.144611, 0.223034, 1.71163], 
]
path_12=[
[59.5413, -47.0583, -78.4386, -0.286434, -0.0273021, 1.76118], 
[59.7801, -47.1237, -78.422, -0.388065, 0.0955107, 1.73752], 
]
path_13=[
[59.5413, -47.2495, -76.4927, 0.373555, -0.766676, 1.627], 
[59.7801, -47.2817, -76.5351, -0.114153, -0.0749472, 1.58949], 
[59.9174, -47.3442, -76.483, -0.218478, 0.0823306, 1.58093], 
]
path_14=[
[60.3899, -45.1838, -74.9524, 0.0200399, 0.617319, 1.65191], 
[60.4897, -45.1894, -74.9473, 0.038747, 0.672806, 1.64882], 
[60.5877, -45.1921, -74.9069, 0.0378092, 0.701514, 1.64506], 
[60.6764, -45.1941, -74.8618, 0.0308891, 0.715429, 1.64345], 
[60.7558, -45.1807, -74.797, 0.0520493, 0.731093, 1.64366], 
[60.852, -45.1822, -74.7346, 0.0466245, 0.749269, 1.64352], 
[60.9184, -45.1869, -74.7041, 0.0187866, 0.744613, 1.63889], 
[60.9873, -45.1976, -74.6695, 0.0200946, 0.719305, 1.63601], 
[61.0824, -45.1825, -74.6577, 0.0077699, 0.620165, 1.62868], 
[61.209, -45.163, -74.6399, -0.0166677, 0.53917, 1.62018], 
[61.3302, -45.1636, -74.6452, -0.0171198, 0.361078, 1.61169], 
[61.4062, -45.155, -74.6501, -0.00891074, 0.280119, 1.60365], 
[61.4164, -45.1533, -74.6685, -0.00908099, 0.236867, 1.59562], 
[61.5528, -45.1641, -74.6608, -0.0313702, 0.171383, 1.58257], 
[61.6178, -45.1625, -74.6785, -0.0178857, 0.0942203, 1.56671], 
[61.7806, -45.1734, -74.7262, -0.055896, -0.0550365, 1.54865], 
[61.8872, -45.1719, -74.7545, -0.0310393, -0.115208, 1.53641], 
[61.9753, -45.1673, -74.7883, -0.0268985, -0.179106, 1.52979], 
[61.9853, -45.1714, -74.8096, -0.036645, -0.225002, 1.51453], 
[62.1433, -45.1634, -74.8308, -0.0245381, -0.212735, 1.50421], 
[62.2512, -45.1675, -74.8758, -0.0249773, -0.251036, 1.49231], 
[62.3503, -45.1694, -74.9221, -0.039501, -0.323421, 1.4788], 
[62.5119, -45.1806, -74.9729, -0.0587504, -0.324646, 1.46751], 
[62.5798, -45.1836, -75.007, -0.0551497, -0.36616, 1.45489], 
[62.6493, -45.1899, -75.0437, -0.0695974, -0.385323, 1.43694], 
[62.7181, -45.1817, -75.0857, -0.0328178, -0.426087, 1.42871], 
[62.8407, -45.1736, -75.1584, -0.00330061, -0.461802, 1.41921], 
[62.9185, -45.1629, -75.1771, 0.0243632, -0.447571, 1.41129], 
]
path_15=[
[65.7705, -47.2326, -78.7317, -0.0486572, 0.00871206, 1.64568], 
[65.8839, -47.253, -78.7949, -0.0942572, -0.320259, 1.62679], 
[65.8939, -47.3129, -78.8815, -0.572696, -0.711254, 1.61746], 
[66.0816, -47.4089, -78.9986, -0.514637, -0.671314, 1.60508], 
[66.2009, -47.4721, -79.0515, -0.529118, -0.588455, 1.59193], 
[66.2915, -47.6382, -79.2135, -0.787309, -0.84219, 1.55978], 
]
path_16=[
[78.6452, -46.6526, -83.9638, -0.12169, 0.0588401, 1.69666], 
[78.7267, -46.6308, -83.9425, -0.155107, 0.0906332, 1.67198], 
]
path_17=[
[84.3191, -52.8833, -86.7804, 0.249487, 0.181714, 1.74274], 
[84.3846, -52.8549, -86.7589, 0.147229, 0.416517, 1.72325], 
[84.4669, -52.8263, -86.727, 0.29744, 0.445817, 1.68918], 
[84.5373, -52.7187, -86.6451, 0.374345, 0.502578, 1.67875], 
[84.8433, -52.7218, -86.5046, 0.131649, 0.626807, 1.69283], 
[84.9089, -52.7334, -86.5216, 0.116972, 0.688649, 1.68788], 
[84.9758, -52.7886, -86.4901, -0.00453553, 0.780382, 1.68604], 
[85.1025, -52.7773, -86.4226, -0.023089, 0.73904, 1.68427], 
[85.2375, -52.7612, -86.3703, -0.00972604, 0.709361, 1.67262], 
[85.2816, -52.7433, -86.3697, 0.121717, 0.671137, 1.66387], 
[85.4255, -52.7119, -86.2905, 0.138987, 0.659167, 1.65813], 
[85.5063, -52.6768, -86.2768, 0.195867, 0.590494, 1.65756], 
[85.6214, -52.5969, -86.2975, 0.289711, 0.427614, 1.66466], 
[85.6416, -52.5557, -86.2978, 0.375532, 0.412115, 1.66248], 
[85.784, -52.4565, -86.2661, 0.40327, 0.357289, 1.67929], 
[85.8734, -52.369, -86.2768, 0.477457, 0.357976, 1.67757], 
[85.966, -52.2746, -86.2399, 0.560672, 0.378801, 1.67135], 
[86.0595, -52.2106, -86.211, 0.465389, 0.40525, 1.64934], 
[86.136, -52.0745, -86.1666, 0.561365, 0.40865, 1.63771], 
[86.2665, -51.9637, -86.1757, 0.548791, 0.276873, 1.63822], 
[86.3486, -51.8704, -86.1417, 0.62648, 0.265613, 1.58862], 
[86.5135, -51.7127, -86.0669, 0.661381, 0.32731, 1.56394], 
[86.6101, -51.558, -86.0092, 0.922901, 0.212568, 1.55305], 
[86.6781, -51.4883, -85.9459, 0.97144, 0.292028, 1.52295], 
[86.7886, -51.5201, -86.002, 0.6881, 0.248338, 1.51191], 
[86.877, -51.5092, -86.0025, 0.611833, 0.242511, 1.46835], 
[86.9424, -51.4878, -85.9949, 0.563426, 0.229662, 1.43006], 
[87.0461, -51.344, -86.0407, 0.632671, 0.0757336, 1.44885], 
[87.1109, -51.1623, -86.1125, 0.772649, -0.134444, 1.49667], 
[87.2962, -51.023, -86.2471, 0.789805, -0.267081, 1.53637], 
]
path_18=[
[84.3846, -53.0326, -86.875, -1.32583, -0.939355, 1.63548], 
[84.4669, -53.0891, -86.9051, -1.34964, -0.815093, 1.61615], 
[84.5373, -53.0301, -86.8201, -1.12537, -0.547502, 1.66407], 
[84.9089, -53.1482, -86.9118, -0.706913, -0.759906, 1.60712], 
[84.9758, -53.1897, -86.8655, -0.689482, -0.670177, 1.6129], 
]
path_19=[
[86.6101, -51.8469, -86.1352, 0.324105, 0.30975, 1.52734], 
[86.6781, -51.7428, -86.11, 0.387447, 0.233334, 1.53717], 
]
path_20=[
[87.1109, -51.2281, -86.3559, -0.542473, 0.15287, 1.70218], 
]
path_21=[
[88.496, -49.6658, -85.9897, -0.499459, 0.519619, 1.72554], 
[88.5729, -49.6339, -85.9276, -0.510173, 0.576359, 1.69354], 
[88.7154, -49.5695, -85.8396, -0.508381, 0.611459, 1.62491], 
[88.9892, -49.5851, -85.686, -0.379606, 0.483088, 1.5871], 
[89.1002, -49.5403, -85.6531, -0.11667, 0.417338, 1.57842], 
[89.2086, -49.5173, -85.6242, -0.0280005, 0.38338, 1.55526], 
[89.3029, -49.4582, -85.6201, 0.0522297, 0.320404, 1.523], 
[89.4563, -49.3809, -85.6068, 0.0748716, 0.321398, 1.53359], 
[89.7042, -49.4569, -85.5861, -0.138457, 0.332301, 1.51063], 
[90.4102, -48.8902, -85.9319, -0.161396, -0.0155001, 1.73508], 
[90.5159, -48.8453, -85.9487, -0.433633, -0.404747, 1.6655], 
[90.8779, -48.9078, -86.0303, -0.261361, -0.346112, 1.66735], 
[90.9831, -48.8776, -86.0224, -0.209683, -0.253181, 1.6546], 
[91.1429, -48.7749, -85.9754, 0.0274183, -0.153669, 1.63539], 
]
path_22=[
[89.4563, -49.3809, -85.6068, 0.0748716, 0.321398, 1.53359], 
[89.5482, -49.3634, -85.5583, 0.0732885, 0.295021, 1.51689], 
[89.7042, -49.129, -85.473, 0.51074, 0.0626665, 1.46938], 
[89.8131, -49.2114, -85.5271, 0.230594, 0.0711433, 1.44515], 
[90.0113, -49.1936, -85.5007, 0.195378, 0.119463, 1.4258], 
[90.0755, -49.1561, -85.5435, 0.18032, -0.00153853, 1.40437], 
[90.1163, -49.1105, -85.5415, 0.213342, -0.0268936, 1.39245], 
[90.2131, -49.0722, -85.5468, 0.231701, -0.049995, 1.38232], 
[90.4102, -49.0218, -85.5839, 0.195996, -0.0941882, 1.36153], 
[90.5159, -48.9825, -85.6003, 0.192092, -0.0649206, 1.33513], 
[90.8779, -48.9308, -85.6807, 0.145878, -0.10993, 1.33977], 
[90.9831, -48.8719, -85.6875, 0.205277, -0.100917, 1.33983], 
[91.1429, -48.8052, -85.7203, 0.223641, -0.11757, 1.34127], 
[91.2664, -48.6623, -85.8331, 0.313726, -0.149545, 1.45727], 
[91.3369, -48.6244, -85.8132, 0.360352, -0.161869, 1.41949], 
[91.4228, -48.5688, -85.7893, 0.424576, -0.137923, 1.38145], 
[91.4826, -48.4911, -85.7427, 0.501205, -0.0772916, 1.35178], 
[91.6138, -48.3578, -85.7001, 0.610915, -0.0329922, 1.30068], 
[91.6879, -48.2803, -85.6727, 0.672809, -0.0132572, 1.27155], 
[91.8264, -48.168, -85.6281, 0.693832, 0.0595222, 1.25428], 
[91.9336, -48.1124, -85.6181, 0.651701, 0.0802753, 1.23271], 
[92.0032, -48.0611, -85.5951, 0.649931, 0.114582, 1.22447], 
[92.1397, -47.9792, -85.5579, 0.612737, 0.180064, 1.20097], 
[92.2162, -47.9362, -85.5439, 0.594388, 0.192675, 1.19113], 
[92.9467, -47.7202, -85.0223, 0.342674, 0.2775, 1.66323], 
[93.1129, -47.6076, -84.9258, 0.429979, 0.327216, 1.65745], 
[93.2467, -47.5632, -84.8889, 0.415363, 0.330193, 1.65969], 
[93.3527, -47.4641, -84.8355, 0.529788, 0.342262, 1.65637], 
[93.4825, -47.2395, -84.6867, 1.09823, 0.743463, 1.63521], 
[93.5883, -47.0723, -84.5469, 0.983773, 0.732384, 1.66121], 
[93.7173, -46.9002, -84.4093, 1.0609, 0.771962, 1.67458], 
[93.8436, -46.8207, -84.3402, 0.96919, 0.735475, 1.66597], 
[93.9475, -46.7538, -84.315, 0.942246, 0.614899, 1.65907], 
[94.0567, -46.6818, -84.2689, 0.926181, 0.594259, 1.6563], 
[94.1833, -46.5189, -84.1382, 1.04801, 0.711116, 1.65763], 
[94.2865, -46.4852, -84.1711, 0.924943, 0.544725, 1.64867], 
[94.3492, -46.4678, -84.2301, 0.865233, 0.365156, 1.6367], 
[94.4124, -46.4678, -84.1476, 0.781826, 0.543886, 1.62748], 
[94.5505, -46.3815, -84.114, 0.756356, 0.461165, 1.61328], 
[94.6692, -46.3148, -84.0562, 0.791342, 0.488979, 1.60716], 
]
path_23=[
[90.9831, -48.8719, -85.6875, 0.205277, -0.100917, 1.33983], 
[91.1429, -48.8052, -85.7203, 0.223641, -0.11757, 1.34127], 
[92.2162, -48.0896, -85.3229, 0.365668, -0.0901952, 1.72994], 
[92.4473, -48.014, -85.2692, 0.155161, 0.059733, 1.70401], 
[92.5936, -47.9425, -85.1981, 0.188975, 0.146, 1.69043], 
[92.712, -47.8601, -85.1165, 0.281753, 0.25602, 1.6842], 
[92.7902, -47.7965, -85.0583, 0.316355, 0.327824, 1.67179], 
[92.9467, -47.7202, -85.0223, 0.342674, 0.2775, 1.66323], 
[93.4825, -47.3859, -84.7958, 0.541181, 0.273002, 1.64524], 
]
path_24=[
[92.2162, -47.9362, -85.5439, 0.594388, 0.192675, 1.19113], 
[92.4473, -47.8113, -85.4876, 0.563405, 0.219248, 1.18105], 
[92.5936, -47.7485, -85.4376, 0.492092, 0.289814, 1.15879], 
[92.712, -47.681, -85.3799, 0.468657, 0.33604, 1.1518], 
[92.7902, -47.6118, -85.3224, 0.536736, 0.401307, 1.15641], 
[92.9467, -47.4778, -85.2216, 0.574914, 0.464775, 1.15768], 
[93.1129, -47.3323, -85.1254, 0.607672, 0.464035, 1.14632], 
[93.2467, -47.2293, -85.059, 0.609838, 0.45846, 1.13752], 
[93.3527, -47.1391, -84.9932, 0.627727, 0.477025, 1.12896], 
[93.4825, -47.0595, -84.9337, 0.592628, 0.478304, 1.11494], 
[93.5883, -46.9886, -84.881, 0.61849, 0.508611, 1.10689], 
[93.7173, -46.9149, -84.8339, 0.58384, 0.48471, 1.10225], 
[93.8436, -46.8293, -84.7774, 0.58863, 0.470593, 1.09837], 
[93.9475, -46.7457, -84.749, 0.613107, 0.432368, 1.0955], 
[94.0567, -46.6443, -84.6798, 0.650197, 0.44893, 1.09017], 
[94.6692, -46.4137, -84.094, 0.453527, 0.466588, 1.59745], 
[94.7095, -46.4102, -84.079, 0.393525, 0.462238, 1.58892], 
[94.8416, -46.394, -84.0449, 0.274161, 0.426183, 1.5861], 
[94.9076, -46.3915, -84.0126, 0.19632, 0.487103, 1.58073], 
]
path_25=[
[96.912, -50.7299, -86.591, -0.244564, 0.129, 1.72522], 
[96.9862, -50.7591, -86.561, -0.358849, 0.241047, 1.70654], 
[97.0545, -50.7871, -86.5556, -0.453297, 0.156502, 1.68475], 
]
path_26=[
[98.7858, -52.2639, -85.5978, -0.189981, -0.00651513, 1.7317], 
[98.9326, -52.348, -85.7247, -0.289765, -0.490799, 1.69171], 
[99.0187, -52.2964, -85.6846, 0.179693, -0.252413, 1.65227], 
[99.1089, -52.415, -85.6701, -0.404948, -0.0679661, 1.67482], 
[99.1822, -52.3177, -85.9057, -0.410813, -0.209586, 1.68501], 
[99.2727, -52.4304, -85.8518, -0.620989, -0.121919, 1.64746], 
[99.3536, -52.4966, -85.9067, -0.779069, -0.135191, 1.61563], 
]
path_27=[
[99.0187, -52.4598, -85.8129, -0.909224, -0.558172, 1.6716], 
[99.3536, -52.4966, -85.9067, -0.779069, -0.135191, 1.61563], 
]
path_28=[
[99.7163, -52.0919, -85.1325, 0.0187603, 0.375421, 1.77041], 
[99.9866, -51.5357, -84.991, -0.143481, 0.137646, 1.71795], 
[100.603, -51.0655, -85.1965, 0.0481548, 0.114849, 1.63185], 
[100.689, -50.9932, -85.1631, 0.140674, 0.157582, 1.61056], 
[100.721, -50.9094, -85.1359, 0.399572, 0.152438, 1.60184], 
[100.949, -50.6153, -85.0329, 1.04606, 0.264452, 1.59992], 
[101.051, -50.5179, -84.9759, 0.982071, 0.404023, 1.58683], 
]
path_29=[
[101.235, -53.1215, -86.0815, -0.106872, -0.621603, 1.69896], 
[101.308, -53.1961, -86.0998, -0.269156, -0.513617, 1.65586], 
[101.405, -53.1515, -86.0535, -0.385424, -0.229397, 1.65843], 
[101.49, -53.0644, -86.0081, -0.0791526, -0.105906, 1.6406], 
[101.65, -53.1048, -86.0487, -0.136729, -0.135261, 1.60142], 
[101.752, -53.0919, -86.0926, -0.147552, -0.225747, 1.57554], 
[101.854, -53.0442, -86.0823, -0.0590528, -0.183117, 1.53557], 
[101.958, -53.0183, -86.0949, -0.0157491, -0.20507, 1.51585], 
[102.04, -52.8949, -86.0004, 0.364956, -0.0354786, 1.55258], 
[102.12, -52.9292, -86.0167, 0.339638, -0.01877, 1.53748], 
[102.321, -52.9072, -86.0246, 0.252638, -0.00996506, 1.53304], 
[102.426, -52.9393, -86.0088, 0.349941, -0.00213271, 1.54555], 
[102.558, -52.9168, -86.0759, 0.25706, -0.00654453, 1.49519], 
[102.713, -52.9096, -86.0103, 0.172917, 0.177751, 1.48808], 
[102.723, -52.9646, -86.037, 0.0366612, 0.126033, 1.45215], 
[102.955, -53.0096, -86.1167, -0.0465677, -0.0641018, 1.40504], 
[102.965, -52.9844, -86.0753, -0.0735496, 0.0426015, 1.41043], 
[103.16, -52.9962, -86.04, -0.113385, 0.118446, 1.40295], 
[103.389, -53.0505, -85.9578, -0.131903, 0.202802, 1.40885], 
[103.557, -53.048, -85.9915, -0.14955, 0.136486, 1.38033], 
[103.766, -52.9601, -85.9263, -0.0210941, 0.247809, 1.35988], 
[103.861, -52.9502, -85.9028, -0.0262885, 0.271897, 1.34576], 
]
path_30=[
[102.04, -53.2256, -86.1842, -0.418499, -0.310993, 1.48916], 
[102.12, -53.244, -86.2156, -0.436101, -0.344373, 1.47375], 
[102.321, -53.2681, -86.2371, -0.378542, -0.206764, 1.46485], 
[102.426, -53.2535, -86.2099, -0.400943, -0.130294, 1.46334], 
[102.558, -53.3241, -86.3151, -0.564035, -0.428984, 1.47414], 
[102.713, -53.2693, -86.1717, -0.425097, -0.107117, 1.44532], 
[102.723, -53.2447, -86.1641, -0.4217, -0.0963775, 1.43471], 
[102.955, -53.3881, -86.1067, -0.288109, -0.334202, 1.56305], 
[102.965, -53.3185, -86.0391, -0.300275, -0.46746, 1.55615], 
[103.16, -53.3323, -85.9191, -0.271537, -0.229307, 1.64499], 
[103.28, -53.2805, -85.868, -0.261596, -0.0688331, 1.63729], 
]
path_31=[
[102.955, -53.0096, -86.1167, -0.0465677, -0.0641018, 1.40504], 
[102.965, -52.9844, -86.0753, -0.0735496, 0.0426015, 1.41043], 
[103.16, -52.9962, -86.04, -0.113385, 0.118446, 1.40295], 
[103.28, -53.0234, -85.9707, -0.106736, 0.205888, 1.41765], 
[103.389, -53.0505, -85.9578, -0.131903, 0.202802, 1.40885], 
[103.557, -53.048, -85.9915, -0.14955, 0.136486, 1.38033], 
[103.689, -52.9656, -85.932, -0.0305569, 0.219506, 1.36561], 
[103.766, -52.9601, -85.9263, -0.0210941, 0.247809, 1.35988], 
[103.861, -52.9502, -85.9028, -0.0262885, 0.271897, 1.34576], 
[104.046, -52.9574, -85.8772, -0.0680202, 0.212293, 1.33316], 
[104.142, -52.9586, -85.8687, -0.0600937, 0.180354, 1.32442], 
[104.219, -53.0805, -85.884, -0.225067, 0.188747, 1.30494], 
[104.352, -53.0783, -85.7816, -0.248668, 0.364301, 1.28836], 
]
path_32=[
[109.889, -53.616, -85.2236, -0.185907, -0.0241696, 1.72792], 
[110.014, -53.5504, -85.1648, 0.252096, 0.33138, 1.69725], 
]

figure()
paths = dir()
for p in paths:
    if p.find('path_') == 0:
        path = np.array(eval(p))
        plot(path[:,0], path[:,1], 'k-')
show()