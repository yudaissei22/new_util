# 周波数を使って一定時間ごとに処理をする。
# 目標は、下投げの動作で、効率よくトルクを手先に伝えること。効率の良さはどう評価するのかについてだが、ログのとり方は教わったので、頑張ってやる。
# また、パラメータを調整する学習の際に、各情報を所得しなくてはいけないのだが、それはどうやるのかな？

import numpy as np

hcf.ast_scv.startAutoBalancer()
hcf.ast_scv.startStabilizer()

def underthrow (z_lhand_force, time):
    freqs = np.linspace(0, 1, 44100) #0から1を44100個の等間隔な数列を生成し、freqsに代入 
    #p,gゲインを弱める
    hcf.rh_svc.setServoPGainPercentageWithTime()
    hcf.rh_svc.setServoDGainPercentageWithTime()

    hcf.seq_svc.setWrenches([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, z_lhand_force, 0, 0, 0], time)
    status = hcf.rh_svc.getStatus2() #collect information and use for learning
    























#参考、これで引数を抑える。
# (angle=[1.8382326743449084e-05, -0.0014965584268793464, -0.3581376373767853, 0.6957328915596008, -0.332487553358078, 0.001510852831415832, 8.449992310488597e-05, -0.0015110961394384503, -0.3581049144268036, 0.6957061290740967, -0.33249348402023315, 0.0015257281484082341, 0.001918549882248044, 0.00016276029055006802, -0.0007011259440332651, 1.8170107068726793e-05, -0.0004078514175489545, 0.04919617995619774, 0.38609790802001953, -0.34693971276283264, -0.08525306731462479, -1.3262698650360107, -0.005336196627467871, 0.005878939293324947, -0.2691209018230438, -0.04689059406518936, 0.3826257884502411, 0.3487054407596588, 0.0860007256269455, -1.3165860176086426, 0.008889561519026756, -0.01101891789585352, -0.2757045030593872, -0.011464565061032772, 0.0046496763825416565, 0.003955571912229061, 0.0008296977030113339, 0.002687811153009534, 0.00015243304369505495, 0.011465301737189293, 0.004651476163417101, -0.007195560727268457, -0.0019200168317183852, -0.002687301253899932, -0.00015246096882037818], command=[4.063703090650961e-05, -0.0014958882238715887, -0.3582160174846649, 0.6938580274581909, -0.33250632882118225, 0.001512912567704916, 6.051413947716355e-05, -0.0015011858195066452, -0.35818150639533997, 0.6938356161117554, -0.33251842856407166, 0.00151827244553715, 0.0019070161506533623, 9.147416858468205e-05, -0.0007008729735389352, 1.84030595846707e-05, -0.0005825846455991268, 0.04920324683189392, 0.3856300711631775, -0.34747856855392456, -0.0856088325381279, -1.3271249532699585, -0.005387031007558107, 0.005766186397522688, -0.26940247416496277, -0.04689577966928482, 0.3821163773536682, 0.34927481412887573, 0.08638268709182739, -1.3174824714660645, 0.008940563537180424, -0.010892174206674099, -0.27601173520088196, -0.007611044216901064, 0.0030968086794018745, 0.0026419847272336483, 0.0005542858853004873, 0.0017948979511857033, 0.0001016817259369418, 0.007611478678882122, 0.003097698325291276, -0.0048095183447003365, -0.0012826357269659638, -0.0017946691950783134, -0.0001016962924040854], torque=[0.7344041466712952, 0.05562223494052887, -2.5863170623779297, -61.8708610534668, -0.8820649981498718, 0.0679691806435585, -0.7915298938751221, 0.822555422782898, -2.527359962463379, -61.72639465332031, -1.171849250793457, -0.24603760242462158, -0.9572969675064087, -5.916748046875, 0.020995765924453735, 0.0023295257706195116, -1.747332215309143, 0.14135994017124176, -9.356504440307617, -10.777242660522461, -7.11531400680542, -17.101247787475586, -1.0166908502578735, -2.255061626434326, -5.631669521331787, -0.10369636118412018, -10.187966346740723, 11.387482643127441, 7.639195442199707, -17.930192947387695, 1.0200427770614624, 2.534877300262451, -6.144493103027344, 0.15414083003997803, -0.06211470440030098, -0.05254349112510681, -0.011016471311450005, -0.035716526210308075, -0.002030052710324526, -0.15415291488170624, -0.062151119112968445, 0.09544169157743454, 0.025495242327451706, 0.035705287009477615, 0.002030587289482355], servoState=[[7, 0, 0, 0, 942305710, 932852671, 932852671, 0, 0, 942305710, 0, 1065353216, 1065353216, 1060897257, 1065353216, 0, 0], [7, 0, 0, 0, -1161555538, -1161549781, -1161549781, 0, 0, -1161555538, 0, 1065353216, 1065353216, 1029952548, 1065353216, 0, 0], [7, 0, 0, 0, -1095276521, -1095279151, -1095279151, 0, 0, -1095276521, 0, 1065353216, 1065353216, -1071282632, 1065353216, 0, 0], [7, 0, 0, 0, 1060217006, 1060248461, 1060248461, 0, 0, 1060217006, 0, 1065353216, 1065353216, -1032356925, 1065353216, 0, 0], [7, 0, 0, 0, -1096139195, -1096139825, -1096139825, 0, 0, -1096139195, 0, 1065353216, 1065353216, -1084109053, 1065353216, 0, 0], [7, 0, 0, 0, 986074348, 986056655, 986056655, 0, 0, 986074348, 0, 1065353216, 1065353216, 1032532845, 1065353216, 0, 0], [7, 0, 0, 0, 947769488, 951137677, 951137677, 0, 0, 947769488, 0, 1065353216, 1065353216, -1085627980, 1065353216, 0, 0], [7, 0, 0, 0, -1161510032, -1161424903, -1161424903, 0, 0, -1161510032, 0, 1065353216, 1065353216, 1062376190, 1065353216, 0, 0], [7, 0, 0, 0, -1095277679, -1095280249, -1095280249, 0, 0, -1095277679, 0, 1065353216, 1065353216, -1071529916, 1065353216, 0, 0], [7, 0, 0, 0, 1060216630, 1060248012, 1060248012, 0, 0, 1060216630, 0, 1065353216, 1065353216, -1032394796, 1065353216, 0, 0], [7, 0, 0, 0, -1096138789, -1096139626, -1096139626, 0, 0, -1096138789, 0, 1065353216, 1065353216, -1080688856, 1065353216, 0, 0], [7, 0, 0, 0, 986120389, 986184433, 986184433, 0, 0, 986120389, 0, 1065353216, 1065353216, -1099173560, 1065353216, 0, 0], [7, 0, 0, 0, 989459672, 989558746, 989558746, 0, 0, 989459672, 0, 1065353216, 1065353216, -1082846870, 1065353216, 0, 0], [7, 0, 0, 0, 952096210, 959097506, 959097506, 0, 0, 952096210, 0, 1065353216, 1065353216, -1061333504, 1065353216, 0, 0], [7, 0, 0, 0, -1170752822, -1170748476, -1170748476, 0, 0, -1170752822, 0, 1065353216, 1065353216, 1017904976, 1065353216, 0, 0], [7, 0, 0, 0, 932864069, 932736002, 932736002, 0, 0, 932864069, 0, 1065353216, 1065353216, 991472373, 1065353216, 0, 0], [7, 0, 0, 0, -1172785000, -1177168668, -1177168668, 0, 0, -1172785000, 0, 1065353216, 1065353216, -1075861355, 1065353216, 0, 0], [7, 0, 0, 0, 1028229464, 1028227567, 1028227567, 0, 0, 1028229464, 0, 1065353216, 1065353216, 1041285289, 1065353216, 0, 0], [7, 0, 0, 0, 1053126990, 1053142688, 1053142688, 0, 0, 1053126990, 0, 1065353216, 1065353216, -1055542210, 1065353216, 0, 0], [7, 0, 0, 0, -1095636810, -1095654891, -1095654891, 0, 0, -1095636810, 0, 1065353216, 1065353216, -1054052458, 1065353216, 0, 0], [7, 0, 0, 0, -1112583249, -1112630999, -1112630999, 0, 0, -1112583249, 0, 1065353216, 1065353216, -1058819929, 1065353216, 0, 0], [7, 0, 0, 0, -1079386309, -1079393482, -1079393482, 0, 0, -1079386309, 0, 1065353216, 1065353216, -1047998629, 1065353216, 0, 0], [7, 0, 0, 0, -1146059343, -1146168509, -1146168509, 0, 0, -1146059343, 0, 1065353216, 1065353216, -1081990419, 1065353216, 0, 0], [7, 0, 0, 0, 1002238535, 1002480670, 1002480670, 0, 0, 1002238535, 0, 1065353216, 1065353216, -1072672018, 1065353216, 0, 0], [7, 0, 0, 0, -1098256609, -1098266057, -1098266057, 0, 0, -1098256609, 0, 1065353216, 1065353216, -1061931357, 1065353216, 0, 0], [7, 0, 0, 0, -1119873590, -1119874982, -1119874982, 0, 0, -1119873590, 0, 1065353216, 1065353216, -1110155582, 1065353216, 0, 0], [7, 0, 0, 0, 1053009090, 1053026183, 1053026183, 0, 0, 1053009090, 0, 1065353216, 1065353216, -1054670359, 1065353216, 0, 0], [7, 0, 0, 0, 1051907110, 1051888005, 1051888005, 0, 0, 1051907110, 0, 1065353216, 1065353216, 1094071073, 1065353216, 0, 0], [7, 0, 0, 0, 1035004264, 1034952998, 1034952998, 0, 0, 1035004264, 0, 1065353216, 1065353216, 1089762378, 1065353216, 0, 0], [7, 0, 0, 0, -1079467196, -1079474716, -1079474716, 0, 0, -1079467196, 0, 1065353216, 1065353216, -1047564023, 1065353216, 0, 0], [7, 0, 0, 0, 1007844209, 1007789446, 1007789446, 0, 0, 1007844209, 0, 1065353216, 1065353216, 1065521347, 1065353216, 0, 0], [7, 0, 0, 0, -1137543913, -1137407823, -1137407823, 0, 0, -1137543913, 0, 1065353216, 1065353216, 1075985262, 1065353216, 0, 0], [7, 0, 0, 0, -1098034839, -1098045148, -1098045148, 0, 0, -1098034839, 0, 1065353216, 1065353216, -1060855888, 1065353216, 0, 0], [7, 0, 0, 0, -1141283311, -1136929313, -1136929313, 0, 0, -1141283311, 0, 1065353216, 1065353216, 1042143000, 1065353216, 0, 0], [7, 0, 0, 0, 994767828, 999840848, 999840848, 0, 0, 994767828, 0, 1065353216, 1065353216, -1115788291, 1065353216, 0, 0], [7, 0, 0, 0, 992814374, 998350270, 998350270, 0, 0, 992814374, 0, 1065353216, 1065353216, -1118357544, 1065353216, 0, 0], [7, 0, 0, 0, 974212479, 978944018, 978944018, 0, 0, 974212479, 0, 1065353216, 1065353216, -1137410450, 1065353216, 0, 0], [7, 0, 0, 0, 988496584, 993011197, 993011197, 0, 0, 988496584, 0, 1065353216, 1065353216, -1122874498, 1065353216, 0, 0], [7, 0, 0, 0, 953499126, 958387823, 958387823, 0, 0, 953499126, 0, 1065353216, 1065353216, -1157297502, 1065353216, 0, 0], [7, 0, 0, 0, 1006201270, 1010555126, 1010555126, 0, 0, 1006201270, 0, 1065353216, 1065353216, -1105339837, 1065353216, 0, 0], [7, 0, 0, 0, 994771649, 999844713, 999844713, 0, 0, 994771649, 0, 1065353216, 1065353216, -1115778516, 1065353216, 0, 0], [7, 0, 0, 0, -1147299542, -1142175555, -1142175555, 0, 0, -1147299542, 0, 1065353216, 1065353216, 1036220143, 1065353216, 0, 0], [7, 0, 0, 0, -1163387363, -1157912301, -1157912301, 0, 0, -1163387363, 0, 1065353216, 1065353216, 1020320614, 1065353216, 0, 0], [7, 0, 0, 0, -1158989029, -1154474641, -1154474641, 0, 0, -1158989029, 0, 1065353216, 1065353216, 1024606133, 1065353216, 0, 0], [7, 0, 0, 0, -1193982520, -1189093906, -1189093906, 0, 0, -1193982520, 0, 1065353216, 1065353216, 990188442, 1065353216, 0, 0]], force=[[-0.26312774419784546, 2.601254940032959, -509.6684875488281, 0.14652492105960846, -0.9118222594261169, 0.7063502669334412], [-0.2654876112937927, 2.6329903602600098, -507.67828369140625, -0.028813226148486137, 1.2684030532836914, -0.7590181231498718], [-28.67852020263672, -10.941460609436035, 10.500028610229492, 1.1458405256271362, -3.4521002769470215, -0.46761834621429443], [30.17745018005371, -11.51586627960205, 11.060680389404297, 1.341780185699463, 3.851008892059326, 0.34864023327827454]], rateGyro=[[-5.584603890440576e-10, -1.1827482326864924e-09, 1.6163081983933125e-09]], accel=[[-9.806520462036133, -0.00016127309936564416, -0.0500972718000412]], batteries=[], voltage=48.25003810715865, current=0.6301802565624753, temperature=[])


