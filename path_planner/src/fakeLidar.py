#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan


class FakeLiDAR(object):
    def __init__(self):
        super(FakeLiDAR, self).__init__()

        self.msg = LaserScan()

    def publishFakeLiDAR(self, pub):
        msg = LaserScan()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "velodyne"

        msg.angle_min = -3.14159274101
        msg.angle_max = 3.14159274101
        msg.angle_increment = 0.00700000021607
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = 0.0
        msg.range_max = 200.0
        msg.ranges = [0.9498553276062012, 0.9238592982292175, 0.945855975151062, 0.933857798576355, 0.9398568272590637, 0.9398568868637085, 0.9378571510314941, 0.9478557109832764, 0.9258590340614319, 0.9378571510314941, 0.9318581223487854, 0.9398568868637085, 0.9678526520729065, 0.9518550038337708, 0.9758514165878296, 1.0118459463119507, 1.0238440036773682, 1.007846474647522, 1.0058468580245972, 0.9598538279533386, float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), 15.193686485290527, 15.013713836669922, 14.989717483520508, 14.945724487304688, 15.217682838439941, float("inf"), float("inf"), 20.252914428710938, 19.972959518432617, 19.93896484375, 19.9969539642334, float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), 24.85621452331543, 24.832218170166016, 25.132171630859375, 10.886341094970703, 10.70436954498291, 10.67037582397461, 10.648378372192383, 10.742363929748535, float("inf"), float("inf"), float("inf"), 29.673480987548828, 29.797460556030273, float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), 15.45164680480957, 15.403655052185059, 15.499640464782715, 15.607623100280762, 15.72360610961914, 15.803593635559082, 14.667766571044922, 14.581780433654785, 14.497793197631836, 6.47701358795166, 6.441019535064697, 6.4230217933654785, 6.401025295257568, 6.419022560119629, 6.4510178565979, 6.481013298034668, 6.555001735687256, 13.713912010192871, 6.3730292320251465, 6.2890424728393555, 6.277044296264648, 6.267045497894287, 6.253047943115234, 6.273044586181641, 6.279043197631836, 6.311039447784424, 6.403025150299072, 13.022017478942871, 12.972024917602539, 12.892037391662598, 12.846043586730957, 12.79005241394043, 12.726061820983887, 12.676068305969238, 12.620077133178711, 12.554088592529297, 12.506095886230469, 12.43610668182373, 12.394113540649414, 12.332121849060059, 12.286128044128418, 12.24013614654541, 12.214139938354492, 12.152149200439453, 12.09815788269043, 12.062163352966309, 12.02216911315918, 11.968177795410156, 11.930183410644531, 11.86219310760498, 11.846196174621582, 11.792204856872559, 11.76420783996582, 11.734214782714844, 11.690218925476074, 11.664223670959473, 11.632227897644043, 11.600234031677246, 11.568238258361816, 11.516246795654297, 11.50424861907959, 11.452256202697754, 11.432259559631348, 11.400263786315918, 11.366270065307617, 11.342273712158203, 11.324275016784668, 11.292280197143555, 11.268282890319824, 11.236289024353027, 11.236289978027344, 11.20429515838623, 11.168299674987793, 11.140303611755371, 11.11230754852295, 11.09631061553955, 11.066315650939941, 11.064315795898438, 11.024321556091309, 11.010323524475098, 11.006324768066406, 10.980327606201172, 10.946333885192871, 10.950333595275879, 10.922337532043457, 10.924336433410645, 10.88634204864502, 10.882344245910645, 10.870346069335938, 10.858345985412598, 10.860345840454102, 10.830350875854492, 10.834351539611816, 10.822352409362793, 10.798355102539062, 10.796355247497559, 10.776359558105469, 10.76636028289795, 10.760360717773438, float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), 61.638614654541016, float("inf"), float("inf"), float("inf"), float("inf"), 4.9392476081848145, 4.863259792327881, 4.831264495849609, 4.801268577575684, 4.793270111083984, 4.7832722663879395, 4.799269676208496, 4.779272079467773, 4.791270732879639, 4.79526948928833, 4.807267665863037, 4.835264205932617, 4.87125825881958, 10.970328330993652, 10.950331687927246, 10.958333015441895, 10.982327461242676, 11.004323959350586, 11.01632308959961, 11.0363187789917, 11.044318199157715, 11.08231258392334, 11.088312149047852, 11.110308647155762, 11.140303611755371, 11.156301498413086, 11.198294639587402, 11.218291282653809, 11.242288589477539, 11.258285522460938, 11.27828311920166, 11.312276840209961, 11.344273567199707, 11.382266998291016, float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), 30.127412796020508, float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), 22.696544647216797, 22.636554718017578, float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), 17.841283798217773, 17.809288024902344, float("inf"), 33.32292556762695, float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), 21.536720275878906, 21.464731216430664, float("inf"), float("inf"), 36.054508209228516, 7.442866802215576, 7.4148712158203125, 7.374876499176025, 7.36887788772583, 7.338881969451904, 7.336882591247559, 7.340882301330566, 7.322884559631348, 7.396873474121094, 7.452865123748779, 7.4728617668151855, 7.488860130310059, 7.510856628417969, 7.552849769592285, 7.640836238861084, float("inf"), 28.725624084472656, 28.541654586791992, 28.625638961791992, 28.597644805908203, 28.713624954223633, float("inf"), 24.278303146362305, 24.12232780456543, 24.010345458984375, 23.978349685668945, 23.970352172851562, 23.978347778320312, 24.04833984375, 24.204313278198242, float("inf"), float("inf"), 25.332141876220703, float("inf"), 23.49642562866211, 23.736387252807617, 30.96728515625, 30.99527931213379, float("inf"), 28.827611923217773, float("inf"), 26.283998489379883, 26.188013076782227, 26.188013076782227, 22.580562591552734, 22.620555877685547, 22.640552520751953, 22.6805477142334, 22.76453399658203, 12.144150733947754, 11.996172904968262, 11.892189979553223, 11.79620361328125, 11.804203033447266, 11.944180488586426, 9.908491134643555, float("inf"), float("inf"), float("inf"), 57.89318084716797, float("inf"), float("inf"), 25.182165145874023, 25.17816734313965, 25.23015785217285, 16.38750457763672, 16.13154411315918, 16.04355812072754, 26.413978576660156, 26.72992706298828, 26.961896896362305, float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), 21.148778915405273, 21.076793670654297, 21.028797149658203, 21.336750030517578, float("inf"), float("inf"), float("inf"), float("inf"), 25.73008155822754, float("inf"), float("inf"), 55.29758071899414, 55.225589752197266, 55.217594146728516, 55.1735954284668, 55.17359924316406, 55.09761047363281, 55.0576171875, 55.04561233520508, 54.98563003540039, 54.95763397216797, 54.9576301574707, 54.94163131713867, float("inf"), float("inf"), float("inf"), float("inf"), 54.89764404296875, 54.89763641357422, float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), 55.50754928588867, 55.52754211425781, float("inf"), float("inf"), 44.16527557373047, 36.16249465942383, float("inf"), float("inf"), float("inf"), 32.127105712890625, 31.983129501342773, 31.843151092529297, 31.967132568359375, float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), 14.459798812866211,
                      14.531786918640137, 14.607775688171387, 14.535786628723145, 14.579779624938965, 14.441800117492676, 14.495793342590332, 14.341815948486328, 14.373809814453125, 14.157844543457031, 14.271827697753906, 14.201837539672852, 14.201837539672852, 14.163843154907227, 14.063858032226562, 14.117850303649902, 14.075857162475586, 14.145846366882324, 14.105852127075195, 14.073856353759766, 14.193839073181152, 14.16584300994873, 12.358118057250977, 12.522092819213867, 11.848196029663086, 11.964177131652832, 11.928183555603027, 11.878190994262695, 11.860194206237793, 11.776206970214844, 11.70821762084961, 11.658225059509277, 11.57023811340332, 11.606232643127441, 11.660223960876465, 11.662223815917969, 11.746211051940918, 11.834197998046875, 12.04416561126709, 12.080161094665527, 12.080161094665527, 12.600081443786621, 12.65207290649414, 12.69206714630127, 12.872038841247559, 16.247526168823242, 16.183536529541016, 16.065553665161133, 15.855587005615234, 15.67961311340332, 15.431649208068848, 15.291671752929688, 15.23568058013916, 15.109699249267578, 14.993715286254883, 14.855737686157227, 14.779748916625977, 11.26828384399414, 11.322275161743164, 13.251982688903809, 13.345966339111328, 14.343815803527832, 11.000325202941895, 14.221835136413574, 14.2518310546875, 14.353814125061035, 14.407806396484375, 14.499792098999023, 14.569780349731445, 14.6317720413208, 9.352575302124023, 9.342577934265137, 9.272587776184082, 9.030624389648438, 9.03662395477295, 9.068619728088379, 9.076617240905762, 9.128609657287598, 9.11661148071289, 9.208597183227539, 9.260590553283691, float("inf"), 11.032320022583008, 12.00017261505127, 10.66037654876709, float("inf"), 10.956332206726074, 10.224443435668945, 10.648378372192383, 10.948332786560059, 10.360422134399414, 11.116308212280273, 10.548393249511719, 9.782510757446289, 9.802506446838379, 13.469950675964355, 9.630534172058105, 9.822503089904785, 9.810505867004395, 9.520549774169922, 9.926488876342773, 11.150301933288574, 10.210445404052734, 10.176450729370117, 10.01447582244873, 11.066315650939941, 9.650530815124512, 9.90249252319336, 9.744516372680664, 10.250439643859863, 9.72052001953125, 9.714520454406738, 9.678525924682617, 10.030472755432129, 10.430411338806152, 10.75036334991455, 9.870497703552246, 9.490554809570312, 10.090462684631348, 9.642531394958496, 12.634076118469238, 9.710521697998047, 7.762818336486816, 7.750819683074951, 7.378876209259033, 7.378876209259033, 7.294889450073242, 7.398873805999756, 7.2029032707214355, 7.154910087585449, 7.198903560638428, 7.24489688873291, 7.278891563415527, 7.074922561645508, 7.042927265167236, 7.102919101715088, 7.012932300567627, 6.930944919586182, 6.974937915802002, 7.00093412399292, 6.990935325622559, 6.900949478149414, 6.938943862915039, 6.934944152832031, 7.010932922363281, 7.010932445526123, 6.996934413909912, 7.258894443511963, 7.230898857116699, 7.260894775390625, 7.24489688873291, 7.236897945404053, 7.272892951965332, 7.322885036468506, 7.314886093139648, 7.462863922119141, 7.636837482452393, 7.650835037231445, 7.618839263916016, 7.650835037231445, 7.680830478668213, 7.706826210021973, 7.690828800201416, 7.716825008392334, 7.852804660797119, 8.452712059020996, 8.468708992004395, 12.74605941772461, float("inf"), 12.746058464050293, 9.492554664611816, 12.746058464050293, 12.74605941772461, 9.65652847290039, 9.638532638549805, 9.552545547485352, 9.542547225952148, float("inf"), 12.27013111114502, 10.598386764526367, 10.534396171569824, 10.48640251159668, 10.468405723571777, 10.356423377990723, 10.344425201416016, 10.364421844482422, 10.352423667907715, 10.40441608428955, 10.424412727355957, 10.428411483764648, 10.450409889221191, 10.430411338806152, 9.324579238891602, 9.356575965881348, 9.38257122039795, 10.582388877868652, 9.066619873046875, 8.880647659301758, 8.856651306152344, 8.762665748596191, 8.828656196594238, 8.776663780212402, 8.81065845489502, 8.844653129577637, 8.850652694702148, float("inf"), 9.386570930480957, 9.366573333740234, 9.398569107055664, float("inf"), float("inf"), 13.094006538391113, float("inf"), float("inf"), float("inf"), 9.40456771850586, 9.428564071655273, float("inf"), 13.395960807800293, float("inf"), float("inf"), 13.50394344329834, float("inf"), float("inf"), 13.675918579101562, float("inf"), float("inf"), 13.8038969039917, float("inf"), float("inf"), 14.029863357543945, float("inf"), float("inf"), 14.145846366882324, float("inf"), float("inf"), 14.293824195861816, float("inf"), float("inf"), 14.513789176940918, float("inf"), 9.878495216369629, float("inf"), float("inf"), 14.945723533630371, 13.213987350463867, float("inf"), 15.089703559875488, float("inf"), 10.682374000549316, 10.686372756958008, 15.645617485046387, 15.517638206481934, 14.14384651184082, 15.743603706359863, float("inf"), 16.139541625976562, 15.907577514648438, float("inf"), 16.167537689208984, float("inf"), 16.53948211669922, 16.739450454711914, 16.739450454711914, float("inf"), 16.939420700073242, 16.859434127807617, 17.191381454467773, 12.584083557128906, 12.27413272857666, 12.956027030944824, 17.593318939208984, 17.581323623657227, 12.674070358276367, 17.941267013549805, 18.141237258911133, 18.469186782836914, 18.393198013305664, 13.985869407653809, 13.82989501953125, 13.669919967651367, 13.633923530578613, 13.453950881958008, float("inf"), 14.629772186279297, 14.673765182495117, 12.518094062805176, 12.446105003356934, 12.28412914276123, 12.300127983093262, 12.264132499694824, 12.34611988067627, 12.34611988067627, 12.258132934570312, 12.290128707885742, 12.398112297058105, 12.34611988067627, 12.234137535095215, 12.340121269226074, 12.418109893798828, 12.368117332458496, 12.524093627929688, 12.432106971740723, 12.624076843261719, 13.044013023376465, 13.77590274810791, 13.2559814453125, 13.37596321105957, 25.012191772460938, 25.280149459838867, 25.572107315063477, 17.113393783569336, 17.10739517211914, float("inf"), 18.713151931762695, float("inf"), float("inf"), float("inf"), float("inf"), 18.241222381591797, float("inf"), 17.74129867553711, 17.629314422607422, 17.553329467773438, 17.64931297302246, 1.1198294162750244, 1.1458255052566528, 1.1098309755325317, 1.1238288879394531, 1.0598387718200684, 1.0598386526107788, 1.0598387718200684, 1.0318429470062256, 1.0218443870544434, 1.0298432111740112, 1.0058469772338867, 1.0178450345993042, 0.9978480339050293, 1.0218443870544434, 1.037842035293579, 0.985849916934967, 1.0218443870544434, 1.0218443870544434, 1.0298432111740112, 1.0298430919647217, 1.0578389167785645, 1.037842035293579, 1.0218443870544434, 1.107831358909607, 1.1598232984542847, float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), 44.113285064697266, 44.11328125, float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), float("inf"), 0.9998477101325989, 0.9998476505279541, 1.0118459463119507, 0.957854151725769, 1.0058468580245972, 0.9338578581809998, 0.957854151725769, 0.957854151725769, 0.949855387210846]
        msg.intensities = [10.0, 19.0, 36.0, 42.0, 39.0, 39.0, 40.0, 35.0, 45.0, 23.0, 18.0, 22.0, 15.0, 14.0, 6.0, 1.0, 1.0, 1.0, 5.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 6.0, 12.0, 6.0, 6.0, 0.0, 0.0, 10.0, 32.0, 32.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 24.0, 24.0, 12.0, 3.0, 7.0, 10.0, 10.0, 10.0, 0.0, 0.0, 0.0, 13.0, 13.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 12.0, 12.0, 12.0, 12.0, 12.0, 6.0, 23.0, 23.0, 17.0, 11.0, 13.0, 13.0, 13.0, 13.0, 13.0, 13.0, 12.0, 5.0, 11.0, 10.0, 14.0, 18.0, 14.0, 14.0, 16.0, 14.0, 3.0, 36.0, 36.0, 35.0, 25.0, 30.0, 30.0, 25.0, 20.0, 20.0, 15.0, 15.0, 25.0, 35.0, 24.0, 19.0, 14.0, 29.0, 34.0, 29.0, 24.0, 33.0, 33.0, 23.0, 23.0, 27.0, 32.0, 32.0, 31.0, 31.0, 31.0, 31.0, 30.0, 30.0, 30.0, 34.0, 29.0, 29.0, 29.0, 28.0, 28.0, 28.0, 28.0, 28.0, 28.0, 28.0, 28.0, 28.0, 28.0, 27.0, 31.0, 27.0, 27.0, 27.0, 31.0, 31.0, 26.0, 26.0, 30.0, 26.0, 26.0, 26.0, 26.0, 26.0, 26.0, 25.0, 25.0, 25.0, 29.0, 29.0, 25.0, 29.0, 25.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 58.0, 0.0, 0.0, 0.0, 0.0, 12.0, 19.0, 16.0, 32.0, 27.0, 22.0, 14.0, 14.0, 14.0, 17.0, 14.0, 17.0, 15.0, 11.0, 30.0, 30.0, 31.0, 27.0, 27.0, 27.0, 27.0, 27.0, 27.0, 28.0, 28.0, 28.0, 28.0, 28.0, 28.0, 28.0, 28.0, 24.0, 20.0, 12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 14.0, 0.0, 0.0, 0.0, 0.0, 0.0, 11.0, 11.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 8.0, 8.0, 0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 11.0, 11.0, 0.0, 0.0, 25.0, 8.0, 5.0, 8.0, 8.0, 10.0, 8.0, 8.0, 8.0, 8.0, 6.0, 7.0, 7.0, 7.0, 2.0, 1.0, 0.0, 13.0, 13.0, 13.0, 13.0, 13.0, 0.0, 12.0, 36.0, 12.0, 24.0, 24.0, 12.0, 24.0, 24.0, 0.0, 0.0, 12.0, 0.0, 11.0, 12.0, 15.0, 15.0, 0.0, 13.0, 0.0, 25.0, 12.0, 25.0, 69.0, 69.0, 58.0, 46.0, 58.0, 4.0, 9.0, 4.0, 4.0, 4.0, 4.0, 8.0, 0.0, 0.0, 0.0, 60.0, 0.0, 0.0, 12.0, 12.0, 12.0, 6.0, 13.0, 19.0, 12.0, 12.0, 13.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 11.0, 11.0, 22.0, 11.0, 0.0, 0.0, 0.0, 0.0, 12.0, 0.0, 0.0, 56.0, 56.0, 56.0, 56.0, 56.0, 56.0, 56.0, 56.0, 56.0, 56.0,
                           56.0, 56.0, 0.0, 0.0, 0.0, 0.0, 56.0, 56.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 57.0, 0.0, 0.0, 39.0, 25.0, 0.0, 0.0, 0.0, 17.0, 17.0, 17.0, 17.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 11.0, 11.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 5.0, 5.0, 17.0, 17.0, 17.0, 17.0, 28.0, 11.0, 16.0, 5.0, 16.0, 16.0, 5.0, 5.0, 10.0, 5.0, 14.0, 14.0, 23.0, 19.0, 14.0, 13.0, 13.0, 13.0, 13.0, 13.0, 13.0, 13.0, 13.0, 13.0, 14.0, 19.0, 19.0, 5.0, 15.0, 10.0, 15.0, 27.0, 33.0, 32.0, 25.0, 25.0, 43.0, 42.0, 42.0, 42.0, 42.0, 42.0, 36.0, 12.0, 4.0, 16.0, 5.0, 34.0, 19.0, 17.0, 40.0, 28.0, 40.0, 35.0, 35.0, 23.0, 6.0, 14.0, 14.0, 11.0, 13.0, 13.0, 13.0, 8.0, 10.0, 8.0, 6.0, 0.0, 3.0, 4.0, 3.0, 0.0, 7.0, 3.0, 3.0, 3.0, 6.0, 4.0, 3.0, 18.0, 2.0, 5.0, 2.0, 2.0, 2.0, 7.0, 8.0, 4.0, 3.0, 9.0, 8.0, 3.0, 2.0, 2.0, 7.0, 3.0, 7.0, 7.0, 7.0, 5.0, 3.0, 3.0, 11.0, 4.0, 2.0, 5.0, 5.0, 7.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 4.0, 2.0, 8.0, 9.0, 1.0, 4.0, 6.0, 9.0, 7.0, 10.0, 10.0, 7.0, 6.0, 11.0, 10.0, 7.0, 9.0, 7.0, 4.0, 2.0, 9.0, 9.0, 8.0, 8.0, 6.0, 8.0, 8.0, 7.0, 5.0, 1.0, 1.0, 7.0, 7.0, 4.0, 6.0, 6.0, 2.0, 3.0, 6.0, 5.0, 0.0, 5.0, 7.0, 5.0, 5.0, 12.0, 2.0, 11.0, 4.0, 0.0, 4.0, 3.0, 3.0, 3.0, 6.0, 6.0, 9.0, 9.0, 9.0, 9.0, 9.0, 9.0, 6.0, 9.0, 6.0, 8.0, 2.0, 3.0, 1.0, 5.0, 6.0, 4.0, 8.0, 11.0, 11.0, 8.0, 1.0, 0.0, 2.0, 6.0, 2.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 4.0, 4.0, 0.0, 10.0, 0.0, 0.0, 16.0, 0.0, 0.0, 11.0, 0.0, 0.0, 11.0, 0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0.0, 0.0, 11.0, 0.0, 0.0, 5.0, 0.0, 2.0, 0.0, 0.0, 6.0, 10.0, 0.0, 6.0, 0.0, 3.0, 7.0, 6.0, 6.0, 5.0, 6.0, 0.0, 6.0, 6.0, 0.0, 6.0, 0.0, 7.0, 7.0, 7.0, 0.0, 7.0, 7.0, 7.0, 5.0, 4.0, 15.0, 8.0, 8.0, 5.0, 8.0, 8.0, 9.0, 9.0, 5.0, 11.0, 5.0, 5.0, 16.0, 0.0, 5.0, 5.0, 5.0, 5.0, 14.0, 19.0, 24.0, 5.0, 5.0, 9.0, 14.0, 5.0, 5.0, 14.0, 15.0, 5.0, 15.0, 10.0, 10.0, 10.0, 5.0, 11.0, 5.0, 5.0, 12.0, 12.0, 12.0, 22.0, 22.0, 0.0, 9.0, 0.0, 0.0, 0.0, 0.0, 8.0, 0.0, 8.0, 8.0, 24.0, 16.0, 15.0, 40.0, 37.0, 5.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 3.0, 4.0, 3.0, 3.0, 5.0, 3.0, 3.0, 3.0, 3.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 39.0, 39.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 3.0, 4.0, 1.0, 18.0, 9.0, 9.0, 5.0]

        self.msg = msg

        pub.publish(self.msg)


if __name__ == "__main__":
    rospy.init_node("fake_lidar")

    fake_lidar = FakeLiDAR()

    lidar_pub = rospy.Publisher("fake_scan", LaserScan, queue_size=1)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        fake_lidar.publishFakeLiDAR(pub=lidar_pub)
        r.sleep()