# -*- coding: utf-8 -*-
# @Author: Heethesh Vhavle
# @Date:   Nov 20, 2019
# @Last Modified by:   Heethesh Vhavle
# @Last Modified time: Nov 20, 2019

import numpy as np

# [0, 1, 2,  3,  4,  5,  6]
# [x, y, r, vx, vy, vr, th]
features = np.array([[0.01171875, 0.04296875, 0.04357195273041725, 0.0, 0.0, 0.0, 1.304544266811189],
    [0.037109375, 0.078125, 0.0871439054608345, 0.0, 0.0, 0.0, 1.127347914205568],
    [0.0859375, -0.09765625, 0.13071586191654205, 0.0, -0.0, 0.0, -0.8491414567207128],
    [0.115234375, 0.130859375, 0.174287810921669, 0.0, 0.0, 0.0, 0.8488050994750359],
    [0.470703125, 0.08984375, 0.47929146885871887, 0.0, 0.0, 0.0, 0.1886028213572419],
    [1.580078125, 1.9765625, 2.5271732807159424, 0.0, 0.0, 0.0, 0.8964170268644452],
    [1.94140625, -1.689453125, 2.5707452297210693, 0.0, -0.0, 0.0, -0.7161170929034729],
    [1.974609375, -1.71875, 2.6143171787261963, 0.0, -0.0, 0.0, -0.7162331401479731],
    [2.41796875, -1.396484375, 2.788604974746704, 0.0, -0.0, 0.0, -0.5237444172204125],
    [3.138671875, -1.52734375, 3.4857561588287354, 0.0, -0.0, 0.0, -0.4528872547038147],
    [3.12109375, -1.65625, 3.5293281078338623, 0.0, -0.0, 0.0, -0.48787629786895287],
    [3.16015625, -1.677734375, 3.5729000568389893, 0.0, -0.0, 0.0, -0.4880627857242857],
    [4.416015625, 0.556640625, 4.444339275360107, 0.0, 0.0, 0.0, 0.1253891309062532],
    [4.458984375, 0.5625, 4.487911224365234, 0.0, 0.0, 0.0, 0.12548695214204317],
    [4.501953125, 0.56640625, 4.531483173370361, 0.0, 0.0, 0.0, 0.12515585463951906],
    [5.341796875, 0.50390625, 5.359350204467773, 0.0, 0.0, 0.0, 0.09405440248751609],
    [5.3984375, 0.337890625, 5.4029221534729, 0.0, 0.0, 0.0, 0.06250890983563445],
    [5.138671875, 2.5, 5.707925796508789, 0.0, 0.0, 0.0, 0.45279507604058433],
    [5.177734375, 2.51953125, 5.751497745513916, 0.0, 0.0, 0.0, 0.4528774010077814],
    [5.591796875, -3.8046875, 6.753652572631836, 0.0, -0.0, 0.0, -0.5974536255274036],
    [5.626953125, -3.828125, 6.797224521636963, 0.0, -0.0, 0.0, -0.5973949694043945],
    [8.15234375, 5.546875, 9.847261428833008, 0.0, 0.0, 0.0, 0.597451894472695],
    [8.390625, 5.26171875, 9.890832901000977, 0.0, 0.0, 0.0, 0.5601044557147236],
    [8.427734375, 5.28515625, 9.934405326843262, 0.0, 0.0, 0.0, 0.560118570468501],
    [10.771484375, -2.4140625, 11.023703575134277, 0.0, -0.0, 0.0, -0.22047292005649058],
    [9.162109375, -6.234375, 11.067276000976562, 0.0, -0.0, 0.0, -0.5974856500395117],
    [9.828125, 5.21484375, 11.110847473144531, 0.0, 0.0, 0.0, 0.48783009202404],
    [0.140625, 0.548828125, 0.5664353966712952, 0.4580317504308908, 1.7875962407416226, 1.8453437089920044, 1.3199649016291295],
    [0.212890625, 0.572265625, 0.6100073456764221, 0.6434130775323813, 1.7295412732793274, 1.8453437089920044, 1.2146463347383654],
    [0.18359375, 0.71875, 0.740723192691803, 0.4567012581804075, 1.7879366222248485, 1.8453437089920044, 1.320709121995752],
    [0.1953125, 0.76171875, 0.7842951416969299, 0.45833785394305737, 1.7875177805994658, 1.8453437089920044, 1.3197936603710374],
    [3.474609375, 2.564453125, 4.313623428344727, 1.484744161218942, 1.0958230605543158, 1.8453437089920044, 0.6358124662883599],
    [3.509765625, 2.58984375, 4.3571953773498535, 1.4848567288638905, 1.095670524867761, 1.8453437089920044, 0.63570973484931],
    [8.271484375, 1.578125, 8.40938663482666, 1.8126473115722617, 0.3458368519493254, 1.8453437089920044, 0.18852530672476378],
    [8.314453125, 1.587890625, 8.452959060668945, 1.8125843702100641, 0.3461665858897313, 1.8453437089920044, 0.188707217271973],
    [8.513671875, 1.908203125, 8.714390754699707, 1.800668676290983, 0.40359103377162237, 1.8453437089920044, 0.2204899975769807],
    [9.255859375, 1.767578125, 9.411541938781738, 1.8125880461736725, 0.3461473373935961, 1.8453437089920044, 0.1886965979171263]]
)
