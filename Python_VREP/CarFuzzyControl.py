# -*- coding: utf-8 -*-
"""
Created on Thu Dec  5 16:11:17 2019

@author: Administrator
"""

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

dis_l = ctrl.Antecedent(np.arange(0, 2.01, 0.01), 'dis_l')
dis_lf = ctrl.Antecedent(np.arange(0, 2.01, 0.01), 'dis_lf')
dis_f = ctrl.Antecedent(np.arange(0, 2.01, 0.01), 'dis_f')
dis_rf = ctrl.Antecedent(np.arange(0, 2.01, 0.01), 'dis_rf')
dis_r = ctrl.Antecedent(np.arange(0, 2.01, 0.01), 'dis_r')

#dis_l parameter
dis_l['close'] = fuzz.trapmf(dis_l.universe, [-1, 0, 0.3, 0.6])
dis_l['near'] = fuzz.trimf(dis_l.universe, [0.5, 0.5, 1.4])
dis_l['far'] = fuzz.trapmf(dis_l.universe, [1, 1.8, 2, 3])
#dis_lf parameter
dis_lf['close'] = fuzz.trapmf(dis_lf.universe, [-1, 0, 1, 1.4])
dis_lf['near'] = fuzz.trimf(dis_lf.universe, [1.2, 1.5, 1.8])
dis_lf['far'] = fuzz.trimf(dis_lf.universe, [1.8, 2, 2])
#dis_f parameter
dis_f['close'] = fuzz.trapmf(dis_f.universe, [-1, 0, 1, 1.4])
dis_f['near'] = fuzz.trimf(dis_f.universe, [1.2, 1.5, 1.8])
dis_f['far'] = fuzz.trapmf(dis_f.universe, [1.6, 1.8, 2, 3])
#dis_rf parameter
dis_rf['close'] = fuzz.trapmf(dis_rf.universe, [-1, 0, 1, 1.4])
dis_rf['near'] = fuzz.trimf(dis_rf.universe, [1.2, 1.5, 1.8])
dis_rf['far'] = fuzz.trimf(dis_rf.universe, [1.8, 2, 2])
#dis_r parameter
dis_r['close'] = fuzz.trapmf(dis_r.universe, [-1, 0, 0.3, 0.6])
dis_r['near'] = fuzz.trimf(dis_r.universe, [0.5, 0.5, 1.4])
dis_r['far'] = fuzz.trapmf(dis_r.universe, [1, 1.8, 2, 3])

dis_l.view()
dis_lf.view()
dis_f.view()
dis_rf.view()
dis_r.view()