# -*- coding: utf-8 -*-
"""
Created on Thu Dec  5 16:11:17 2019

@author: Administrator
"""

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
#input
dis_l = ctrl.Antecedent(np.arange(0, 2.01, 0.01), 'dis_l')
dis_lf = ctrl.Antecedent(np.arange(0, 2.01, 0.01), 'dis_lf')
dis_f = ctrl.Antecedent(np.arange(0, 2.01, 0.01), 'dis_f')
dis_rf = ctrl.Antecedent(np.arange(0, 2.01, 0.01), 'dis_rf')
dis_r = ctrl.Antecedent(np.arange(0, 2.01, 0.01), 'dis_r')
goal_position = ctrl.Antecedent(np.arange(-150, 150.01, 0.01), 'goal_position')
#output
steer_angle = ctrl.Consequent(np.arange(50, 130.01, 0.01), 'steer_angle')
speed = ctrl.Consequent(np.arange(0, 2.01, 0.01), 'speed')

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
#goal_position parameter  //chang the interval because of the calculation of angle
goal_position['left'] = fuzz.trapmf(goal_position.universe, [10, 60, 150, 180])
goal_position['front'] = fuzz.trimf(goal_position.universe, [-20, 0, 20])
goal_position['right'] = fuzz.trapmf(goal_position.universe, [-180, -150, -60, -10])
#steer_angle parameter
steer_angle['turn_l'] = fuzz.trimf(steer_angle.universe, [0, 50, 80])
steer_angle['turn_lsmall'] = fuzz.trimf(steer_angle.universe, [55, 75, 90])
steer_angle['forward'] = fuzz.trimf(steer_angle.universe, [70, 90, 110])
steer_angle['turn_rsmall'] = fuzz.trimf(steer_angle.universe, [90, 105, 125])
steer_angle['turn_r'] = fuzz.trimf(steer_angle.universe, [100, 130, 180])
#speed parameter
speed['stop'] = fuzz.trimf(speed.universe, [-1, 0, 0])
speed['slow'] = fuzz.trimf(speed.universe, [0.2, 0.2, 1.2])
speed['quick'] = fuzz.trimf(speed.universe, [0.8, 1.4, 1.4])

# =============================================================================
# dis_l.view()
# dis_lf.view()
# dis_f.view()
# dis_rf.view()
# dis_r.view()
# goal_position.view()
# steer_angle.view()
# speed.view()
# =============================================================================

#rulelist
#rule = ctrl.Rule(dis_l['close'] & dis_lf['close'] & dis_f['close'] & dis_rf['close'] & dis_r['close'] & goal_position['front'], (steer_angle['forward'], speed['stop']))
rule1 = ctrl.Rule(dis_l['close'] & dis_f['close'] & dis_r['close'], (steer_angle['forward'], speed['stop']))

rule2 = ctrl.Rule(~dis_lf['close'] & dis_f['far'] & ~dis_rf['close'] & goal_position['front'], (steer_angle['forward'], speed['quick']))
rule3 = ctrl.Rule(dis_lf['close'] & dis_f['far'] & ~dis_rf['close'] & goal_position['front'], (steer_angle['turn_rsmall'], speed['slow']))
rule4 = ctrl.Rule(~dis_lf['close'] & dis_f['far'] & dis_rf['close'] & goal_position['front'], (steer_angle['turn_lsmall'], speed['slow']))
rule5 = ctrl.Rule(dis_lf['close'] & dis_f['far'] & dis_rf['close'] & goal_position['front'], (steer_angle['forward'], speed['slow']))

rule6 = ctrl.Rule(dis_l['far'] & ~dis_f['far'] & goal_position['front'], (steer_angle['turn_l'], speed['slow']))
rule7 = ctrl.Rule(~dis_l['far'] & ~dis_f['far'] & dis_r['far'] & goal_position['front'], (steer_angle['turn_r'], speed['slow']))
rule8 = ctrl.Rule(dis_l['near'] & ~dis_f['far'] & ~dis_r['far'] & goal_position['front'], (steer_angle['turn_lsmall'], speed['slow']))
rule9 = ctrl.Rule(dis_l['close'] & ~dis_f['far'] & dis_r['near'] & goal_position['front'], (steer_angle['turn_rsmall'], speed['slow']))
rule10 = ctrl.Rule(dis_l['close'] & ~dis_f['far'] & dis_r['close'] & goal_position['front'], (steer_angle['forward'], speed['stop']))

rule11 = ctrl.Rule(dis_l['far'] & goal_position['left'], (steer_angle['turn_l'], speed['slow']))
rule12 = ctrl.Rule(~dis_l['far'] & dis_f['far'] & goal_position['left'], (steer_angle['forward'], speed['quick']))
rule13 = ctrl.Rule(~dis_l['far'] & ~dis_f['far'] & dis_r['far'] & goal_position['left'], (steer_angle['turn_r'], speed['slow']))
rule14 = ctrl.Rule(dis_l['close'] & ~dis_f['close'] & ~dis_r['close'] & goal_position['left'], (steer_angle['turn_r'], speed['slow']))

rule15 = ctrl.Rule(dis_r['far'] & goal_position['right'], (steer_angle['turn_r'], speed['slow']))
rule16 = ctrl.Rule(dis_f['far'] & ~dis_r['far'] & goal_position['right'], (steer_angle['forward'], speed['quick']))
rule17 = ctrl.Rule(dis_l['far'] & ~dis_f['far'] & ~dis_r['far'] & goal_position['right'], (steer_angle['turn_l'], speed['slow']))
rule18 = ctrl.Rule(~dis_l['close'] & ~dis_f['close'] & dis_r['close'] & goal_position['right'], (steer_angle['turn_l'], speed['slow']))

rule19 = ctrl.Rule(dis_lf['close'] & dis_rf['near'], (steer_angle['turn_r'], speed['slow']))
rule20 = ctrl.Rule(dis_lf['close'] & dis_rf['far'], (steer_angle['turn_r'], speed['slow']))
rule21 = ctrl.Rule(dis_lf['near'] & dis_rf['far'], (steer_angle['turn_rsmall'], speed['slow']))
rule22 = ctrl.Rule(dis_lf['near'] & dis_rf['close'], (steer_angle['turn_l'], speed['slow']))
rule23 = ctrl.Rule(dis_lf['far'] & dis_rf['close'], (steer_angle['turn_l'], speed['slow']))
rule24 = ctrl.Rule(dis_lf['far'] & dis_rf['near'], (steer_angle['turn_lsmall'], speed['slow']))

# =============================================================================
# rule1.view()
# rule2.view()
# =============================================================================

car_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18, rule19, rule20, rule21, rule22, rule23, rule24])
car = ctrl.ControlSystemSimulation(car_ctrl)

car.input['dis_l'] = 1.21
car.input['dis_lf'] = 1.35
car.input['dis_f'] = 0.675
car.input['dis_rf'] = 1.38
car.input['dis_r'] = 1.38
car.input['goal_position']=51.3

# Crunch the numbers
car.compute()
print(car.output['steer_angle'])
print(car.output['speed'])
# =============================================================================
# steer_angle.view(sim=car)
# speed.view(sim=car)
# =============================================================================
