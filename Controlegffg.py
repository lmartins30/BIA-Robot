#!/usr/bin/python

import numpy as np
import skfuzzy as fuzz
import time
import rospy
import math
import time
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from skfuzzy import control as ctrl

#duvidas?: https://github.com/scikit-fuzzy/scikit-fuzzy

intm_left = 0
intm_right = 0
S_Esq = 0
S_Fre = 0
S_Dir = 0

def callback(msg):
	global intm_left, intm_right
	V_Esq = msg.data[0]
	V_Dir = msg.data[1]
	S_Esq = msg.data[2]
	S_Fre = msg.data[3]
	S_Dir = msg.data[4]
	global S_Dir
	global S_Esq
	global S_Fre
	if V_Esq != 0:
		V_Esquerda =20/V_Esq
   	else:
		V_Esquerda = 0
	if V_Dir != 0:
		V_Direita = 20/V_Dir
	else:
		V_Direita = 0
	robot.input['Sesq'] = S_Esq/100.
	robot.input['Scen'] = S_Fre/100.
	robot.input['Sdir'] = S_Dir/100.
	robot.compute()
	intm_left = robot.output['Mesq']
	intm_right = robot.output['Mdir']


# New Antecedent/Consequent objects hold universe variables and membership
# functions
Sesq = ctrl.Antecedent(np.arange(0.01, 4.0, 0.01),'Sesq')
Scen = ctrl.Antecedent(np.arange(0.01, 4.0, 0.01),'Scen')
Sdir = ctrl.Antecedent(np.arange(0.01, 4.0, 0.01),'Sdir')

Mesq = ctrl.Consequent(np.arange(-8, 8, .1), 'Mesq')
Mdir = ctrl.Consequent(np.arange(-8, 8, .1), 'Mdir')

# Custom membership functions can be built interactively with a familiar,
# Pythonic API
Sesq['muito perto'] = fuzz.trapmf(Sesq.universe, [0.01,0.01, 0.3, 0.6])
Sesq['perto'] = fuzz.trimf(Sesq.universe, [0.3, 0.6, 1])
Sesq['longe'] = fuzz.trimf(Sesq.universe, [0.6, 1, 1.5])
Sesq['muito longe'] = fuzz.trapmf(Sesq.universe, [1, 1.5, 4,4])
Sdir['muito perto'] = fuzz.trapmf(Sdir.universe, [0.01,0.01, 0.3, 0.6])
Sdir['perto'] = fuzz.trimf(Sdir.universe, [0.3, 0.6, 1])
Sdir['longe'] = fuzz.trimf(Sdir.universe, [0.6, 1, 1.5])
Sdir['muito longe'] = fuzz.trapmf(Sdir.universe, [1, 1.5, 4,4])
Scen['muito perto'] = fuzz.trapmf(Scen.universe, [0.01,0.1, 0.3, 0.6])
Scen['perto'] = fuzz.trimf(Scen.universe, [0.3, 0.6, 1])
Scen['longe'] = fuzz.trimf(Scen.universe, [0.6, 1, 1.5])
Scen['muito longe'] = fuzz.trapmf(Scen.universe, [1, 1.5, 4,4])

Mesq['positivo alto'] = fuzz.trapmf(Mesq.universe, [2.66, 4.66, 6.66,6.66])
Mesq['positivo medio'] = fuzz.trimf(Mesq.universe, [0, 2.66, 4.66])
Mesq['zero'] = fuzz.trimf(Mesq.universe, [-2.66, 0, 2.66])
Mesq['negativo medio'] = fuzz.trimf(Mesq.universe, [-4.66, -2.66, 0])
Mesq['negativo alto'] = fuzz.trapmf(Mesq.universe, [-6.66,-6.66, -4.66, -2.66])
Mdir['positivo alto'] = fuzz.trapmf(Mdir.universe, [2.66, 4.66, 6.66, 6.66])
Mdir['positivo medio'] = fuzz.trimf(Mdir.universe, [0, 2.66, 4.66])
Mdir['zero'] = fuzz.trimf(Mdir.universe, [-2.66, 0, 2.66])
Mdir['negativo medio'] = fuzz.trimf(Mdir.universe, [-4.66, -2.66, 0])
Mdir['negativo alto'] = fuzz.trapmf(Mdir.universe, [-6.66, -6.66, -4.66, -2.66])

 
rule1 = ctrl.Rule(Sesq['muito perto'] & Sdir['muito perto'], (Mdir['zero'], Mesq['zero']))

rule2 = ctrl.Rule(Sesq['muito perto'] & Sdir['perto'], (Mdir['negativo medio'], Mesq['positivo medio']))

rule3 = ctrl.Rule(Sesq['perto'] & Sdir['muito perto'], (Mdir['positivo medio'], Mesq['negativo medio']))

#regra 2 e 3 sao o grupo 1 da tabela de regras.

rule4 = ctrl.Rule(Sesq['muito perto'] & Sdir['longe'] & Scen['perto'], (Mdir['zero'], Mesq['positivo medio']))

rule5 = ctrl.Rule(Sesq['muito perto'] & Sdir['longe'] & Scen['muito perto'], (Mdir['negativo medio'], Mesq['positivo alto']))

rule6 = ctrl.Rule(Sesq['muito perto'] & Sdir['longe'], (Mdir['positivo medio'], Mesq['positivo alto']))

rule7 = ctrl.Rule(Sesq['longe'] & Sdir['muito perto'] & Scen['perto'], (Mdir['positivo medio'], Mesq['zero']))

rule8 = ctrl.Rule(Sesq['longe'] & Sdir['muito perto'] & Scen['muito perto'], (Mdir['positivo alto'], Mesq['negativo medio']))

rule9 = ctrl.Rule(Sesq['longe'] & Sdir['muito perto'], (Mdir['positivo alto'], Mesq['positivo medio']))

# regra 4,5,6,7,8 e 9 sao o grupo 2 da tabela de regras.

rule10 = ctrl.Rule(Sesq['muito perto'] & Sdir['muito longe'], (Mdir['positivo medio'], Mesq['positivo alto']))

rule11 = ctrl.Rule(Sesq['muito longe'] & Sdir['muito perto'], (Mdir['positivo alto'], Mesq['positivo medio']))

#regra 10 e 11 sao o grupo 3 da tabela de regras.

rule12 = ctrl.Rule(Sesq['perto'] & Sdir['perto'] & Scen['perto'], (Mdir['zero'], Mesq['zero']))

rule13 = ctrl.Rule(Sesq['perto'] & Sdir['perto'] & Scen['longe'], (Mdir['positivo medio'], Mesq['positivo medio']))

rule14 = ctrl.Rule(Sesq['perto'] & Sdir['longe'] & Scen['perto'], (Mdir['zero'], Mesq['positivo medio']))

rule15 = ctrl.Rule(Sesq['perto'] & Sdir['longe'] & Scen['longe'], (Mdir['positivo medio'], Mesq['positivo alto']))

rule16 = ctrl.Rule(Sesq['longe'] & Sdir['perto'] & Scen['perto'], (Mdir['positivo medio'], Mesq['zero']))

rule17 = ctrl.Rule(Sesq['longe'] & Sdir['perto'] & Scen['longe'], (Mdir['positivo alto'], Mesq['positivo medio']))

#regra 14,15,16 e 17 sao o grupo 4 da tabela de regras.

rule18 = ctrl.Rule(Sesq['perto'] & Sdir['muito longe'] & Scen['perto'], (Mdir['zero'], Mesq['positivo medio']))

rule19 = ctrl.Rule(Sesq['perto'] & Sdir['muito longe'] & Scen['longe'], (Mdir['positivo medio'], Mesq['positivo alto']))

rule20 = ctrl.Rule(Sesq['muito longe'] & Sdir['perto'] & Scen['perto'], (Mdir['positivo medio'], Mesq['zero']))

rule21 = ctrl.Rule(Sesq['muito longe'] & Sdir['perto'] & Scen['longe'], (Mdir['positivo alto'], Mesq['positivo medio']))

#regra 18,19,20 e 21 sao o grupo 5 da tabela de regras.

rule22 = ctrl.Rule(Sesq['longe'] & Sdir['longe'], (Mdir['positivo alto'], Mesq['positivo alto']))

rule23 = ctrl.Rule(Sesq['longe'] & Sdir['longe'] & Scen['muito perto'], (Mdir['positivo medio'], Mesq['positivo medio']))

rule24 = ctrl.Rule(Sesq['muito longe'] & Sdir['muito longe'], (Mdir['positivo alto'], Mesq['positivo alto']))

rule25 = ctrl.Rule(Sesq['muito longe'] & Sdir['muito longe'] & Scen['muito perto'], (Mdir['positivo medio'], Mesq['positivo medio']))

rule26 = ctrl.Rule(Sesq['longe'] & Sdir['muito longe'], (Mdir['positivo alto'], Mesq['positivo alto']))

rule27 = ctrl.Rule(Sesq['longe'] & Sdir['muito longe'] & Scen['muito perto'], (Mdir['positivo medio'], Mesq['positivo medio']))

rule28 = ctrl.Rule(Sesq['muito longe'] & Sdir['longe'], (Mdir['positivo alto'], Mesq['positivo alto']))

rule29 = ctrl.Rule(Sesq['muito longe'] & Sdir['longe'] & Scen['muito perto'], (Mdir['positivo medio'], Mesq['positivo medio']))

#regra 26,27,28 e 29 sao o grupo 6 da tabela de regras.

robot_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18, rule19, rule20, rule21, rule22, rule23, rule24, rule25, rule26, rule27, rule28, rule29])
robot = ctrl.ControlSystemSimulation(robot_ctrl)

if __name__=='__main__':

 	outfile  =  open("out.txt", 'a')
	rospy.init_node('chatter')
	rospy.Subscriber('chatter',Int32MultiArray, callback)
 	pub = rospy.Publisher('feedback', Int32MultiArray, queue_size=1)
 	rate = rospy.Rate(10)
      
	feedback = Int32MultiArray()
	feedback.data = [int(0), int(0)]

	while not rospy.is_shutdown():
		feedback.data[0] = intm_right
		feedback.data[1] = intm_left
		print(feedback.data)
		outfile.write('%d ' % S_Esq)
		outfile.write('%d ' % S_Fre)
		outfile.write('%d ' % S_Dir)
		outfile.write(' '.join([str(i) for i in feedback.data]))
		outfile.write('\n')
	        pub.publish(feedback)
        	rate.sleep()

	outfile.close()
