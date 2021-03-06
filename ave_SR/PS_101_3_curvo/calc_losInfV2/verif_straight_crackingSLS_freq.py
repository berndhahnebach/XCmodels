# -*- coding: utf-8 -*-

import os

#Project directory structure
execfile('./directs.py')

from postprocess import limit_state_data as lsd
from postprocess import RC_material_distribution
#from materials.ehe import EHE_limit_state_checking
from materials.ec2 import EC2_limit_state_checking
from solution import predefined_solutions

lsd.LimitStateData.internal_forces_results_directory= dir_int_forces
lsd.LimitStateData.check_results_directory=  dir_checks

#Reinforced concrete sections on each element.
#reinfConcreteSections=RC_material_distribution.RCMaterialDistribution()
reinfConcreteSections.mapSectionsFileName='./mapSectionsReinforcementTenStiff.pkl'
reinfConcreteSections=RC_material_distribution.loadRCMaterialDistribution()
#Checking material for limit state.
limitStateLabel= lsd.freqLoadsCrackControl.label
lsd.freqLoadsCrackControl.controller= EC2_limit_state_checking.CrackStraightController(limitStateLabel= lsd.freqLoadsCrackControl.label)
lsd.freqLoadsCrackControl.controller.analysisToPerform=predefined_solutions.simple_static_modified_newton
meanFCs= lsd.freqLoadsCrackControl.check(reinfConcreteSections)



