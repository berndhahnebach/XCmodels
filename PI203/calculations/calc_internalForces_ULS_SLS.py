# -*- coding: utf-8 -*-
import os
from postprocess import limit_state_data as lsd

model_path="../"
#Project directory structure
execfile(model_path+'project_directories.py')
lsd.LimitStateData.internal_forces_results_directory= '../'+internal_forces_results_directory

modelDataInputFile=model_path+"model_data.py" #data for FE model generation
execfile(modelDataInputFile)

#RC sections definition.
execfile(model_path+'sectionsDef.py')

#Define section for each element (spatial distribution of RC sections).
preprocessor= prep
reinfConcreteSectionDistribution.assign(elemSet=preprocessor.getSets.getSet('foundExtSlab').getElements,setRCSects=FoundExtSlabRCSect)
reinfConcreteSectionDistribution.assign(elemSet=preprocessor.getSets.getSet('foundIntSlab').getElements,setRCSects=FoundIntSlabRCSect)
reinfConcreteSectionDistribution.assign(elemSet=preprocessor.getSets.getSet('leftUpWall').getElements,setRCSects=LeftUpWallRCSect)
reinfConcreteSectionDistribution.assign(elemSet=preprocessor.getSets.getSet('leftDownWall').getElements,setRCSects=LeftDownWallRCSect)
reinfConcreteSectionDistribution.assign(elemSet=preprocessor.getSets.getSet('rightUpWall').getElements,setRCSects=RightUpWallRCSect)
reinfConcreteSectionDistribution.assign(elemSet=preprocessor.getSets.getSet('rightDownWall').getElements,setRCSects=RightDownWallRCSect)
reinfConcreteSectionDistribution.assign(elemSet=preprocessor.getSets.getSet('midWall').getElements,setRCSects=MidWallRCSect)
reinfConcreteSectionDistribution.assign(elemSet=preprocessor.getSets.getSet('upDeckExtSlab').getElements,setRCSects=UpDeckExtSlabRCSect)
reinfConcreteSectionDistribution.assign(elemSet=preprocessor.getSets.getSet('upDeckIntSlab').getElements,setRCSects=UpDeckIntSlabRCSect)
reinfConcreteSectionDistribution.assign(elemSet=preprocessor.getSets.getSet('downDeckExtSlab').getElements,setRCSects=DownDeckExtSlabRCSect)
reinfConcreteSectionDistribution.assign(elemSet=preprocessor.getSets.getSet('downDeckIntSlab').getElements,setRCSects=DownDeckIntSlabRCSect)

reinfConcreteSectionDistribution.dump()

#Set of entities for which analysis is going to be performed.
setCalc= shellElements

loadCombinations= preprocessor.getLoadHandler.getLoadCombinations

#Limit states to calculate internal forces for.
limitStates= [lsd.normalStressesResistance, # Normal stresses resistance.
lsd.shearResistance, # Shear stresses resistance (IS THE SAME AS NORMAL STRESSES, THIS IS WHY IT'S COMMENTED OUT).
lsd.freqLoadsCrackControl, # RC crack control under frequent loads
lsd.quasiPermanentLoadsCrackControl, # RC crack control under quasi-permanent loads
lsd.fatigueResistance # Fatigue resistance.
] 

for ls in limitStates:
  ls.saveAll(feProblem=FEcase,combContainer=combContainer,setCalc=setCalc,fConvIntForc= 1.0)
  print 'combinations for ', ls.label, ': ', loadCombinations.getKeys()

