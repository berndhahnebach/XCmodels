# -*- coding: utf-8 -*-
from postprocess.control_vars import *
from postprocess import limit_state_data as lsd
from postprocess.xcVtk.FE_model import vtk_FE_graphic
from postprocess.xcVtk import control_var_diagram as cvd

model_path="../"
#Project directory structure
execfile(model_path+'project_directories.py')

modelDataInputFile=model_path+"model_data.py" #data for FE model generation
execfile(modelDataInputFile)


#Load properties to display:
fName= model_path+check_results_directory+'verifRsl_crackingSLS_qperm.py'
execfile(fName)


limitStateLabel= lsd.quasiPermanentLoadsCrackControl.label

#Possible arguments: 'getCF', 'getMaxSteelStress'
argument= 'getMaxSteelStress'


setDispRes=beamX   #set of linear elements to which display results 

setDisp=overallSet    #set of elements (any type) to be displayed

diagram= cvd.ControlVarDiagram(scaleFactor= 0.1,fUnitConv= 1000,sets=[setDispRes],attributeName= limitStateLabel,component= argument)
diagram.addDiagram()

defDisplay= vtk_FE_graphic.RecordDefDisplayEF()
 #predefined view names: 'XYZPos','XNeg','XPos','YNeg','YPos',
 #                        'ZNeg','ZPos'  (defaults to 'XYZPos')
#defDisplay.viewName= "YPos" #Point of view.
defDisplay.setupGrid(setDisp)
defDisplay.defineMeshScene(None,defFScale=0.0)
defDisplay.appendDiagram(diagram) #Append diagram to the scene.

caption= cfg.capTexts[limitStateLabel] + ', ' + cfg.capTexts[argument] + '. '+ setDispRes.description.capitalize() + ', ' + 'Dir. 1'
defDisplay.displayScene(caption)



