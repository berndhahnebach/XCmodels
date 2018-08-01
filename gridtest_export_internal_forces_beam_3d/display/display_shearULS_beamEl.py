# -*- coding: utf-8 -*-
from postprocess.control_vars import *
from postprocess.xcVtk.FE_model import vtk_FE_graphic
from postprocess.xcVtk import control_var_diagram as cvd

model_path="../"
#Project directory structure
execfile(model_path+'project_directories.py')

modelDataInputFile=model_path+"model_data.py" #data for FE model generation
execfile(modelDataInputFile)


#Load properties to display:
preprocessor= model.getPreprocessor()
fName= model_path+check_results_directory+'verifRsl_shearULS.py'
execfile(fName)
execfile('../captionTexts.py')


limitStateLabel= lsd.shearResistance.label


#Possible arguments: 'CF', 'N', 'My', 'Mz', 'Mu', 'Vy', 'Vz', 'theta', 'Vcu', 'Vsu', 'Vu'
argument= 'CF'
setDisp=colsSet
#setDisp=botColSet

diagram= cvd.ControlVarDiagram(scaleFactor= 10,fUnitConv= 1,sets=[setDisp.elSet],attributeName= limitStateLabel,component= argument)
diagram.addDiagram()


defDisplay= vtk_FE_graphic.RecordDefDisplayEF()
defDisplay.viewName= "YPos" #Point of view.
defDisplay.setupGrid(preprocessor.getSets.getSet('total'))
defDisplay.defineMeshScene(None)
defDisplay.appendDiagram(diagram) #Append diagram to the scene.

#execfile('draw_supports.py')
#defDisplay.renderer.AddActor(supportsActor)

caption= capTexts[limitStateLabel] + ', ' + capTexts[argument] + '. '+ setDisp.genDescr.capitalize() + ', ' + setDisp.sectDescr[1]
defDisplay.displayScene(caption)


