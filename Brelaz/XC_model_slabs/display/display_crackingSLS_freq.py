# -*- coding: utf-8 -*-
from postprocess.control_vars import *
from postprocess import limit_state_data as lsd
from postprocess.xcVtk.FE_model import vtk_display_limit_state as dls


model_path="../"
modelDataInputFile=model_path+"model_data.py" #data for FE model generation
execfile(modelDataInputFile)


#Load properties to display:
preprocessor= FEcase.getPreprocessor
execfile(projectDirs.getCrackingSLSFreqFileName())
execfile('../captionTexts.py')


limitStateLabel= lsd.freqLoadsCrackControl.label
#Possible arguments: 'getMaxSteelStress', 'getCF'
argument= 'getMaxSteelStress'

setDisp= shells
dls.displayFieldDirs1and2(limitStateLabel,argument,setDisp,None,1.0,None,capTexts,defFScale=0.0)


