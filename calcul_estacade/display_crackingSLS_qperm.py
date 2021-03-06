# -*- coding: utf-8 -*-

from postprocess.control_vars import *
from postprocess import limit_state_data as lsd
from postprocess.xcVtk import vtk_grafico_base
from postprocess.xcVtk.FE_model import vtk_FE_graphic
from postprocess.xcVtk.FE_model import Fields


model_path="./"
#Project directory structure
execfile(model_path+'project_directories.py')

modelDataInputFile=model_path+"model_data.py" #data for FE model generation
execfile(modelDataInputFile)


#Load properties to display:
fName= model_path+check_results_directory+'verifRsl_crackingSLS_qperm.py'
execfile(fName)
execfile('./captionTexts.py')

limitStateLabel= lsd.quasiPermanentLoadsCrackControl.label
defDisplay= vtk_FE_graphic.RecordDefDisplayEF()

argument= 'getMaxSteelStress'
#argument= 'crackControlVarsNeg.steelStress'


sets= [setDeck,setDock,setParapet]

for setDisp in sets:
  attributeName= limitStateLabel + 'Sect1'
  field= Fields.getScalarFieldFromControlVar(attributeName,argument,setDisp,None,1.0)
  field.plot(defDisplay,caption= capTexts[limitStateLabel] + ', ' + capTexts[argument] + '. '+ setDisp.genDescr.capitalize() + ', ' + setDisp.sectDescr[0] )
  attributeName= limitStateLabel + 'Sect2'
  field= Fields.getScalarFieldFromControlVar(attributeName,argument,setDisp,None,1.0)
  field.plot(defDisplay,caption= capTexts[limitStateLabel] + ', ' + capTexts[argument] + '. '+ setDisp.genDescr.capitalize() + ', ' + setDisp.sectDescr[1] )

