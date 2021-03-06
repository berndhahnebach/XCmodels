# -*- coding: utf-8 -*-
import os
from postprocess import limit_state_data as lsd
from postprocess import element_section_map
from postprocess.xcVtk.FE_model import vtk_FE_graphic
from miscUtils import LogMessages as lmsg


model_path="../"
#Project directory structure
execfile(model_path+'project_directories.py')
lsd.LimitStateData.internal_forces_results_directory= '../'+internal_forces_results_directory

modelDataInputFile=model_path+"model_data.py" #data for FE model generation
execfile(modelDataInputFile)

lc=gm.QuickGraphics(model)
#solve for load case
#lc.solve(loadCaseName='GselfWeight',loadCaseExpr='1.0*GselfWeight') #solve for load case
#lc.solve(loadCaseName='GearthPress',loadCaseExpr='1.0*GearthPress') #solve for load case
#lc.solve(loadCaseName='QtrafSit1a',loadCaseExpr='1.0*QtrafSit1a')
#lc.solve(loadCaseName='QtrafSit1b',loadCaseExpr='1.0*QtrafSit1b')
#lc.solve(loadCaseName='QtrafSit2a',loadCaseExpr='1.0*QtrafSit2a')
# lc.solve(loadCaseName='QtrafSit2b',loadCaseExpr='1.0*QtrafSit2b')

#display displacemets or rotations (global axes) 
#available components: 'uX', 'uY', 'uZ', 'rotX', rotY', 'rotZ'
lc.displayDispRot(itemToDisp='uX',setToDisplay=foundDeckSet,fConvUnits=1e3,unitDescription= '(mm)')
lc.displayDispRot(itemToDisp='uY',setToDisplay=foundDeckSet,fConvUnits=1e3,unitDescription= '(mm)') 
lc.displayDispRot(itemToDisp='uZ',setToDisplay=foundDeckSet,fConvUnits=1e3,unitDescription= '(mm)')

#display internal forces, available components: 'N1', 'N2', 'M1', 'M2', 'Q1', 'Q2'
lc.displayIntForc(itemToDisp='N1',setToDisplay=foundDeckSet,fConvUnits= 1e-3,unitDescription= '[kN/m]')
lc.displayIntForc(itemToDisp='N2',setToDisplay=foundDeckSet,fConvUnits= 1e-3,unitDescription= '[kN/m]')
lc.displayIntForc(itemToDisp='M1',setToDisplay=foundDeckSet,fConvUnits= 1e-3,unitDescription= '[kN m/m]')
lc.displayIntForc(itemToDisp='M2',setToDisplay=foundDeckSet,fConvUnits= 1e-3,unitDescription= '[kN m/m]')
lc.displayIntForc(itemToDisp='Q1',setToDisplay=foundDeckSet,fConvUnits= 1e-3,unitDescription= '[kN/m]')
lc.displayIntForc(itemToDisp='Q2',setToDisplay=foundDeckSet,fConvUnits= 1e-3,unitDescription= '[kN/m]')

#display ground pressures
defDisplay= vtk_FE_graphic.RecordDefDisplayEF()
foundationElasticSupports.displayPressures(defDisplay,'Ground pressures',fUnitConv= 1e-6,unitDescription= '[MPa]')

#display displacemets or rotations (IN LOCAL AXIS) 
#available components: 'uX', 'uY', 'uZ', 'rotX', rotY', 'rotZ'
lc.displayDispRot(itemToDisp='uX',setToDisplay=wallsSet,fConvUnits=1e3,unitDescription= '(mm)')
lc.displayDispRot(itemToDisp='uY',setToDisplay=wallsSet,fConvUnits=1e3,unitDescription= '(mm)') 
lc.displayDispRot(itemToDisp='uZ',setToDisplay=wallsSet,fConvUnits=1e3,unitDescription= '(mm)')

#display internal forces, available components: 'N1', 'N2', 'M1', 'M2', 'Q1', 'Q2'
lc.displayIntForc(itemToDisp='N1',setToDisplay=wallsSet,fConvUnits= 1e-3,unitDescription= '[kN/m]')
lc.displayIntForc(itemToDisp='N2',setToDisplay=wallsSet,fConvUnits= 1e-3,unitDescription= '[kN/m]')
lc.displayIntForc(itemToDisp='M1',setToDisplay=wallsSet,fConvUnits= 1e-3,unitDescription= '[kN m/m]')
lc.displayIntForc(itemToDisp='M2',setToDisplay=wallsSet,fConvUnits= 1e-3,unitDescription= '[kN m/m]')
lc.displayIntForc(itemToDisp='Q1',setToDisplay=wallsSet,fConvUnits= 1e-3,unitDescription= '[kN/m]')
lc.displayIntForc(itemToDisp='Q2',setToDisplay=wallsSet,fConvUnits= 1e-3,unitDescription= '[kN/m]')
