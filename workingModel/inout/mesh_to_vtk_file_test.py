# -*- coding: utf-8 -*-

execfile('../model_gen.py') #FE model generation
#execfile('captionTexts.py')
from postprocess.xcVtk.FE_model import vtk_FE_graphic
import vtk


defDisplay= vtk_FE_graphic.RecordDefDisplayEF()
setToDisp= overallSet  # decks  # see model_gen.py


defDisplay.FEmeshGraphic(xcSet= setToDisp,caption='',viewNm="-X+Y+Z",defFScale=1.0)

writer = vtk.vtkXMLUnstructuredGridWriter();
writer.SetFileName("test.vtp");
writer.SetInputData(defDisplay.gridRecord.uGrid)
writer.Write()
