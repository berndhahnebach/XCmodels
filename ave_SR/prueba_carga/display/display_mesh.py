# -*- coding: utf-8 -*-

execfile('../model_data.py')
execfile('../captionTexts.py')
from postprocess.xcVtk.FE_model import vtk_FE_graphic


defDisplay= vtk_FE_graphic.RecordDefDisplayEF()

setToDisp=surf_eje2_set
defDisplay.FEmeshGraphic(xcSets=[setToDisp],caption='',viewNm='XYZPos',defFScale=1.0)

