# -*- coding: utf-8 -*-

from postprocess.xcVtk import vtk_grafico_base
from postprocess.xcVtk.FE_model import vtk_FE_graphic
from postprocess.xcVtk import linear_load_diagram as lld
import vtk
execfile('../model_data.py')
execfile('../loadStateData.py')

loadCasesToDisplay=[G1,Q1]

#End data

for lc in loadCasesToDisplay:
    for st in lc.setsToDispBeamLoads:
        lcs=gm.QuickGraphics(model)
        capt=lc.loadCaseDescr + ', ' + st.genDescr + ', '  + lc.unitsLoads
        lcs.dispLoadCaseBeamEl(loadCaseName=lc.loadCaseName,setToDisplay=st.elSet,fUnitConv=lc.unitsScaleLoads,elLoadComp=lc.compElLoad,elLoadScaleF=lc.vectorScaleLoads,nodLoadScaleF=lc.vectorScalePointLoads,viewName=lc.viewName,hCamFc=lc.hCamFc,caption= capt,fileName=None)

