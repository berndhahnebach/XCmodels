# -*- coding: utf-8 -*-
from postprocess.control_vars import *
from postprocess import limit_state_data as lsd
from postprocess.xcVtk.FE_model import vtk_display_limit_state as dls

#FE model generation
execfile("../model_gen.py")

#Load properties to display:
execfile(cfg.verifCrackFreqFile)


#  Config
argument= 'wk'    #Possible arguments: 'N', 'My','Mz','s_rmax','eps_sm','wk'
fUnitConv=1e3     #unit conversion factor (i.e m->mm => fUnitConv= 1e3)
setDisp= wall     #Set of shell elements to be displayed
viewName='XYZPos' #predefined view names: 'XYZPos','XNeg','XPos','YNeg',
                  #'YPos','ZNeg','ZPos'  (defaults to 'XYZPos')
rgMinMax=(0,0.3)  #truncate values to be included in the range
                  #(if None -> don't truncate)
#  End config 

dls.displayFieldDirs1and2(limitStateLabel=lsd.freqLoadsCrackControl.label,argument=argument,elementSet=setDisp,component=None,fUnitConv=fUnitConv,fileName=None,captionTexts=cfg.capTexts,defFScale=0.0,viewName=viewName,rgMinMax=rgMinMax) 




