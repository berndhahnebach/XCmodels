# -*- coding: utf-8 -*-

execfile('../model_data.py')
execfile('../captionTexts.py')

from postprocess.xcVtk.FE_model import quick_graphics as qg

    # displayLocalAxes: vector field display of the element local axes.
    # Parameters:
    #   setToDisplay:   set of elements to be displayed
    #                   (defaults to 'total')
    #   vectorScale:    factor to apply to the vectors length in the 
    #                   representation (defaults to 1).
    #   viewNm:         name of the view  that contains the renderer (possible
    #                   options: "XYZPos","XYZNeg", "XPos", "XNeg","YPos","YNeg"
    #                   "ZPos", "ZNeg") (defaults to "XYZPos")
    #   hCamFct:   factor that applies to the height of the camera position 
    #              in order to change perspective of isometric views 
    #              (defaults to 1, usual values 0.1 to 10)
    #   fileName:       full name of the graphic file to generate. Defaults to 
    #                   None, in this case it returns a console output graphic.

#qg.displayLocalAxes(preprocessor=prep,setToDisplay=shells,vectorScale=0.15,viewNm="XYZPos",hCamFct=2.0,caption= capTexts['LocalAxes'],fileName=None,defFScale=0.0)
qg.displayLocalAxes(prep=prep,setToDisplay=deck,vectorScale=1,viewNm="XYZPos",hCamFct=1.0,caption=capTexts['LocalAxes'],fileName=None,defFScale=0.0)


