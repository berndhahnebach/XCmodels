# -*- coding: utf-8 -*-
#from postprocess.reports import graphical_reports

execfile("../model_gen.py") #FE model generation
execfile('../loadStateData.py')


#ordered list of load cases (from those defined in ../loadStateData.py
#or redefined lately) to be displayed:
loadCasesToDisplay=[G1]

textfl=open(cfg.reportSimplLCFile,'w')  #tex file to be generated
for lc in loadCasesToDisplay:
    lc.simplLCReports(FEproblem=FEcase,pathGr=cfg.reportSimplLCGrPath,texFile=textfl,grWdt=cfg.grWidth,capStdTexts=cfg.capTexts)

textfl.close()

