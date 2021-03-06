# -*- coding: utf-8 -*-

execfile('model_data.py')
def resultAction(prb,nmbAction):
  prb.getPreprocessor.resetLoadCase()
  prb.getPreprocessor.getLoadHandler.getLoadPatterns.addToDomain(nmbAction)
  #Solución
  solution= predefined_solutions.SolutionProcedure()
  analysis= solution.simpleStaticLinear(prb)
  result= analysis.analyze(1)
  prb.getPreprocessor.getLoadHandler.getLoadPatterns.removeFromDomain(nmbAction)

from postprocess import get_reactions as gr
 
def getReactions(prb,nmbAction):
  preprocessor= prb.getPreprocessor   
  resultAction(prb,nmbAction)
  reactions= gr.Reactions(preprocessor,supportNodes)
  forces= reactions.getReactionForces()
  for key in forces:
    reac= forces[key]
    print "Appui ", key , nmbAction, reac*1e-3, " (kN)"
    #print "M= ", tmp[2]/1e3
  return reactions

reactions= {}

for lc in loadCaseNames:
  r= getReactions(mainBeam,lc)
  reactions[lc]= r
  R= r.getResultant().reduceTo(pt[10].getPos)
  print 'lc= ', lc, ' resultant: ', R

