# -*- coding: utf-8 -*-


execfile('model_data.py')
from materials.ec3 import EC3Beam as ec3b

crossSectionClass= 1
ec3beams= list()
for key in lineDict:
  ec3beams.append(ec3b.EC3Beam(lineDict[key],IPE450A))

def resultComb(prb,nmbComb):
  preprocessor.resetLoadCase()
  preprocessor.getLoadHandler.getLoadCombinations.addToDomain(nmbComb)
  #Solución
  solution= predefined_solutions.SolutionProcedure()
  analysis= solution.simpleStaticLinear(prb)
  result= analysis.analyze(1)
  for l in ec3beams:
    l.updateLateralBucklingReductionFactor(crossSectionClass)
  result= analysis.analyze(1) #Update resistant values 
  preprocessor.getLoadHandler.getLoadCombinations.removeFromDomain(nmbComb)

# chiLT= 1.0 #Lateral-torsional buckling reduction factor
# recorder= IPE450A.installULSControlRecorder("element_prop_recorder",setMainBeam.getElements,crossSectionClass,chiLT)
for l in ec3beams:
  l.installULSControlRecorder("element_prop_recorder",crossSectionClass)

analisis= predefined_solutions.simple_static_linear(mainBeam)


combContainer.dumpCombinations(preprocessor)
resultComb(mainBeam,"ELU00")
resultComb(mainBeam,"ELU01")
#resultComb(mainBeam,"ELU02")
#resultComb(mainBeam,"ELU03")

import vtk
from postprocess.xcVtk import vtk_grafico_base
from postprocess.xcVtk.FE_model import vtk_FE_graphic
from postprocess.xcVtk import vtk_internal_force_diagram as ifd
from postprocess.xcVtk import ElementPropertyDiagram as epd

defGrid= vtk_grafico_base.RecordDefGrid()
defGrid.nmbSet= "total"

#diagram= ifd.InternalForceDiagram(0.1,1e-3,[setMainBeam],"Mz")
#diagram= epd.ElementPropertyDiagram(1,1,[setMainBeam],"chiLT")
diagram= epd.ElementPropertyDiagram(10,1,[setMainBeam],"FCTNCP")
#diagram= epd.ElementPropertyDiagram(-0.02,1e-3,[setMainBeam],"Mz-")
#diagram= epd.ElementPropertyDiagram(-0.02,1e-3,[setMainBeam],"Mz+")
#diagram= epd.ElementPropertyDiagram(10,1,[setMainBeam],"FCVCP")
#diagram= epd.ElementPropertyDiagram(0.1,1e-3,[setMainBeam],"Vy+")
#diagram= epd.ElementPropertyDiagram(0.1,1e-3,[setMainBeam],"Vy-")
diagram.addDiagram()

defDisplay= vtk_FE_graphic.RecordDefDisplayEF()
defDisplay.viewName= "YPos"
defDisplay.setupGrid(preprocessor.getSets.getSet('total'))
defDisplay.defineMeshScene(None)
defDisplay.appendDiagram(diagram) #Append diagram to the scene.

execfile('draw_supports.py')
defDisplay.renderer.AddActor(supportsActor)

defDisplay.displayScene()

#ec5CS.printResultsELU(elems,sccPanne)




