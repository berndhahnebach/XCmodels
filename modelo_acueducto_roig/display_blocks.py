execfile('./fe_model.py')
from postprocess.xcVtk.CAD_model import vtk_CAD_graphic

defDisplay= vtk_CAD_graphic.RecordDefDisplayCAD()

defDisplay.displayBlocks(xcTotalSet,caption= xcTotalSet.name+' set')
#defDisplay.displayBlocks(setParapet,caption= setParapet.name+' set')
#defDisplay.displayBlocks(setColumns,caption= setColumns.name+' set')
#defDisplay.displayBlocks(setTransverseBeams,caption= setTransverseBeams.name+' set')
#defDisplay.displayBlocks(springLines,caption= springLines.name+' set')