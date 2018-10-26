# -*- coding: utf-8 -*-

import os
import xc_base
import geom
import xc
import math
from model import predefined_spaces
from model.geometry import grid_model as gm
from model.mesh import finit_el_model as fem
from model.boundary_cond import spring_bound_cond as sprbc
from model.sets import sets_mng as sets
from materials import typical_materials as tm
from actions import loads
from actions import load_cases as lcases
from actions import combinations as cc
from actions.earth_pressure import earth_pressure as ep
from model.geometry import geom_utils as gut
from materials.ehe import EHE_materials
#from materials.sia262 import SIA262_materials

# Default configuration of environment variables.
import sys
sys.path.append('../..')
from xcmodelsdir import xcmodelsdir
fullProjPath = xcmodelsdir + 'workingModel2/'
execfile(fullProjPath+'env_config.py')

#Auxiliary data
 #Geometry
LbeamX=5
LbeamY=6
LcolumnZ=6
Wfoot=2.0
hbeamX=0.5
hbeamY=0.3
hcolumnZ=0.40
wbeamX=0.35
wbeamY=0.5
wcolumnZ=0.40
deckTh=0.20
wallTh=0.5
footTh=0.7

 #Actions
qdeck1=1e3  #N/m2
# qdeck2=2e3   #N/m2
Qbeam=3e3  #N/m
qunifBeam=5e3
qLinDeck2=30 #N/m
Qwheel=5e3  #N
firad=math.radians(31)  #internal friction angle (radians)                   
KearthPress=(1-math.sin(firad))/(1+math.sin(firad))     #Active coefficient of p
densSoil=800       #mass density of the soil (kg/m3)
densWater=1000      #mass density of the water (kg/m3)


#Materials
concrete=EHE_materials.HA30
reinfSteel=EHE_materials.B500S
# concrete=SIA262_materials.c30_37
# reinfSteel=SIA262_materials.B500B

eSize= 0.2     #length of elements

#             *** GEOMETRIC model (points, lines, surfaces) - SETS ***
FEcase= xc.FEProblem()
preprocessor=FEcase.getPreprocessor
prep=preprocessor   #short name
nodes= prep.getNodeHandler
elements= prep.getElementHandler
elements.dimElem= 3
# Problem type
modelSpace= predefined_spaces.StructuralMechanics3D(nodes) #Defines the
# dimension of the space: nodes by three coordinates (x,y,z) and 
# six DOF for each node (Ux,Uy,Uz,thetaX,thetaY,thetaZ)


###################################################################################################
# coordinates in global X,Y,Z axes for the grid generation
xList=[0,LbeamX/2.0,LbeamX]
yList=[-Wfoot/2.,0,Wfoot/2.,LbeamY]
zList=[0,LcolumnZ/2.0,LcolumnZ]
#auxiliary data
lastXpos=len(xList)-1
lastYpos=len(yList)-1
lastZpos=len(zList)-1

# grid model definition
gridGeom= gm.GridModel(prep,xList,yList,zList)

# Grid geometric entities definition (points, lines, surfaces)
# Points' generation
gridGeom.generatePoints()

#Displacements of the grid points in a range
#syntax: movePointsRange(ijkRange,vDisp)
#        ijkRange: range for the search
#        vDisp: xc vector displacement
# for i in range(1,len(xList)):
#     r= gm.IJKRange((i,0,lastZpos),(i,lastYpos,lastZpos))
#     gridGeom.movePointsRange(r,xc.Vector([0.0,0.0,-trSlope*xList[i]]))


#Slope (in X direction, Y direction or both) the grid points in a range
#syntax: slopePointsRange(ijkRange,slopeX,xZeroSlope,slopeY,yZeroSlope)
#     ijkRange: range for the search.
#     slopeX: slope in X direction, expressed as deltaZ/deltaX 
#                       (defaults to 0 = no slope applied)
#     xZeroSlope: coordinate X of the "rotation axis".
#     slopeY: slope in Y direction, expressed as deltaZ/deltaY)
#                       (defaults to 0 = no slope applied)
#     yZeroSlope: coordinate Y of the "rotation axis".

#Scale in X with origin xOrig (fixed axis: X=xOrig) to the points in a range
#Only X coordinate of points is modified in the following way:
#       x_scaled=xOrig+scale*(x_inic-xOrig)
#syntax: scaleCoorXPointsRange(ijkRange,xOrig,scale)
#     ijkRange: range for the search.
#     xOrig: origin X to apply scale (point in axis X=xOrig)
#            are not affected by the transformation 
#     scale: scale to apply to X coordinate

#Scale in Y with origin yOrig (fixed axis: Y=yOrig) to the points in a range
#Only Y coordinate of points is modified in the following way:
#       y_scaled=yOrig+scale*(y_inic-yOrig)
#syntax: scaleCoorYPointsRange(ijkRange,yOrig,scale)
#     ijkRange: range for the search.
#     yOrig: origin Y to apply scale (point in axis Y=yOrig)
#            are not affected by the transformation 
#     scale: scale to apply to Y coordinate

#Scale in Z with origin zOrig (fixed axis: Z=zOrig) to the points in a range
#Only Z coordinate of points is modified in the following way:
#       z_scaled=zOrig+scale*(z_inic-zOrig)
#syntax: scaleCoorZPointsRange(ijkRange,zOrig,scale)
#     ijkRange: range for the search.
#     zOrig: origin Z to apply scale (point in axis Z=zOrig)
#            are not affected by the transformation 
#     scale: scale to apply to Z coordinate


#Ranges for lines and surfaces
# extractIncludedIJranges(step): subranges index K=constant (default step=1)
# extractIncludedIKranges((step): subranges index J=constant (default step=1)
# extractIncludedJKranges((step): subranges index I=constant (default step=1)
# extractIncludedIranges(stepJ,stepK): subranges indexes J,K=constant (default
#                                      stpes= 1)
# idem for J and K ranges
columnZ_rg=gm.IJKRange((0,1,0),(lastXpos,lastYpos,lastZpos)).extractIncludedKranges(stepI=2,stepJ=2)+[gm.IJKRange((1,lastYpos,0),(1,lastYpos,1))]
decklv1_rg=gm.IJKRange((0,1,1),(lastXpos,lastYpos,lastZpos)).extractIncludedIJranges(step=2)
wall_rg=gm.IJKRange((0,1,0),(lastXpos,1,1))

#Lines generation
columnZ=gridGeom.genLinMultiRegion(lstIJKRange=columnZ_rg,nameSet='columnZ')

#Surfaces generation
decklv1=gridGeom.genSurfMultiRegion(lstIJKRange=decklv1_rg,nameSet='decklv1')
wall=gridGeom.genSurfOneRegion(ijkRange=wall_rg,nameSet='wall')
decklv1.description='Deck level 1'
decklv1.color=cfg.colors['purple01']
wall.description='Wall'
wall.color=cfg.colors['green01']
columnZ.description='Columns'
columnZ.color=cfg.colors['red03']


###################################################################################################
#                         *** MATERIALS *** 
concrProp=tm.MaterialData(name='concrProp',E=concrete.Ecm(),nu=concrete.nuc,rho=concrete.density())

# Isotropic elastic section-material appropiate for plate and shell analysis
deck_mat=tm.DeckMaterialData(name='deck_mat',thickness= deckTh,material=concrProp)
deck_mat.setupElasticSection(preprocessor=prep)   #creates the section-material
wall_mat=tm.DeckMaterialData(name='wall_mat',thickness= wallTh,material=concrProp)
wall_mat.setupElasticSection(preprocessor=prep)   #creates the section-material

#Geometric sections
#rectangular sections
from materials.sections import section_properties as sectpr
geomSectColumnZ=sectpr.RectangularSection(name='geomSectColumnZ',b=wcolumnZ,h=hcolumnZ)

# Elastic material-section appropiate for 3D beam analysis, including shear
  # deformations.
  # Attributes:
  #   name:         name identifying the section
  #   section:      instance of a class that defines the geometric and
  #                 mechanical characteristiscs
  #                 of a section (e.g: RectangularSection, CircularSection,
  #                 ISection, ...)
  #   material:     instance of a class that defines the elastic modulus,
  #                 shear modulus and mass density of the material

columnZ_mat= tm.BeamMaterialData(name= 'columnZ_mat', section=geomSectColumnZ, material=concrProp)
columnZ_mat.setupElasticShear3DSection(preprocessor=prep)


###################################################################################################
#                         ***FE model - MESH***
# IMPORTANT: it's convenient to generate the mesh of surfaces before meshing
# the lines, otherwise, sets of shells can take also beam elements touched by


columnZ_mesh=fem.LinSetToMesh(linSet=columnZ,matSect=columnZ_mat,elemSize=eSize,vDirLAxZ=xc.Vector([1,0,0]),elemType='ElasticBeam3d',coordTransfType='linear')
decklv1_mesh=fem.SurfSetToMesh(surfSet=decklv1,matSect=deck_mat,elemSize=eSize,elemType='ShellMITC4')
decklv1_mesh.generateMesh(prep)     #mesh the set of surfaces
wall_mesh=fem.SurfSetToMesh(surfSet=wall,matSect=wall_mat,elemSize=eSize,elemType='ShellMITC4')
wall_mesh.generateMesh(prep) 

fem.multi_mesh(preprocessor=prep,lstMeshSets=[columnZ_mesh])     #mesh these sets


###################################################################################################
#                       ***BOUNDARY CONDITIONS***
# Regions resting on springs (Winkler elastic foundation)
#       wModulus: Winkler modulus of the foundation (springs in Z direction)
#       cRoz:     fraction of the Winkler modulus to apply for friction in
#                 the contact plane (springs in X, Y directions)
#foot_wink=sprbc.ElasticFoundation(wModulus=20e7,cRoz=0.2)
#foot_wink.generateSprings(xcSet=foot)

# Springs (defined by Kx,Ky,Kz) to apply on nodes, points, 3Dpos, ...
# Default values for Kx, Ky, Kz are 0, which means that no spring is
# created in the corresponding direction
# spring_col=sprbc.SpringBC(name='spring_col',modelSpace=modelSpace,Kx=10e3,Ky=50e3,Kz=30e3)
# a=spring_col.applyOnNodesIn3Dpos(lst3DPos=[geom.Pos3d(0,LbeamY,0)])

#fixed DOF (ux:'0FF_FFF', uy:'F0F_FFF', uz:'FF0_FFF',
#           rx:'FFF_0FF', ry:'FFF_F0F', rz:'FFF_FF0')
n_col1=nodes.getDomain.getMesh.getNearestNode(geom.Pos3d(0,LbeamY,0))
modelSpace.fixNode('000_FFF',n_col1.tag)
n_col2=nodes.getDomain.getMesh.getNearestNode(geom.Pos3d(LbeamX,LbeamY,0))
modelSpace.fixNode('000_FFF',n_col2.tag)
n_col3=nodes.getDomain.getMesh.getNearestNode(geom.Pos3d(LbeamX/2.,LbeamY,0))
modelSpace.fixNode('FF0_000',n_col3.tag)



###################################################################################################
decks=decklv1
decks.name='decks'
decks.description='Decks'
decks.color=cfg.colors['purple01']
allShells=decklv1+wall
allShells.description='Shell elements'
overallSet=columnZ+wall+decklv1
#overallSet=beamX+beamY+columnZ+wall+foot+decklv1+decklv2
overallSet.description='overall set'
overallSet.color=cfg.colors['purple01']

#sets for displaying some results
pBase=gut.rect2DPolygon(xCent=LbeamX/2.,yCent=0,Lx=LbeamX,Ly=LbeamY-1.0)

allShellsRes=sets.set_included_in_orthoPrism(preprocessor=prep,setInit=allShells,prismBase=pBase,prismAxis='Z',setName='allShellsRes')
 




