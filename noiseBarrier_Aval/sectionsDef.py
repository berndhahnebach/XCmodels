# -*- coding: utf-8 -*-

import math
import os
import xc_base
import geom
import xc
# Macros
from materials.sections.fiber_section import defSimpleRCSection
from postprocess import RC_material_distribution



from materials.sia262 import SIA262_materials
from materials.sia262 import SIA262_limit_state_checking

from postprocess import limit_state_data as lsd
from postprocess import element_section_map

#Béton project: béton 35/25 

#Dimensions of the pieces (rectangular approximation)
#Bottom column
BCside=0.90     #side of the square column

#Top column
TCwidthSupFlange=0.45
TCthickSupFlange=0.14
TCthickWeb=0.20
TCheightWeb=0.24
TCwidthInfFlange=0.45
TCthickInfFlange=0.17

ArTC=TCwidthSupFlange*TCthickSupFlange+TCthickWeb*TCheightWeb+TCwidthInfFlange*TCthickInfFlange

TCheight=TCthickSupFlange+TCheightWeb+TCthickInfFlange
TCwidth=ArTC/TCheight

#Curve column
CCheight=0.5
CCwidth=0.2

concrete= SIA262_materials.c25_30
reinfSteel= SIA262_materials.B500A
coverForAll=0.025

reinfConcreteSectionDistribution= RC_material_distribution.RCMaterialDistribution()
sections= reinfConcreteSectionDistribution.sectionDefinition

#execfile('../generic_layers_aux.py')
execfile('../generic_fis.py')


botColRCSects= defSimpleRCSection.RecordRCSlabBeamSection(name='botColRCSects',sectionDescr='fut fondation',concrType=concrete, reinfSteelType=reinfSteel,width=BCside-0.25,depth=BCside)
#auxiliar data
layer1=defSimpleRCSection.MainReinfLayer(rebarsDiam=14e-3,areaRebar= areaFi14,rebarsSpacing=coverForAll+14e-3,width=BCside,nominalCover=coverForAll)
layer1.nRebars=8
layer2=defSimpleRCSection.MainReinfLayer(rebarsDiam=10e-3,areaRebar= areaFi10,rebarsSpacing=coverForAll+14e-3,width=BCside,nominalCover=coverForAll+0.12)
layer2.nRebars=2
layer3=defSimpleRCSection.MainReinfLayer(rebarsDiam=10e-3,areaRebar= areaFi10,rebarsSpacing=coverForAll+14e-3,width=BCside,nominalCover=coverForAll+0.26)
layer3.nRebars=2
layer4=defSimpleRCSection.MainReinfLayer(rebarsDiam=10e-3,areaRebar= areaFi10,rebarsSpacing=coverForAll+14e-3,width=BCside,nominalCover=coverForAll+0.42)
layer4.nRebars=2

shear1=defSimpleRCSection.RecordShearReinforcement(familyName= "sh1",nShReinfBranches= 2.0,areaShReinfBranch= areaFi8,shReinfSpacing= 0.15,angAlphaShReinf= math.pi/2.0,angThetaConcrStruts= math.pi/4.0)
#end auxiliar data
botColRCSects.dir1PositvRebarRows=[layer1,layer2,layer3,layer4]
botColRCSects.dir1NegatvRebarRows=[layer1,layer2,layer3]
botColRCSects.dir2PositvRebarRows=[layer1,layer2,layer3,layer4]
botColRCSects.dir2NegatvRebarRows=[layer1,layer2,layer3]
botColRCSects.dir1ShReinfY=shear1
botColRCSects.dir1ShReinfZ=shear1
botColRCSects.dir2ShReinfY=shear1
botColRCSects.dir2ShReinfZ=shear1

botColRCSects.creaTwoSections() 
sections.append(botColRCSects)

topColRCSects= defSimpleRCSection.RecordRCSlabBeamSection(name='topColRCSects',sectionDescr='pilier prefabriqué inf.',concrType=concrete, reinfSteelType=reinfSteel,width=TCheight,depth=TCwidth)  
#auxiliar data
layer1=defSimpleRCSection.MainReinfLayer(rebarsDiam=16e-3,areaRebar= areaFi16,rebarsSpacing=coverForAll+16e-3,width=TCheight,nominalCover=coverForAll)
layer1.nRebars=2
layer2=defSimpleRCSection.MainReinfLayer(rebarsDiam=16e-3,areaRebar= areaFi16,rebarsSpacing=coverForAll+16e-3,width=TCheight,nominalCover=coverForAll+0.1)
layer2.nRebars=2
layer3=defSimpleRCSection.MainReinfLayer(rebarsDiam=16e-3,areaRebar= areaFi16,rebarsSpacing=coverForAll+16e-3,width=TCheight,nominalCover=coverForAll+0.15)
layer3.nRebars=2
layer4=defSimpleRCSection.MainReinfLayer(rebarsDiam=12e-3,areaRebar= areaFi12,rebarsSpacing=0.10,width=TCheight,nominalCover=coverForAll)
layer4.nRebars=2
layer5=defSimpleRCSection.MainReinfLayer(rebarsDiam=12e-3,areaRebar= areaFi12,rebarsSpacing=0.10,width=TCheight,nominalCover=coverForAll+0.1)
layer5.nRebars=3

#end auxiliar data
topColRCSects.dir1PositvRebarRows=[layer1,layer2,layer3,layer4,layer5]
topColRCSects.dir1NegatvRebarRows=[layer1,layer2,layer4,layer5]
topColRCSects.dir2PositvRebarRows=[layer1,layer2,layer3,layer4,layer5]
topColRCSects.dir2NegatvRebarRows=[layer1,layer2,layer4,layer5]
topColRCSects.dir1ShReinfY=shear1
topColRCSects.dir1ShReinfZ=shear1
topColRCSects.dir2ShReinfY=shear1
topColRCSects.dir2ShReinfZ=shear1

topColRCSects.creaTwoSections() 
sections.append(topColRCSects)

glsColRCSects= defSimpleRCSection.RecordRCSlabBeamSection(name='glsColRCSects',sectionDescr='pilier prefabriqué sup.',concrType=concrete, reinfSteelType=reinfSteel,width=CCheight,depth=CCwidth)  
#auxiliar data
layer1=defSimpleRCSection.MainReinfLayer(rebarsDiam=14e-3,areaRebar= areaFi14,rebarsSpacing=coverForAll+12e-3,width=CCheight,nominalCover=coverForAll)
layer1.nRebars=2
layer2=defSimpleRCSection.MainReinfLayer(rebarsDiam=10e-3,areaRebar= areaFi10,rebarsSpacing=0.20,width=CCheight,nominalCover=coverForAll)
layer2.nRebars=2
#end auxiliar data
glsColRCSects.dir1PositvRebarRows=[layer1,layer2]
glsColRCSects.dir1NegatvRebarRows=[layer1,layer2]
glsColRCSects.dir2PositvRebarRows=[layer1,layer2]
glsColRCSects.dir2NegatvRebarRows=[layer1,layer2]
glsColRCSects.dir1ShReinfY=shear1
glsColRCSects.dir1ShReinfZ=shear1
glsColRCSects.dir2ShReinfY=shear1
glsColRCSects.dir2ShReinfZ=shear1

glsColRCSects.creaTwoSections() 
sections.append(glsColRCSects)
