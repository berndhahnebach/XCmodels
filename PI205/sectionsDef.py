# -*- coding: utf-8 -*-

import math
import os
import xc_base
import geom
import xc
# Macros
#from materials.ehe import auxEHE
from materials.sections.fiber_section import defSimpleRCSection
from postprocess import RC_material_distribution




from materials.sia262 import SIA262_materials as concrete
from materials.sia262 import SIA262_materials
from materials.sia262 import SIA262_materials
from materials.sia262 import SIA262_limit_state_checking

from postprocess import limit_state_data as lsd
from postprocess import element_section_map


areaFi6= SIA262_materials.section_barres_courantes[6e-3]
areaFi8= SIA262_materials.section_barres_courantes[8e-3]
areaFi10= SIA262_materials.section_barres_courantes[10e-3]
areaFi12= SIA262_materials.section_barres_courantes[12e-3]
areaFi14= SIA262_materials.section_barres_courantes[14e-3]
areaFi16= SIA262_materials.section_barres_courantes[16e-3]
areaFi18= SIA262_materials.section_barres_courantes[18e-3]
areaFi20= SIA262_materials.section_barres_courantes[20e-3]
areaFi22= SIA262_materials.section_barres_courantes[22e-3]
areaFi26= SIA262_materials.section_barres_courantes[26e-3]
areaFi30= SIA262_materials.section_barres_courantes[30e-3]
areaFi34= SIA262_materials.section_barres_courantes[34e-3]
areaFi40= SIA262_materials.section_barres_courantes[40e-3]

concrete= concrete.c30_37
reinfSteel= SIA262_materials.B500A

reinfConcreteSectionDistribution= RC_material_distribution.RCMaterialDistribution()
sections= reinfConcreteSectionDistribution.sectionDefinition

#Generic layers (rows of rebars)
fi8s125r30=defSimpleRCSection.MainReinfLayer(rebarsDiam=8e-3,areaRebar= areaFi8,rebarsSpacing=0.125,width=1.0,nominalCover=0.030)
fi8s125r44=defSimpleRCSection.MainReinfLayer(rebarsDiam=8e-3,areaRebar= areaFi8,rebarsSpacing=0.125,width=1.0,nominalCover=0.044)
fi10s200r44=defSimpleRCSection.MainReinfLayer(rebarsDiam=10e-3,areaRebar= areaFi10,rebarsSpacing=0.200,width=1.0,nominalCover=0.044)
fi10s250r42=defSimpleRCSection.MainReinfLayer(rebarsDiam=10e-3,areaRebar= areaFi10,rebarsSpacing=0.250,width=1.0,nominalCover=0.042)
fi12s250r30=defSimpleRCSection.MainReinfLayer(rebarsDiam=12e-3,areaRebar= areaFi12,rebarsSpacing=0.250,width=1.0,nominalCover=0.030)
fi12s250r46=defSimpleRCSection.MainReinfLayer(rebarsDiam=12e-3,areaRebar= areaFi12,rebarsSpacing=0.250,width=1.0,nominalCover=0.046)
fi12s150r35=defSimpleRCSection.MainReinfLayer(rebarsDiam=12e-3,areaRebar= areaFi12,rebarsSpacing=0.150,width=1.0,nominalCover=0.035)
fi14s250r30=defSimpleRCSection.MainReinfLayer(rebarsDiam=14e-3,areaRebar= areaFi14,rebarsSpacing=0.25,width=1.0,nominalCover=0.030)
fi14s125r30=defSimpleRCSection.MainReinfLayer(rebarsDiam=14e-3,areaRebar= areaFi14,rebarsSpacing=0.125,width=1.0,nominalCover=0.030)
fi16s125r30=defSimpleRCSection.MainReinfLayer(rebarsDiam=16e-3,areaRebar= areaFi16,rebarsSpacing=0.125,width=1.0,nominalCover=0.030)
fi16s250r50=defSimpleRCSection.MainReinfLayer(rebarsDiam=16e-3,areaRebar= areaFi16,rebarsSpacing=0.250,width=1.0,nominalCover=0.050)
fi16s150r35=defSimpleRCSection.MainReinfLayer(rebarsDiam=16e-3,areaRebar= areaFi16,rebarsSpacing=0.150,width=1.0,nominalCover=0.035)
fi16s250r56=defSimpleRCSection.MainReinfLayer(rebarsDiam=16e-3,areaRebar= areaFi16,rebarsSpacing=0.250,width=1.0,nominalCover=0.056)
fi18s125r30=defSimpleRCSection.MainReinfLayer(rebarsDiam=18e-3,areaRebar= areaFi18,rebarsSpacing=0.125,width=1.0,nominalCover=0.030)
fi18s125r44=defSimpleRCSection.MainReinfLayer(rebarsDiam=18e-3,areaRebar= areaFi18,rebarsSpacing=0.125,width=1.0,nominalCover=0.044)
fi18s150r35=defSimpleRCSection.MainReinfLayer(rebarsDiam=18e-3,areaRebar= areaFi18,rebarsSpacing=0.150,width=1.0,nominalCover=0.035)
fi20s250r30=defSimpleRCSection.MainReinfLayer(rebarsDiam=20e-3,areaRebar= areaFi20,rebarsSpacing=0.250,width=1.0,nominalCover=0.030)
fi20s150r35=defSimpleRCSection.MainReinfLayer(rebarsDiam=20e-3,areaRebar= areaFi20,rebarsSpacing=0.150,width=1.0,nominalCover=0.035)
fi22s150r35=defSimpleRCSection.MainReinfLayer(rebarsDiam=22e-3,areaRebar= areaFi22,rebarsSpacing=0.150,width=1.0,nominalCover=0.035)
fi20s125r30=defSimpleRCSection.MainReinfLayer(rebarsDiam=20e-3,areaRebar= areaFi20,rebarsSpacing=0.125,width=1.0,nominalCover=0.030)
fi20s125r50=defSimpleRCSection.MainReinfLayer(rebarsDiam=20e-3,areaRebar= areaFi20,rebarsSpacing=0.125,width=1.0,nominalCover=0.050)
fi26s250r30=defSimpleRCSection.MainReinfLayer(rebarsDiam=26e-3,areaRebar= areaFi26,rebarsSpacing=0.250,width=1.0,nominalCover=0.030)
fi26s250r50=defSimpleRCSection.MainReinfLayer(rebarsDiam=26e-3,areaRebar= areaFi26,rebarsSpacing=0.250,width=1.0,nominalCover=0.050)

DeckExtSlabRCSect= defSimpleRCSection.RecordRCSlabBeamSection('deckExtSlabRCSect',"underpass.",concrete, reinfSteel,0.55)
#[0]: transversal rebars
#[1]: longitudinal rebars
DeckExtSlabRCSect.dir1PositvRebarRows=[fi20s150r35,fi12s150r35]
DeckExtSlabRCSect.dir1NegatvRebarRows=[fi20s150r35]
DeckExtSlabRCSect.dir2PositvRebarRows=[fi12s150r35]
DeckExtSlabRCSect.dir2NegatvRebarRows=[fi12s150r35]

DeckExtSlabRCSect.creaTwoSections() 
sections.append(DeckExtSlabRCSect)


DeckIntSlabRCSect= defSimpleRCSection.RecordRCSlabBeamSection('deckIntSlabRCSect',"underpass.",concrete, reinfSteel,0.5)
#[0]: transversal rebars
#[1]: longitudinal rebars
DeckIntSlabRCSect.dir1PositvRebarRows=[fi12s150r35]
DeckIntSlabRCSect.dir1NegatvRebarRows=[fi22s150r35,fi22s150r35]
DeckIntSlabRCSect.dir2PositvRebarRows=[fi12s150r35]
DeckIntSlabRCSect.dir2NegatvRebarRows=[fi18s150r35]

DeckIntSlabRCSect.creaTwoSections() 
sections.append(DeckIntSlabRCSect)

FoundExtSlabRCSect= defSimpleRCSection.RecordRCSlabBeamSection(name='foundExtSlabRCSect',sectionDescr="underpass.",depth=0.60,concrType=concrete, reinfSteelType=reinfSteel)
#[0]: transversal rebars
#[1]: longitudinal rebars
FoundExtSlabRCSect.dir1PositvRebarRows=[fi20s150r35]
FoundExtSlabRCSect.dir1NegatvRebarRows=[fi20s150r35,fi12s150r35]
FoundExtSlabRCSect.dir2PositvRebarRows=[fi12s150r35]
FoundExtSlabRCSect.dir2NegatvRebarRows=[fi12s150r35]

FoundExtSlabRCSect.creaTwoSections() 
sections.append(FoundExtSlabRCSect)

FoundIntSlabRCSect= defSimpleRCSection.RecordRCSlabBeamSection('foundIntSlabRCSect',"underpass.",concrete, reinfSteel,0.60)
#[0]: transversal rebars
#[1]: longitudinal rebars
FoundIntSlabRCSect.dir1PositvRebarRows=[fi20s150r35,fi20s150r35]
FoundIntSlabRCSect.dir1NegatvRebarRows=[fi12s150r35]
FoundIntSlabRCSect.dir2PositvRebarRows=[fi16s150r35]
FoundIntSlabRCSect.dir2NegatvRebarRows=[fi12s150r35]

FoundIntSlabRCSect.creaTwoSections() 
sections.append(FoundIntSlabRCSect)

LeftWallRCSect= defSimpleRCSection.RecordRCSlabBeamSection('leftWallRCSect',"underpass.",concrete, reinfSteel,0.45)
#[0]: horizontal rebars
#[1]: vertical rebars
LeftWallRCSect.dir1PositvRebarRows=[fi12s150r35]
LeftWallRCSect.dir1NegatvRebarRows=[fi12s150r35]
LeftWallRCSect.dir2PositvRebarRows=[fi12s150r35]
LeftWallRCSect.dir2NegatvRebarRows=[fi20s150r35,fi16s150r35]

LeftWallRCSect.creaTwoSections() 
sections.append(LeftWallRCSect)

RightWallRCSect= defSimpleRCSection.RecordRCSlabBeamSection('rightWallRCSect',"underpass.",concrete, reinfSteel,0.45)
#[0]: horizontal rebars
#[1]: vertical rebars
RightWallRCSect.dir1PositvRebarRows=[fi12s150r35]
RightWallRCSect.dir1NegatvRebarRows=[fi12s150r35]
RightWallRCSect.dir2PositvRebarRows=[fi20s150r35,fi16s150r35]
RightWallRCSect.dir2NegatvRebarRows=[fi12s150r35]

RightWallRCSect.creaTwoSections() 
sections.append(RightWallRCSect)






