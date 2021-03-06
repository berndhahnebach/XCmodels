# -*- coding: utf-8 -*-

import math
import os
import xc_base
import geom
import xc
# Macros
from materials.sections.fiber_section import defSimpleRCSection
from postprocess import RC_material_distribution



from materials.ehe import EHE_materials
from materials.sia262 import SIA262_limit_state_checking
from postprocess import limit_state_data as lsd
from postprocess import element_section_map

#Thickness of the elements


concrete=EHE_materials.HA30
reinfSteel= EHE_materials.B500S


reinfConcreteSectionDistribution= RC_material_distribution.RCMaterialDistribution()
sections= reinfConcreteSectionDistribution.sectionDefinition

execfile('../generic_layers_aux.py')

#instances of defSimpleRCSection.RecordRCSlabBeamSection that defines the
#variables that make up THE TWO reinforced concrete sections in the two
#reinforcement directions of a slab or the front and back ending sections
#of a beam element

dintelRCSects= defSimpleRCSection.RecordRCSlabBeamSection(name='dintel',sectionDescr='dintel',concrType=concrete, reinfSteelType=reinfSteel,depth=deckTh)  
dintelRCSects.dir1PositvRebarRows=[fi16s150r60]  #long. sup.
dintelRCSects.dir1NegatvRebarRows=[fi20s200r60]  #long. inf.
dintelRCSects.dir2PositvRebarRows=[fi16s200r60]  #transv.sup
dintelRCSects.dir2NegatvRebarRows=[fi20s150r60,fi20s150r60]  #transv.inf

dintelRCSects.creaTwoSections() 
sections.append(dintelRCSects)   

