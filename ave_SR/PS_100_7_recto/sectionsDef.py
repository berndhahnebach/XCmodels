# -*- coding: utf-8 -*-

#import os
import xc_base
import geom
import xc
from materials.sections.fiber_section import defSimpleRCSection as rcs
#from materials.ehe import EHE_materials
from materials.ec2 import EC2_materials
import math

#Auxiliary data
execfile('../basic_data.py')


rnom=35 #recubrimiento nominal 

areaFi16= math.pi*(16*1e-3)**2/4.0

#instances of rcs.RecordRCSlabBeamSection that define the
#variables that make up THE TWO reinforced concrete sections in the two
#reinforcement directions of a slab or the front and back ending sections
#of a beam element

losSupV2RCSects= rcs.RecordRCSlabBeamSection(name='losSupV2RCSects',sectionDescr='losa aligerada, cara superior.',concrType=concrete, reinfSteelType=reinfSteel,depth=espLosAlig,elemSetName='losSupV2')
#D1: transversal rebars
#D2: longitudinal rebars
#positiv: top face
#negativ: bottom face
losSupV2RCSects.dir1PositvRebarRows=[rcs.rebLayer(16,200,rnom)]
losSupV2RCSects.dir1NegatvRebarRows=[rcs.rebLayer(12,200,rnom+espLosAlig/2.)]
losSupV2RCSects.dir2PositvRebarRows=[rcs.rebLayer(16,200,rnom+20)]
losSupV2RCSects.dir2NegatvRebarRows=[rcs.rebLayer(12,200,rnom+espLosAlig/2.)]


losInfV2RCSects= rcs.RecordRCSlabBeamSection(name='losInfV2RCSects',sectionDescr='losa aligerada, cara inferior',concrType=concrete, reinfSteelType=reinfSteel,depth=espLosAlig,elemSetName='losInfV2')
#D1: transversal rebars
#D2: longitudinal rebars
#positiv: top face
#negativ: bottom face
losInfV2RCSects.dir1NegatvRebarRows=[rcs.rebLayer(14,200,rnom)]
losInfV2RCSects.dir1PositvRebarRows=[rcs.rebLayer(8,200,rnom)]
losInfV2RCSects.dir2NegatvRebarRows=[rcs.rebLayer(20,200,rnom+espLosAlig/2.0)]
losInfV2RCSects.dir2PositvRebarRows=[rcs.rebLayer(25,200,rnom+16)]


voladzCentV2RCSects= rcs.RecordRCSlabBeamSection(name='voladzCentV2RCSects',sectionDescr='voladizo, zona central vano',concrType=concrete, reinfSteelType=reinfSteel,depth=espVoladzMax,elemSetName='voladzCentV2')
#D1: transversal rebars
#D2: longitudinal rebars
#positiv: top face
#negativ: bottom face
voladzCentV2RCSects.dir1PositvRebarRows=[rcs.rebLayer(20,200,rnom)]
voladzCentV2RCSects.dir2PositvRebarRows=[rcs.rebLayer(20,200,rnom+20)]
voladzCentV2RCSects.dir1NegatvRebarRows=[rcs.rebLayer(12,200,rnom)]
voladzCentV2RCSects.dir2NegatvRebarRows=[rcs.rebLayer(12,200,rnom+12)]

voladzExtrV2RCSects= rcs.RecordRCSlabBeamSection(name='voladzExtrV2RCSects',sectionDescr='voladizo, zona exterior vano',concrType=concrete, reinfSteelType=reinfSteel,depth=espVoladzMin,elemSetName='voladzExtrV2')
#D1: transversal rebars
#D2: longitudinal rebars
voladzExtrV2RCSects.dir1PositvRebarRows=[rcs.rebLayer(20,200,rnom)]
voladzExtrV2RCSects.dir2PositvRebarRows=[rcs.rebLayer(20,200,rnom+20)]
voladzExtrV2RCSects.dir1NegatvRebarRows=[rcs.rebLayer(12,200,rnom)]
voladzExtrV2RCSects.dir2NegatvRebarRows=[rcs.rebLayer(12,200,rnom+12)]


losSupRP1RCSects= rcs.RecordRCSlabBeamSection(name='losSupRP1RCSects',sectionDescr='riostra sobre pila, cara superior.',concrType=concrete, reinfSteelType=reinfSteel,depth=espLosAlig,elemSetName='losSupRP1')
#D1: transversal rebars
#D2: longitudinal rebars
#positiv: top face
#negativ: bottom face
losSupRP1RCSects.dir1PositvRebarRows=[rcs.rebLayer(25,130,rnom)]
losSupRP1RCSects.dir1NegatvRebarRows=[rcs.rebLayer(25,130,rnom+espLosAlig/2.0)]
losSupRP1RCSects.dir2PositvRebarRows=[rcs.rebLayer(25,200,rnom)]
losSupRP1RCSects.dir2NegatvRebarRows=[rcs.rebLayer(20,200,rnom+espLosAlig/2.0)]


losInfRP1RCSects= rcs.RecordRCSlabBeamSection(name='losInfRP1RCSects',sectionDescr='riostra sobre pila, cara inferior',concrType=concrete, reinfSteelType=reinfSteel,depth=espLosAlig,elemSetName='losInfRP1')
#D1: transversal rebars
#D2: longitudinal rebars
#positiv: top face
#negativ: bottom face
losInfRP1RCSects.dir1NegatvRebarRows=[rcs.rebLayer(16,130,rnom),rcs.rebLayer(25,130,rnom+16)]
losInfRP1RCSects.dir1PositvRebarRows=[rcs.rebLayer(16,130,rnom++espLosAlig/2.0)]
losInfRP1RCSects.dir2NegatvRebarRows=[rcs.rebLayer(25,200,rnom)]
losInfRP1RCSects.dir2PositvRebarRows=[rcs.rebLayer(16,130,rnom+40)]

voladzCentRP1RCSects= rcs.RecordRCSlabBeamSection(name='voladzCentRP1RCSects',sectionDescr='voladizo, zona central riostra pila',concrType=concrete, reinfSteelType=reinfSteel,depth=espVoladzMax,elemSetName='voladzCentRP1')
#D1: transversal rebars
#D2: longitudinal rebars
#positiv: top face
#negativ: bottom face
voladzCentRP1RCSects.dir1PositvRebarRows=[rcs.rebLayer(25,130,rnom)]
voladzCentRP1RCSects.dir2PositvRebarRows=[rcs.rebLayer(20,200,rnom+25),rcs.rebLayer(25,200,rnom+25)]
voladzCentRP1RCSects.dir1NegatvRebarRows=[rcs.rebLayer(12,130,rnom)]
voladzCentRP1RCSects.dir2NegatvRebarRows=[rcs.rebLayer(12,200,rnom+12)]

voladzExtrRP1RCSects= rcs.RecordRCSlabBeamSection(name='voladzExtrRP1RCSects',sectionDescr='voladizo, zona exterior riostra pila',concrType=concrete, reinfSteelType=reinfSteel,depth=espVoladzMax,elemSetName='voladzExtrRP1')
#D1: transversal rebars
#D2: longitudinal rebars
#positiv: top face
#negativ: bottom face
voladzExtrRP1RCSects.dir1PositvRebarRows=[rcs.rebLayer(25,130,rnom)]
voladzExtrRP1RCSects.dir2PositvRebarRows=[rcs.rebLayer(20,200,rnom+25),rcs.rebLayer(25,200,rnom+25)]
voladzExtrRP1RCSects.dir1NegatvRebarRows=[rcs.rebLayer(12,130,rnom)]
voladzExtrRP1RCSects.dir2NegatvRebarRows=[rcs.rebLayer(12,200,rnom+12)]


murAligV2RCSects= rcs.RecordRCSlabBeamSection(name='murAligV2RCSects',sectionDescr='nervios losa aligerada',concrType=concrete, reinfSteelType=reinfSteel,depth=espEntreAlig,elemSetName='murAligV2')
#D1: horizontal rebars
#D2: vertical rebars
#positiv: top face
#negativ: bottom face
# murAligV2RCSects.dir1PositvRebarRows=[rcs.rebLayer(8,200,rnom+12)]
# murAligV2RCSects.dir2PositvRebarRows=[rcs.rebLayer(16,200,rnom)]
# murAligV2RCSects.dir1NegatvRebarRows=[rcs.rebLayer(8,200,rnom+12)]
# murAligV2RCSects.dir2NegatvRebarRows=[rcs.rebLayer(16,200,rnom)]
murAligV2RCSects.dir1PositvRebarRows=[rcs.rebLayer(8,200,rnom+12)]
murAligV2RCSects.dir2PositvRebarRows=[rcs.rebLayer(8,200,rnom)]
murAligV2RCSects.dir1NegatvRebarRows=[rcs.rebLayer(8,200,rnom+12)]
murAligV2RCSects.dir2NegatvRebarRows=[rcs.rebLayer(8,200,rnom)]
shear2=rcs.RecordShearReinforcement(familyName= "shear2",nShReinfBranches= 2.0,areaShReinfBranch= areaFi16,shReinfSpacing= 0.10,angAlphaShReinf= math.pi/2.0,angThetaConcrStruts= math.pi/4.0)
murAligV2RCSects.dir2ShReinfY=shear2


pilasInfRCSects= rcs.RecordRCSlabBeamSection(name='pilasInfRCSects',sectionDescr='pilas, zona inferior ',concrType=concrete, reinfSteelType=reinfSteel,width=lRectEqPila,depth=lRectEqPila,elemSetName='pilasInf')
#comprobación a cortante
#pilasInfRCSects= rcs.RecordRCSlabBeamSection(name='pilasInfRCSects',sectionDescr='pilas ',concrType=concrete, reinfSteelType=reinfSteel,width=1.0,depth=lRectEqPila**2,elemSetName='pilasInf')
#D1: cara dorsal
#D2: cara frontal
#positiv: top face
#negativ: bottom face
sep_mm=(lRectEqPila*1e3-2*(rnom +16+25/2.))/8.
capa1=rcs.rebLayer(25,100,rnom +16)
capa1.nRebars=8
capa2=rcs.rebLayer(25,100,rnom +29+sep_mm)
capa2.nRebars=2
capa3=rcs.rebLayer(25,100,rnom +29+2*sep_mm)
capa3.nRebars=2
capa4=rcs.rebLayer(25,100,rnom +29+3*sep_mm)
capa4.nRebars=2
capacent=rcs.rebLayer(25,100,rnom +29+4*sep_mm)
capacent.nRebars=2
shear1=rcs.RecordShearReinforcement(familyName= "shear1",nShReinfBranches= 2.0,areaShReinfBranch= areaFi16,shReinfSpacing= 0.15,angAlphaShReinf= math.pi/2.0,angThetaConcrStruts= math.pi/4.0)

pilasInfRCSects.dir1PositvRebarRows=[capa1,capa2,capa3,capa4]
pilasInfRCSects.dir1NegatvRebarRows=[capa1,capa2,capa3,capa4,capacent]
pilasInfRCSects.dir2PositvRebarRows=[capa1,capa2,capa3,capa4]
pilasInfRCSects.dir2NegatvRebarRows=[capa1,capa2,capa3,capa4,capacent]
pilasInfRCSects.dir1ShReinfY=shear1 
pilasInfRCSects.dir2ShReinfY=shear1 

pilasSupRCSects= rcs.RecordRCSlabBeamSection(name='pilasSupRCSects',sectionDescr='pilas, zona superior ',concrType=concrete, reinfSteelType=reinfSteel,width=lRectEqPila,depth=lRectEqPila,elemSetName='pilasSup')
#comprobación a cortante
#pilasSupRCSects= rcs.RecordRCSlabBeamSection(name='pilasSupRCSects',sectionDescr='pilas ',concrType=concrete, reinfSteelType=reinfSteel,width=1.0,depth=lRectEqPila**2,elemSetName='pilasSup')
pilasSupRCSects.dir1PositvRebarRows=[capa1,capa2,capa3,capa4]
pilasSupRCSects.dir1NegatvRebarRows=[capa1,capa2,capa3,capa4,capacent]
pilasSupRCSects.dir2PositvRebarRows=[capa1,capa2,capa3,capa4]
pilasSupRCSects.dir2NegatvRebarRows=[capa1,capa2,capa3,capa4,capacent]
pilasSupRCSects.dir1ShReinfY=shear1 
pilasSupRCSects.dir2ShReinfY=shear1 


riostrEstr1RCSects= rcs.RecordRCSlabBeamSection(name='riostrEstr1RCSects',sectionDescr='riostra estribo 1 ',concrType=concrete, reinfSteelType=reinfSteel,width=espRiostrEstr,depth=cantoLosa,elemSetName='riostrEstr1')
sep_mm=(cantoLosa*1e3-2*(rnom +16+25/2.))/6.
capa5=rcs.rebLayer(25,100,rnom +16)
capa5.nRebars=9
capa6=rcs.rebLayer(12,100,rnom +29+sep_mm)
capa6.nRebars=2
capa7=rcs.rebLayer(12,100,rnom +29+2*sep_mm)
capa7.nRebars=2
capa8=rcs.rebLayer(12,100,rnom +29+3*sep_mm)
capa8.nRebars=2
shear2=rcs.RecordShearReinforcement(familyName= "shear2",nShReinfBranches= 3.0,areaShReinfBranch= areaFi16,shReinfSpacing= 0.20,angAlphaShReinf= math.pi/2.0,angThetaConcrStruts= math.pi/4.0)

riostrEstr1RCSects.dir1PositvRebarRows=[capa5,capa6,capa7]
riostrEstr1RCSects.dir1NegatvRebarRows=[capa5,capa6,capa7,capa8]
riostrEstr1RCSects.dir2PositvRebarRows=[capa5,capa6,capa7]
riostrEstr1RCSects.dir2NegatvRebarRows=[capa5,capa6,capa7,capa8]
riostrEstr1RCSects.dir1ShReinfY=shear2
riostrEstr1RCSects.dir2ShReinfY=shear2
riostrEstr1RCSects.dir1ShReinfZ=shear2
riostrEstr1RCSects.dir2ShReinfZ=shear2



