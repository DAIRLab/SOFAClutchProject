# -*- coding: utf-8 -*-

import Sofa

import os
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

# this scene is an adapted example from the SoftRobots plugin, see examples there for more information
def createScene(rootNode):
        accordianZPos = 2
        boxZPos= 7
        rootNode.createObject('CollisionPipeline',depth='1',draw='0',verbose='0')
        rootNode.createObject('BruteForceDetection',name='N2')
        rootNode.createObject('MinProximityIntersection', alarmDistance='0.2',contactDistance='0.1',name='Proximity')
        rootNode.createObject('RuleBasedContactManager', name="Response", response="FrictionContact", rules="1 * FrictionContact?mu=0.3" )
        rootNode.createObject('CollisionGroup',name='Group')
        rootNode.createObject('RequiredPlugin', pluginName='SoftRobots')
        rootNode.createObject('RequiredPlugin', pluginName='SofaMiscCollision')
        rootNode.createObject('VisualStyle', displayFlags="showVisualModels hideBehaviorModels showCollisionModels \
                                hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe")

        rootNode.createObject('FreeMotionAnimationLoop')

        rootNode.createObject('GenericConstraintSolver', maxIterations="70", tolerance="1e-3")

        rootNode.createObject('BackgroundSetting', color='0 0.168627 0.211765')
        rootNode.findData('gravity').value="0 -981.0 0"
        rootNode.findData('dt').value=0.01
        rootNode.createObject('PythonScriptController', filename="springy_controller.py", classname="controller")

	rootNode.createObject('RequiredPlugin', pluginName='UserInteraction')
	rootNode.createObject('RequiredPlugin', pluginName='SofaLCM')
	rootNode.createObject('LCMController',name='controller')
        ##########################################
        # Floor                                  #
        ##########################################
        floor = rootNode.createChild('Floor')
        floor.createObject('MeshTopology',name='loader',filename=path+'floor.obj',rotation='0 0 0')
        floor.createObject('MechanicalObject', name='floor mech obj')
        floor.createObject('Triangle', simulated='0',moving='0')
        floor.createObject('Line',simulated='0',moving='0')
        floor.createObject('Point',simulated='0',moving='0')

        ##########################################
        # sphere                                 #
        ##########################################
        box = rootNode.createChild('Box')
        box.createObject('EulerImplicit', firstOrder='0', rayleighStiffness="0.2", rayleighMass="0.2",vdamping='1')
        box.createObject('SparseLDLSolver')
        box.createObject('MeshVTKLoader',name='loader',filename=path+'sphere.vtk')
        box.createObject('TetrahedronSetTopologyContainer', src='@loader')
        box.createObject('TetrahedronSetTopologyModifier')
        box.createObject('TetrahedronSetTopologyAlgorithms', template='Vec3d')
        box.createObject('TetrahedronSetGeometryAlgorithms', template='Vec3d')
        box.createObject('MechanicalObject',name='box',template='Vec3d',dz=boxZPos,dy='2')
        box.createObject('UniformMass',totalMass='.1')
        box.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.3',  youngModulus='5000')
        box.createObject('LinearSolverConstraintCorrection')
        box.createObject('Triangle')
        box.createObject('Point')
        box.createObject('Line')


        ##########################################
        # FEM Model                              #
        ##########################################
        accordion = rootNode.createChild('accordion')
        accordion.createObject('EulerImplicit', firstOrder='0', rayleighStiffness="0.2", rayleighMass="0.2",vdamping='2')
        accordion.createObject('SparseLDLSolver')

        accordion.createObject('MeshVTKLoader', name='loader', filename=path+'Springy.vtk', rotation="0 0 0")
        accordion.createObject('TetrahedronSetTopologyContainer', src='@loader')
        accordion.createObject('TetrahedronSetTopologyModifier')
        accordion.createObject('TetrahedronSetTopologyAlgorithms', template='Vec3d')
        accordion.createObject('TetrahedronSetGeometryAlgorithms', template='Vec3d')

        accordion.createObject('MechanicalObject', name='tetras', template='Vec3d',dy=accordianZPos)
        
        accordion.createObject('UniformMass', totalMass='.1')
        accordion.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.3',  youngModulus='1500')

        accordion.createObject('BoxROI', name='ROI1', box='-2 -2 0 2 2 0.5', drawBoxes='true')
        accordion.createObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness='1e12')

        accordion.createObject('LinearSolverConstraintCorrection')
        accordion.createObject('TriangleModel')
        accordion.createObject('Point')
        accordion.createObject('Line')
        
        ##########################################
        # Pressure                               #
        ##########################################
	# a child of the main node is created to simulate the interal surface pressure
        cavity = accordion.createChild('cavity')

        cavity.createObject('MeshSTLLoader', name='loader', filename=path+'Springy_Cavity.stl')
        cavity.createObject('Mesh', src='@loader', name='topo')
        cavity.createObject('MechanicalObject', name='cavity',dy=accordianZPos)

        cavity.createObject('SurfacePressureConstraint', template='Vec3d', name="pressure",
		    triangles='@topo.triangles',
		    valueType="1",
		    value="3")

	#this mapping will use the center of each element to appropriately map forces between the triangular mesh of the cavity and the tetrahedral mesh of the body
        cavity.createObject('BarycentricMapping', name='mapping',  mapForces='false', mapMasses='false')

        # ##########################################
        # # Visualization                          #
        # ##########################################
        accordionVisu = accordion.createChild('visu')
        accordionVisu.createObject('MeshSTLLoader', filename=path+"Springy.stl", name="loader")
        accordionVisu.createObject('OglModel', src="@loader", color="0.4 0.4 0.4 0.5",dy=accordianZPos)
        accordionVisu.createObject('BarycentricMapping')



        return rootNode
