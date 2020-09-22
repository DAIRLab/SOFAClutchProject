import Sofa

import os
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

def createScene(rootNode):

    rootNode.createObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields hideInteractionForceFields hideWireframe')
    rootNode.createObject('RequiredPlugin', name='SoftRobots')
    rootNode.createObject('FreeMotionAnimationLoop')
    rootNode.createObject('GenericConstraintSolver', maxIterations='100', tolerance = '0.0001')
    rootNode.createObject('PythonScriptController', filename="shell_LCM_controller.py", classname="controller")

    shell = rootNode.createChild('OuterMesh')
    shell.createObject('EulerImplicit', name='odesolver',vdamping=50)
    shell.createObject('ShewchukPCGLinearSolver', iterations='30', name='linearsolver', tolerance='1e-5', preconditioners='preconditioner', use_precond='true', update_step='1')
    shell.createObject('MeshObjLoader', name='loader', filename=path+'sphere.obj')
    shell.createObject('TriangleSetTopologyContainer', src='@loader', name='outer_container')
    shell.createObject('TriangleSetTopologyModifier')
    shell.createObject('TriangleSetTopologyAlgorithms', template='Vec3d')
    shell.createObject('TriangleSetGeometryAlgorithms', template='Vec3d')

    shell.createObject('MechanicalObject', name='state')
    shell.createObject('UniformMass', totalMass='0.5')


    shell.createObject('TriangularFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.3',  youngModulus='100')
    shell.createObject('SparseLDLSolver', name='preconditioner')
    shell.createObject('LinearSolverConstraintCorrection', solverName='preconditioner')
    shell.createObject('SurfacePressureConstraint', name='pressureConstraint', triangles='@outer_container.triangles', value='3', valueType="1")

    shell.createObject('BoxROI', name='boxROI', box='-1 -1 -1 1 -.8 1', drawBoxes='true', position="@outer.rest_position", triangles="@outer_container.triangles")
    shell.createObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness='1e12')
              

    return rootNode
