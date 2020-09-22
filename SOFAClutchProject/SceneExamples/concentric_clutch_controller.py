
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa
import math
import numpy as np

class controller(Sofa.PythonScriptController):

    def bwdInitGraph(self, node):

        self.node = node
        self.shellNode=self.node.getChild('OuterMesh')

        self.MecaObject1=self.shellNode.getObject('outer')

        self.pressureConstraint = self.shellNode.getObject('pressureConstraint')

        

        self.clutch = self.shellNode.getObject('Clutch')

        self.initialPositions = np.array(self.shellNode.getObject('outer').rest_position)

        self.triangles = np.array(self.shellNode.getObject('loader').triangles)
        self.numTriangles = self.triangles.shape[0]

        self.ring1 = self.getRing(0,.3,-1,0)
        self.ring2 = self.getRing(.3,.7,-1,0)
        self.ring3 = self.getRing(.7,1,-1,0)

        self.ring4 = self.getRing(0,.3,0,1)
        self.ring5 = self.getRing(.3,.7,0,1)
        self.ring6 = self.getRing(.7,1,0,1)
	print len(self.ring1)
        self.all = range(0,self.numTriangles)

        initialAxialClutch =  int(self.clutch.findData('AxialClutchActivation').value[0][0])
        initialTransverseClutch = int(self.clutch.findData('TransverseClutchActivation').value[0][0])
        initialPoissons = self.clutch.findData('poissonRatio').value[0][0]
        self.AxialClutch = np.ones((self.numTriangles,1)).astype(int).flatten() *initialAxialClutch
        self.TransverseClutch = np.ones((self.numTriangles,1)).astype(int).flatten() *initialTransverseClutch 
        self.PoissonsRatio = np.ones((self.numTriangles,1)).flatten() *initialPoissons 


        

        self.clutch.findData('AxialClutchActivation').value = self.constructInputArray(self.AxialClutch)
        self.clutch.findData('TransverseClutchActivation').value = self.constructInputArray(self.TransverseClutch)
        self.clutch.findData('poissonRatio').value = self.constructInputArray(self.PoissonsRatio)
        
        self.clutch.reinit()
        
        # print self.ring2
        # print self.ring3



            
        

    def onKeyPressed(self,c):
    
        print str(c) + " pressed"
        if (c == "+"):
            pressureValue = self.pressureConstraint.findData('value').value[0][0] + 0.01
            #if pressureValue > 1.5:
                #pressureValue = 1.5
            self.pressureConstraint.findData('value').value = str(pressureValue)
            self.pressureConstraint.reinit()

        if (c == "_"):

            pressureValue = self.pressureConstraint.findData('value').value[0][0] - 0.01
            #if pressureValue < -1.5:
                #pressureValue = -1.5
            self.pressureConstraint.findData('value').value = str(pressureValue)
            self.pressureConstraint.reinit()
        

        if (c=="1"):
            self.changeAxialClutch(self.ring1)
            


        if (c=="2"):
            self.changeAxialClutch(self.ring4)

        if (c=="3"):
            self.changeTransverseClutch(self.ring1)

        if (c=="4"):
            self.changeTransverseClutch(self.ring4)

        if (c=="5"):
            self.changeAxialClutch(self.ring2)

        if (c=="6"):
            self.changeAxialClutch(self.ring5)

        if (c=="7"):
            self.changeTransverseClutch(self.ring2)

        if (c=="8"):
            self.changeTransverseClutch(self.ring5)

        if (c=="9"):
            self.changeAxialClutch(self.ring3)

        if (c=="0"):
            self.changeAxialClutch(self.ring6)

        if (c=="-"):
            self.changeTransverseClutch(self.ring3)

        if (c=="="):
            self.changeTransverseClutch(self.ring6)

        if(c=="`"):
            print 'reset'
            self.clutch.findData('AxialClutchActivation').value = self.constructInputArray(self.AxialClutch)
            self.clutch.findData('TransverseClutchActivation').value = self.constructInputArray(self.TransverseClutch)



    def getRing(self, min_dist,max_dist, x_min = -10000,x_max = 10000):
        ring = []
        for i,triangle in enumerate(self.triangles):
            center = self.getBarycenter(triangle)
            
            dist = np.linalg.norm(center[[0,2]])

            if(dist > min_dist and dist<max_dist and center[0] < x_max and center[0] > x_min):
                ring.append(i)
            # print [str(self.initialPositions[triangle]),str(self.getBarycenter(triangle))]
            # if((self.initialPositions[triangle][:,2] < max_dist) and np.any(self.initialPositions[triangle][:,2] > min_dist)):
            #     ring.append(i)

        print 'ring made with: ' + str(ring)

        return ring
            

    def getBarycenter(self,triangle):       
        return np.mean(self.initialPositions[triangle],axis=0)

    def constructInputArray(self, array):
        out = ""
        for data in array:
            out = out + str(data) + " "
        return str(out)

    def changeAxialClutch(self, index ):
        self.AxialClutch[index] = np.logical_not(self.AxialClutch[index]).astype(int)

        self.clutch.findData('AxialClutchActivation').value = self.constructInputArray(self.AxialClutch)
        self.clutch.reinit()




    def changeTransverseClutch(self, index):
        self.TransverseClutch[index] = np.logical_not(self.TransverseClutch[index]).astype(int)
        self.clutch.findData('TransverseClutchActivation').value = self.constructInputArray(self.TransverseClutch)
        self.clutch.reinit()




