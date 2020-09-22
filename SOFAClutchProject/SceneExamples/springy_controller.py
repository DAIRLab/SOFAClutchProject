#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa
import math

class controller(Sofa.PythonScriptController):





    def initGraph(self, node):

            self.node = node
            self.accordNode=self.node.getChild('accordion')
            self.pressureConstraintNode = self.accordNode.getChild('cavity')

    def onKeyPressed(self,c):
            self.dt = self.node.findData('dt').value

            self.MecaObject=self.accordNode.getObject('tetras')

            self.pressureConstraint = self.pressureConstraintNode.getObject('pressure')
            print('key pressed')
            if (c == "-"):
                pressureValue = self.pressureConstraint.findData('value').value[0][0] - 0.05
                if pressureValue < 0:
                    pressureValue = 0
                self.pressureConstraint.findData('value').value = str(pressureValue)

            elif (c == "+"):
                pressureValue = self.pressureConstraint.findData('value').value[0][0] + 0.05
                if pressureValue > 100:
                    pressureValue = 100
                self.pressureConstraint.findData('value').value = str(pressureValue)
            elif(c=="1"):
                stiffness = self.accordNode.findData('youngModulus').value[0][0] - 50
                if stiffness < 50:
                    stiffness = 50
                self.accordNode.findData('youngModulus').value[0][0] = stiffness
            elif(c=="2"):
                stiffness = self.accordNode.findData('youngModulus').value[0][0] + 50
                if stiffness > 1000:
                    stiffness = 1000
                self.accordNode.findData('youngModulus').value[0][0] = stiffness


            
