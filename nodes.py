import numpy as np
import utilits as ut
import math
import operator


class node:
    def __init__(self, index):
        self.neighbours = []
        self.heuristic = 99999
        self.gcost = 99999
        self.hash = index

    def print(self):
        print(self.hash)

    def getNeigh(self):
        return self.neighbours

    def printneigh(self):
        for i in self.neighbours:
            print(i.hash)

class spnode(node):
    def __init__(self, state, gm, gcost, prestate, goal = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15] ):
        self.state = state
        self.goal = goal
        self.zeroPosi = 0
        #generated method
        for i in range(16):
            if self.state[i] == 0:
                self.zeroPosi = i
                break
        self.geneMethod = gm
        self.prestate = prestate
        #only get neighbour when we want
        #self.getHash(self.state)
        #only get hash when we want

        # things in path are states.
        #only used in A-star, record the path to get current g-cost
        self.heuristic = self.getHeuristic(self.state, goal)
        self.gcost = gcost
        # should be assigned in algorithm, start node is 0
        self.nextGcost = gcost+1
        # for generated node

    def getHeuristic(self, state, goal = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]):
        heur = 0
        for i in range(16):
            tile = state[i]
            currposi = i
            targetposi = 0

            if state[i] == 0: continue
            #we don't take the blank to calculate heuristic
            for j in goal:
                if j == tile:break
                targetposi = targetposi + 1

            (currx, curry) = np.divmod(currposi,4)#行，列，从0开始, target和current似乎反了，但是无所谓
            (targx, targy) = np.divmod(targetposi,4) #these should be replaced by constant
            heur = heur + abs(currx-targx) + abs(curry-targy)
        return heur

    def update(self, other):
        #everything for update gcost
        self.geneMethod = other.geneMethod
        self.prestate = other.prestate
        self.gcost = other.gcost
        self.nextGcost = self.gcost + 1

    def print(self):
        for i in range(4):
            for j in range(4):
                print(str(self.state[j + 4 * i]) + " ", end="")
            print("\n")
        print("----------------------")

    def switch_left(self):
        ##move 0 to left
        mod_0posi_4 = divmod(self.zeroPosi, 4)[1]
        if mod_0posi_4 == 0:
            #print("can't move left")
            return -1
        else:
            i = self.zeroPosi
            newstate = []
            for j in self.state:
                newstate.append(j)
            (newstate[i], newstate[i - 1]) = ut.swap(newstate[i], newstate[i - 1])
            generatedNode = spnode(newstate, 'left', self.nextGcost, self.state, self.goal)
            #generatedNode.print()
            return generatedNode

    def switch_right(self):
        ##move 0 to right
        mod_0posi_4 = divmod(self.zeroPosi, 4)[1]
        if mod_0posi_4 == 3:
            #print("can't move right")
            return -1
        else:
            i = self.zeroPosi
            newstate = []
            for j in self.state:
                newstate.append(j)
            (newstate[i], newstate[i + 1]) = ut.swap(newstate[i], newstate[i + 1])
            generatedNode = spnode(newstate, 'right', self.nextGcost, self.state, self.goal)
            #generatedNode.print()
            return generatedNode

    def switch_up(self):
        ##move 0 to up
        if self.zeroPosi <= 3:
            #print("can't move up")
            return -1
        else:
            i = self.zeroPosi
            newstate = []
            for j in self.state:
                newstate.append(j)
            (newstate[i], newstate[i - 4]) = ut.swap(newstate[i], newstate[i - 4])
            generatedNode = spnode(newstate, 'up', self.nextGcost, self.state, self.goal)
            #generatedNode.print()
            return generatedNode

    def switch_down(self):
        ##move 0 to up
        if self.zeroPosi >= 12:
            #print("can't move down")
            return -1
        else:
            i = self.zeroPosi
            newstate = []
            for j in self.state:
                newstate.append(j)
            (newstate[i], newstate[i + 4]) = ut.swap(newstate[i], newstate[i + 4])
            generatedNode = spnode(newstate, 'down', self.nextGcost, self.state, self.goal)
            #generatedNode.print()
            return generatedNode

    def getNeigh(self):
        neighbours = []

        if self.geneMethod == 'right':
            zero_left = -1
        else:
            zero_left = self.switch_left()
        if ut.isNotNum(zero_left):
            neighbours.append(zero_left)

        if self.geneMethod == 'left':
            zero_right = -1
        else:
            zero_right = self.switch_right()
        if ut.isNotNum(zero_right):
            neighbours.append(zero_right)

        if self.geneMethod == 'down':
            zero_up = -1
        else:
            zero_up = self.switch_up()
        if ut.isNotNum(zero_up):
            neighbours.append(zero_up)

        if self.geneMethod == 'up':
            zero_down = -1
        else:
            zero_down = self.switch_down()
        if ut.isNotNum(zero_down):
            neighbours.append(zero_down)

        #print("end")
        return neighbours

class voxelnode(node):
    def __init__(self, coordinate, goal, prestate, gcost, vomap):
        self.state = coordinate
        self.goal = goal
        #gm should be a list indicate how three dimension changed
        self.prestate = prestate
        #for tracking the result
        self.heuristic = self.getHeuristic()
        self.gcost = gcost
        self.vomap = vomap
        # for generated node
        self.geneMethod = 0
        #never used

    def print(self):
        #for i in self.state:
        print(self.state)

    def getHeuristic(self):
        coodChange = [0,0,0]
        for i in range(3):
            coodChange[i] = abs(self.state[i]-self.goal[i])
        heuristic = (math.sqrt(3) - math.sqrt(2)) * (np.min(coodChange)) + (math.sqrt(2) - 1) * (
            np.median(coodChange)) + (np.max(coodChange))
        return heuristic

    def update(self, other):
        #everything for update gcost, actually everything that can change
        self.geneMethod = other.geneMethod
        self.prestate = other.prestate
        self.gcost = other.gcost
        self.nextGcost = self.gcost + 1

    def checkstates(self, state, vomap):
        if (state[0] > vomap.mapinfo[0] or state[0] < 0) or (state[1] > vomap.mapinfo[1] or state[1] < 0) or (state[2] > vomap.mapinfo[2] or state[2] < 0):
            return True

        for i in vomap.blocknode:
            if operator.eq(state, i):
                return True
        return False

    def getNeigh(self):
        #this is not an efficient way
        neighbours = []
        for i in range(3):
            #x
            for j in range(3):
                #y
                for k in range(3):
                    #z
                    if i == 1 and j == 1 and k == 1:
                        # x = i-1, y = j-1, z = k-1
                        continue
                    x = i-1
                    y = j-1
                    z = k-1
                    #these refer to the change of each direction
                    checkx = False
                    checky = False
                    checkz = False
                    if x != 0:
                        checkstate = [self.state[0]+x, self.state[1], self.state[2]]
                        if self.checkstates(checkstate, self.vomap): continue
                        else: checkx = True
                    if y != 0:
                        checkstate = [self.state[0], self.state[1]+y, self.state[2]]
                        if self.checkstates(checkstate, self.vomap): continue
                        else: checky = True
                    if z != 0:
                        checkstate = [self.state[0], self.state[1], self.state[2]+z]
                        if self.checkstates(checkstate, self.vomap): continue
                        else: checkz = True

                    if checkx and checky:
                        checkstate = [self.state[0]+x, self.state[1]+y, self.state[2]]
                        if self.checkstates(checkstate, self.vomap): continue
                    if checkx and checkz:
                        checkstate = [self.state[0]+x, self.state[1], self.state[2]+z]
                        if self.checkstates(checkstate, self.vomap): continue
                    if checkz and checky:
                        checkstate = [self.state[0], self.state[1]+y, self.state[2]+z]
                        if self.checkstates(checkstate, self.vomap): continue

                    if checkx and checkz and checky:
                        checkstate = [self.state[0]+x, self.state[1]+y, self.state[2]+z]
                        if self.checkstates(checkstate, self.vomap): continue

                    newstate = [self.state[0]+x, self.state[1]+y, self.state[2]+z]
                    #should avoid re computing this
                    nextgcost = math.sqrt(x**2+y**2+z**2)
                    newvonode = voxelnode(newstate, self.goal, self.state, self.gcost+nextgcost, self.vomap)
                    neighbours.append(newvonode)

        return neighbours