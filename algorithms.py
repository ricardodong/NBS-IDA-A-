import numpy as np
import utilits as ut
import nodes
import maps
import math
import time
import heapq
import operator

class DFS_limit:

    def __init__(self):
        self.path = []
        self.safe = False
        #indicate whether we have reach the limitation of level,
        # if it is, then it means we cannot find result because of limitation,
        # else because of no goal find
        self.respath = []
        self.expandednode = 0

    def GetPath(self, start, goal, level, lelimit, starttime):

        start.setNeigh()
        self.expandednode = self.expandednode + 1
        neigh = start.neighbours
        self.path.append(start)
        #print(start.hash)
        if start.hash == goal.hash:
            print("find!")
            self.respath = self.path
            endtime = time.time()
            print("time used:"+str(endtime-starttime))
            return 1
        elif level < lelimit:
            nextlevel = level+1
            for i in neigh:
                currtime = time.time()
                if currtime - starttime >= 60:
                    self.safe = False
                    return -1
                if self.isInPath(i):
                    #print("same")
                    continue
                j = self.GetPath(i, goal, nextlevel, lelimit, starttime)
                if j == 1:
                    return 1
            self.path.remove(start)
            return -1
        else:
            self.safe = True
            return -1

    def isInPath(self, neighNode):
        for i in self.path:
            if neighNode.hash == i.hash:
                return True
        return False

class DFID:
    def __init__(self):
        self.path = []
        self.respath = []
        self.findlevel = 99999
        self.expandednode = 0

    def search(self,start, goal):
        lelimit = 0
        starttime = time.time()
        while True:
            newdfs = DFS_limit()
            found  = newdfs.GetPath(start, goal, 0, lelimit, starttime)
            self.expandednode = self.expandednode + newdfs.expandednode
            if found == 1:
                self.respath = newdfs.respath
                self.findlevel = lelimit
                return 1
            else:
                if newdfs.safe == False:
                    print("generated nodes:" +str(self.expandednode))
                    break
                else:
                    lelimit = lelimit + 1
                    print(lelimit)

        return -1

class priqueNode:
    #它有arrayinfo的index，然后用index对应的array元素进行比较, A STAR
    def __init__(self, index, array):
        self.index = index
        self.array = array
        #will not change
        self.gcost = 0
        self.heuristic = 0
        #change dynamicially

    def __eq__(self, other):
        #for f-cost, not for state
        self.heuristic = self.array[self.index].heuristic
        self.gcost = self.array[self.index].gcost
        other.heuristic = self.array[other.index].heuristic
        other.gcost = self.array[other.index].gcost
        if self.heuristic == other.heuristic and self.gcost == other.gcost :
            return True
        else:
            return False

    def __lt__(self, other):
        # less then, here means less fcost or less heuristic
        self.heuristic = self.array[self.index].heuristic
        self.gcost = self.array[self.index].gcost
        other.heuristic = self.array[other.index].heuristic
        other.gcost = self.array[other.index].gcost

        selfFcost = self.heuristic+self.gcost
        otherFcost = other.heuristic + other.gcost
        if selfFcost < otherFcost or (selfFcost == otherFcost and self.gcost > other.gcost):
            #choose the less fcost, or the bigger gcost if same fcost (less heuristic)
            return True
        else:
            return False

    def __gt__(self, other):
        # great then, here means bigger fcost or bigger heuristic
        self.heuristic = self.array[self.index].heuristic
        self.gcost = self.array[self.index].gcost
        other.heuristic = self.array[other.index].heuristic
        other.gcost = self.array[other.index].gcost

        selfFcost = self.heuristic+self.gcost
        otherFcost = other.heuristic + other.gcost
        if selfFcost > otherFcost or (selfFcost == otherFcost and self.gcost < other.gcost):
            #choose the less fcost, or the bigger gcost if same fcost (less heuristic)
            return True
        else:
            return False

    def __le__(self, other):
        return (self.__lt__(other) or self.__eq__(other))

    def __ge__(self, other):
        return (self.__gt__(other) or self.__eq__(other))

class A_star:
    def __init__(self):
        self.path = []
        self.updatednode = 0
        self.hashTaForAllnodes = {}
        self.arrayForInfo = []
        self.openlist = []

    def showlists(self):
        print("all nodes number: "+str(len(self.hashTaForAllnodes)))
        print("open len: " + str(len(self.openlist)))
        print("updated node: "+ str(self.updatednode))

    def generatePath(self, node):
        newnode = node
        while ut.isNotNum(newnode.prestate):
            self.path.append(newnode)
            nodeindex = self.hashTaForAllnodes[tuple(newnode.prestate)]
            newnode = self.arrayForInfo[nodeindex]

    def search(self, start, goal):

        #get time
        starttime = time.time()

        #problem is that goal is fixed
        if start.heuristic == 0:
            print("finish time = 0")
            return 1

        #init of all nodes and open list
        self.hashTaForAllnodes[tuple(start.state)] = 0
        self.arrayForInfo.append(start)
        nodeaddtoclose = start
        indexOfarray = 1
        #first is start and added

        #interative adding
        while True:
            #generate openlist (update the f-cost)
            #fast???
            ineigh = nodeaddtoclose.getNeigh()
            openlistchange = False
            #print(len(ineigh))
            for j in ineigh:
                # should update gcost here.
                inOporClose = ut.inHashTable(self.hashTaForAllnodes, tuple(j.state))
                if inOporClose[0]:
                    # update path and gcost
                    origNodeindex = inOporClose[1]
                    originalNode = self.arrayForInfo[origNodeindex]
                    if j.gcost < originalNode.gcost:
                        self.updatednode = self.updatednode + 1
                        self.arrayForInfo[origNodeindex].update(j)
                        openlistchange = True
                else:
                    self.hashTaForAllnodes[tuple(j.state)] = indexOfarray
                    self.arrayForInfo.append(j)
                    openlistNode = priqueNode(indexOfarray, self.arrayForInfo)
                    heapq.heappush(self.openlist, openlistNode)
                    indexOfarray = indexOfarray + 1
                    #add to openlist

            #should not heapfiy here
            if openlistchange:
                heapq.heapify(self.openlist)

            #select one from openlist to close list
            minnodeindex = heapq.heappop(self.openlist)
            minnodeindex = minnodeindex.index
            minnode = self.arrayForInfo[minnodeindex]

            #if the selected is the goal, then end search, else add it to close list
            if minnode.heuristic == 0:
                #found goal
                endtime = time.time()
                print("finish time is "+str(endtime-starttime))
                self.showlists()
                print(minnode.gcost)
                self.generatePath(minnode)
                return 1
            else:
                nodeaddtoclose = minnode
                endtime = time.time()
                if endtime-starttime >= 60 :
                    print("finish time is " + str(endtime - starttime))
                    self.showlists()
                    break



class IDA_limit:

    def __init__(self):
        self.path = []
        self.safe = False
        #indicate whether we have reach the limitation of level,
        # if it is, then it means we cannot find result because of limitation,
        # else because of no goal find
        self.respath = []
        self.expandednode = 0
        self.nextfcostlimit = 999999

    def GetPath(self, start, goal, fcostlimit):
        startfcost = start.gcost + start.heuristic
        #print(start.hash)
        if startfcost <= fcostlimit:
            if start.heuristic == 0:
                # goal is not used
                print("find!")
                self.respath = self.path
                return 1
            self.expandednode = self.expandednode + 1
            #self.path.append(start)
            neigh = start.getNeigh()
            for i in neigh:
                #if self.isInPath(i): continue
                j = self.GetPath(i, goal, fcostlimit)
                if j == 1:
                    return 1
            #self.path.remove(start)
            return -1
        else:
            self.safe = True
            if self.nextfcostlimit > startfcost:
                self.nextfcostlimit = startfcost
            return -1

    def isInPath(self, neighNode):
        for i in self.path:
            if operator.eq(neighNode.state, i.state):
                return True
        return False

class IDA:
    def __init__(self):
        self.path = []
        self.findgcost = 99999
        self.expandednode = 0

    def search(self, start, goal):
        gcostlimit = start.heuristic + start.gcost
        starttime  = time.time()
        while True:
            newdfs = IDA_limit()
            found  = newdfs.GetPath(start, goal, gcostlimit)
            self.expandednode = self.expandednode + newdfs.expandednode
            if found == 1:
                self.path = newdfs.respath
                self.findgcost = gcostlimit
                endtime = time.time()
                #print("result path: " + self.respath)
                print("find gcost: " + str(self.findgcost))
                print("generated node: " + str(self.expandednode))
                print("finish time: " + str(endtime - starttime))
                return 1
            else:
                if newdfs.safe == False:
                    print("cannot find goal..")
                    print("generated nodes:" +str(self.expandednode))
                    return -1
                else:
                    gcostlimit = newdfs.nextfcostlimit
                    print(gcostlimit)
        return -1

class waitpriqueNode:
    #它有arrayinfo的index，然后用index对应的array元素进行比较, A STAR
    def __init__(self, index, array):
        self.index = index
        self.array = array
        #will not change
        self.gcost = 0
        self.heuristic = 0
        #change dynamicially

    def __eq__(self, other):
        #for f-cost, not for state
        self.heuristic = self.array[self.index].heuristic
        self.gcost = self.array[self.index].gcost
        other.heuristic = self.array[other.index].heuristic
        other.gcost = self.array[other.index].gcost
        selfFcost = self.heuristic + self.gcost
        otherFcost = other.heuristic + other.gcost
        if selfFcost == otherFcost:
            return True
        else:
            return False

    def __lt__(self, other):
        self.heuristic = self.array[self.index].heuristic
        self.gcost = self.array[self.index].gcost
        other.heuristic = self.array[other.index].heuristic
        other.gcost = self.array[other.index].gcost
        selfFcost = self.heuristic + self.gcost
        otherFcost = other.heuristic + other.gcost
        if selfFcost < otherFcost:
            return True
        else:
            return False

    def __gt__(self, other):
        self.heuristic = self.array[self.index].heuristic
        self.gcost = self.array[self.index].gcost
        other.heuristic = self.array[other.index].heuristic
        other.gcost = self.array[other.index].gcost
        selfFcost = self.heuristic + self.gcost
        otherFcost = other.heuristic + other.gcost
        if selfFcost > otherFcost :
            return True
        else:
            return False

    def __le__(self, other):
        return (self.__lt__(other) or self.__eq__(other))

    def __ge__(self, other):
        return (self.__gt__(other) or self.__eq__(other))

class readypriqueNode:
    #它有arrayinfo的index，然后用index对应的array元素进行比较, A STAR
    def __init__(self, index, array):
        self.index = index
        self.array = array
        #will not change
        #change dynamicially

    def __eq__(self, other):
        #for f-cost, not for state
        self.gcost = self.array[self.index].gcost
        other.gcost = self.array[other.index].gcost
        if self.gcost == other.gcost:
            return True
        else:
            return False

    def __lt__(self, other):
        self.gcost = self.array[self.index].gcost
        other.gcost = self.array[other.index].gcost
        if self.gcost < other.gcost:
            return True
        else:
            return False

    def __gt__(self, other):
        self.gcost = self.array[self.index].gcost
        other.gcost = self.array[other.index].gcost
        if self.gcost > other.gcost :
            return True
        else:
            return False

    def __le__(self, other):
        return (self.__lt__(other) or self.__eq__(other))

    def __ge__(self, other):
        return (self.__gt__(other) or self.__eq__(other))

class NBS:
    def __init__(self):
        self.path = []
        self.updatednode = 0

        self.hashfrontTa = {}
        self.frontArrayInfo = []
        self.hashbackTa = {}
        self.backArrayInfo = []

        self.frontWaitList = []
        self.frontReadyList = []
        self.backWaitList = []
        self.backReadyList = []

    def showlists(self):
        print("all nodes number: "+str(len(self.frontArrayInfo)+len(self.backArrayInfo)))
        print("front Waiting len: " + str(len(self.frontWaitList)))
        print("front ready len: " + str(len(self.frontReadyList)))
        print("back Waiting len: " + str(len(self.backWaitList)))
        print("back ready len: " + str(len(self.backReadyList)))
        print("updated node: "+ str(self.updatednode))

    def generatePath(self, node):
        newnode = node
        while ut.isNotNum(newnode.prestate):
            self.path.append(newnode)
            nodeindex = self.hashTa[tuple(newnode.prestate)]
            newnode = self.ArrayInfo[nodeindex]

    def search(self, start, goal):

        #problem is that goal is fixed
        if start.heuristic == 0:
            print("finish time = 0")
            return 1

        #get time
        starttime = time.time()

        #init of all nodes and open list
        self.hashfrontTa[tuple(start.state)] = 0
        self.frontArrayInfo.append(start)
        self.hashbackTa[tuple(goal.state)] = 0
        self.backArrayInfo.append(goal)
        nodeFrontClose = start
        nodeBackClose = goal
        indexArrayfront = 1
        indexArrayback = 1
        estimateCstar = max(start.heuristic, goal.heuristic)
        print(estimateCstar)
        #first is start and added

        while True:
            ##########################front##############################
            frontNeigh = nodeFrontClose.getNeigh()
            FrontOpenlistchange = False
            #print(len(ineigh))
            for j in frontNeigh:
                # should update gcost here.
                inOporClose = ut.inHashTable(self.hashfrontTa, tuple(j.state))
                if inOporClose[0]:
                    # update path and gcost
                    origNodeindex = inOporClose[1]
                    originalNode = self.frontArrayInfo[origNodeindex]
                    if j.gcost < originalNode.gcost:
                        self.updatednode = self.updatednode + 1
                        self.frontArrayInfo[origNodeindex].update(j)
                        FrontOpenlistchange = True
                else:
                    self.hashfrontTa[tuple(j.state)] = indexArrayfront
                    self.frontArrayInfo.append(j)
                    if j.gcost + j.heuristic < estimateCstar:
                        readylistNode = readypriqueNode(indexArrayfront, self.frontArrayInfo)
                        heapq.heappush(self.frontReadyList, readylistNode)
                    else:
                        waitlistNode = waitpriqueNode(indexArrayfront, self.frontArrayInfo)
                        heapq.heappush(self.frontWaitList, waitlistNode)
                    indexArrayfront = indexArrayfront + 1
                    #add to openlist

            ##########################back##############################
            backNeigh = nodeBackClose.getNeigh()
            backOpenlistchange = False
            # print(len(ineigh))
            for j in backNeigh:
                # should update gcost here.
                inOporClose = ut.inHashTable(self.hashbackTa, tuple(j.state))
                if inOporClose[0]:
                    # update path and gcost
                    origNodeindex = inOporClose[1]
                    originalNode = self.backArrayInfo[origNodeindex]
                    if j.gcost < originalNode.gcost:
                        self.updatednode = self.updatednode + 1
                        self.backArrayInfo[origNodeindex].update(j)
                        backOpenlistchange = True
                else:
                    self.hashbackTa[tuple(j.state)] = indexArrayback
                    self.backArrayInfo.append(j)
                    if j.gcost + j.heuristic < estimateCstar:
                        readylistNode = readypriqueNode(indexArrayback, self.backArrayInfo)
                        heapq.heappush(self.backReadyList, readylistNode)
                    else:
                        waitlistNode = waitpriqueNode(indexArrayback, self.backArrayInfo)
                        heapq.heappush(self.backWaitList, waitlistNode)
                    indexArrayback = indexArrayback + 1
                    # add to openlist

            # should not heapfiy here
            if backOpenlistchange or FrontOpenlistchange:
                heapq.heapify(self.backWaitList)
                heapq.heapify(self.backReadyList)
                heapq.heapify(self.frontWaitList)
                heapq.heapify(self.frontReadyList)

            #################pop wait to ready, select pair from ready, raise estimate c######################
            while True:
                if (not self.frontWaitList and not self.frontReadyList) or (not self.backWaitList and not self.backReadyList):
                    print("run out of node on OPEN!!! cannot find the goal")
                    return -1
                smallestG = 999999
                smallestFW = 999999
                smallestBW = 999999

                if self.frontReadyList and self.backReadyList:
                    frontReadyG = self.frontArrayInfo[self.frontReadyList[0].index].gcost
                    backReadyG = self.backArrayInfo[self.backReadyList[0].index].gcost
                    smallestG = frontReadyG + backReadyG
                    if smallestG <= estimateCstar:
                        frontExpandIndexNode = heapq.heappop(self.frontReadyList)
                        frontExpandIndex = frontExpandIndexNode.index
                        backExpandIndexNode = heapq.heappop(self.backReadyList)
                        backExpandIndex = backExpandIndexNode.index
                        break

                while True:
                    if self.frontWaitList:
                        currFroWaitFcost = self.frontArrayInfo[self.frontWaitList[0].index].heuristic + self.frontArrayInfo[self.frontWaitList[0].index].gcost
                    else:
                        break
                    if currFroWaitFcost <= estimateCstar:
                        popfromwait = heapq.heappop(self.frontWaitList)
                        addtoready  = readypriqueNode(popfromwait.index, self.frontArrayInfo)
                        heapq.heappush(self.frontReadyList, addtoready)
                    else:
                        smallestFW = currFroWaitFcost
                        break

                while True:
                    if self.backWaitList:
                        currBacWaitFcost = self.backArrayInfo[self.backWaitList[0].index].heuristic + self.backArrayInfo[self.backWaitList[0].index].gcost
                    else:
                        break
                    if currBacWaitFcost <= estimateCstar:
                        popfromwait = heapq.heappop(self.backWaitList)
                        addtoready = readypriqueNode(popfromwait.index, self.backArrayInfo)
                        heapq.heappush(self.backReadyList, addtoready)
                    else:
                        smallestBW = currBacWaitFcost
                        break

                if min(smallestG, smallestFW, smallestBW) != 999999:
                    estimateCstar = min(smallestG, smallestFW, smallestBW)
                print(smallestG)
                self.showlists()
                endtime = time.time()
                print("finish in :" + str(endtime - starttime))

            ###########termination and add to close#####################
            frontExpand = self.frontArrayInfo[frontExpandIndex]
            backExpand = self.backArrayInfo[backExpandIndex]
            if operator.eq(frontExpand.state, backExpand.state):
                endtime = time.time()
                print("find!!, gcost:" + str(frontExpand.gcost + backExpand.gcost))
                print("finish in :" + str(endtime - starttime))
                self.showlists()
                return 1
            frontExpandNeigh = frontExpand.getNeigh()
            for k in frontExpandNeigh:
                if operator.eq(k.state, backExpand.state):
                    endtime = time.time()
                    print("find!!, gcost:" + str(k.gcost+backExpand.gcost))
                    print("finish in :" + str(endtime- starttime))
                    self.showlists()
                    return 1
            nodeFrontClose = frontExpand
            nodeBackClose = backExpand
            #nodeFrontClose.print()
            #nodeBackClose.print()

if __name__ == '__main__':

    #newgraph = graph()

    #newsearch = DFS_limit()
    #find = newsearch.GetPath(newgraph.nodes[0],newgraph.nodes[100], 0, 10)
    #for i in newsearch.respath:
    #    print(i.hash, end=" ")

    #21
    #a = [12 ,8 ,14 ,6 ,11 ,4 ,7 ,0 ,5 ,1 ,10 ,15 ,3 ,13 ,9 ,2]
    #11
    #a = [5 ,9 ,13 ,14 ,6 ,3 ,7 ,12 ,10 ,8 ,4 ,0 ,15 ,2 ,11 ,1]
    #12
    #a = [14 ,1 ,9 ,6 ,4 ,8 ,12 ,5 ,7 ,2 ,3 ,0 ,10 ,11 ,13 ,15]
    #2
    #a = [13 ,5 ,4 ,10 ,9 ,12 ,8 ,14 ,2 ,3 ,7 ,1 ,0 ,15 ,11 ,6]
    #79
    #a = [0 ,1 ,9 ,7 ,11 ,13 ,5 ,3 ,14 ,12 ,4 ,2 ,8 ,6 ,10 ,15]
    # 9
    a = [3 ,14 ,9 ,11 ,5 ,4 ,8 ,2 ,13 ,12 ,6 ,7 ,10 ,1 ,15 ,0]
    firstnode = nodes.spnode(a,'none', 0, 0)
    #'none' refers to generated method of current stp node, first 0 refers to gcost, next 0 refers to previous node
    goalstate = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
    goalnode = nodes.spnode(goalstate,'none', 999999, 0)
    #goalnodeforNBS = nodes.spnode(goalstate, 'none', 0, 0, a)
    #a refers to goal state

    mapfile = open("warframe\Simple.3dmap")
    vomap = maps.voxelmap(mapfile)
    statefile = open("warframe-3dscen\Simple.3dmap.3dscen")
    #newvoxelmap = maps.voxelmap(mapfile)
    statefile.readline()
    statefile.readline()
    for i in range(20):
        statefile.readline()
    states = statefile.readline().split(" ")
    startstate = states[0:3]
    for i in range(3):
        startstate[i] = int(startstate[i])
    vogoalstate= states[3:6]
    for i in range(3):
        vogoalstate[i] = int(vogoalstate[i])
    bestres = states[6]
    bestres = float(bestres)
    ratio = states[7]
    ratio = float(ratio)

    vofirstnode = nodes.voxelnode(startstate, vogoalstate, 0, 0, vomap)
    vogoalnode = nodes.voxelnode(vogoalstate, startstate, 0, 0, vomap)

    newsearch = NBS()
    find = newsearch.search(vofirstnode,vogoalnode)
    #for i in newsearch.path:
    #    i.print()