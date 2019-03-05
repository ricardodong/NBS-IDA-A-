import nodes


class voxelmap:
    def __init__(self,mapfile):
        mapinfo = mapfile.readline().split(" ")
        self.mapinfo = mapinfo[1:4]
        for i in range(3):
            self.mapinfo[i] = int(self.mapinfo[i])
        self.blocknode = []
        #store the hash value of the block nodes
        while True:
            blocknode = mapfile.readline()
            if not blocknode:
                break
            blocknode = blocknode.split(" ")
            for i in range(3):
                blocknode[i] = int(blocknode[i])
            self.blocknode.append(blocknode)


class graph:
    def __init__(self):
        ##for test, create a binary tree
        depth = 10
        bf = 2 ##branching factor
        ##initial the first node
        self.nodes = []
        firstnode  = nodes.node(0)
        self.nodes.append(firstnode)
        ##create tree
        k=1
        for i in range(depth):
            #indicate which level are we in
            numOfPCurrNodes = bf**i
            numOfPTotalNodes = len(self.nodes)
            for m in range(numOfPCurrNodes):
                #indicate the current level
                indexOfCurrNode = numOfPTotalNodes-(numOfPCurrNodes-m)
                for n in range(bf):
                    ##create bf 个 nodes
                    createdNode = nodes.node(k)
                    k=k+1
                    self.nodes[indexOfCurrNode].neighbours.append(createdNode)
                    createdNode.neighbours.append(self.nodes[indexOfCurrNode])
                    self.nodes.append(createdNode)
