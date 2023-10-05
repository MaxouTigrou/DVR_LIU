#!/usr/bin/env python
from os import system
from GuiTextArea import GuiTextArea 
from RouterPacket import RouterPacket
from F import F
from copy import deepcopy

from RouterSimulator import RouterSimulator

import numpy as np

class RouterNode:
    
    def __init__(self, ID, sim, costs):
        self.myID = ID
        self.sim = sim
        self.myGUI = GuiTextArea(f"  Output window for Router #{ID}  ")
        self.costs = np.array(costs)
        self.routercosts = np.full(
            (self.sim.NUM_NODES, self.sim.NUM_NODES),
            self.sim.INFINITY,
            dtype=int,
        )
        self.routes = np.arange(self.sim.NUM_NODES)
        self.table = np.zeros(
            (self.sim.NUM_NODES, self.sim.NUM_NODES), dtype=int
        )
        
        np.copyto(self.costs, costs)
        np.copyto(self.routercosts[self.myID], costs)
        self.printDistanceTable()
        
        for i in range(self.sim.NUM_NODES):
            pk = RouterPacket(self.myID, i, costs)
            self.sendUpdate(pk)

    def recalcRoutes(self):
        bUpdated = False
        for to_node_x in range(self.sim.NUM_NODES):
            if to_node_x == self.myID:
                continue

            min_cost_to_x = self.sim.INFINITY
            via_neighbor_y = self.routes[to_node_x]

            for neighbor_y in range(self.sim.NUM_NODES):
                if (
                    neighbor_y == self.myID
                    or self.costs[neighbor_y] == self.sim.INFINITY
                ):
                    continue

                if (
                    self.routercosts[neighbor_y][to_node_x]
                    + self.costs[neighbor_y]
                    < min_cost_to_x
                ):
                    min_cost_to_x = (
                        self.routercosts[neighbor_y][to_node_x]
                        + self.costs[neighbor_y]
                    )
                    via_neighbor_y = neighbor_y

            if min_cost_to_x != self.routercosts[self.myID][to_node_x]:
                self.routercosts[self.myID][to_node_x] = min_cost_to_x
                self.routes[to_node_x] = via_neighbor_y
                bUpdated = True

        return bUpdated

    def sendUpdateToNeighbors(self):

        for nb_y in range(self.sim.NUM_NODES):
            if nb_y == self.myID :
                continue
                
            pk = RouterPacket(self.myID, nb_y, self.routercosts[self.myID])
                
            if self.sim.POISONREVERSE:
                for x in range(self.sim.NUM_NODES):
                    # Test if a router is used to go through another router
                    if self.routes[x] == nb_y and x != nb_y:
                        pk.mincost[x] = self.sim.INFINITY
                self.myGUI.println(f"Packets from {self.myID} to {nb_y} send costs = {pk.mincost}")
            self.sendUpdate(pk)


    def recvUpdate(self, pkt):
        bUpdated = False
        self.routercosts[pkt.sourceid] = pkt.mincost
        bUpdated = self.recalcRoutes()
        self.printDistanceTable()
        
        if bUpdated:
            self.sendUpdateToNeighbors()

    def sendUpdate(self, pkt):
        if pkt.sourceid != pkt.destid and self.costs[pkt.destid] != self.sim.INFINITY:
            self.sim.toLayer2(pkt)

    def printDistanceTable(self):
        self.myGUI.println(
            f"Current routercosts for {self.myID} at time {self.sim.getClocktime()}"
        )

        self.myGUI.println()
        self.myGUI.println("DistanceTable")
        self.myGUI.print("    dst  |")

        for i in range(self.sim.NUM_NODES):
            self.myGUI.print(f"      {i}")

        self.myGUI.println()
        self.myGUI.println("---------------------------------")

        for j in range(self.sim.NUM_NODES):
            self.myGUI.print(f"nbr {j:5} |")

            for i in range(self.sim.NUM_NODES):
                self.myGUI.print(f"{self.routercosts[j][i]:5}")

            self.myGUI.println()

        self.myGUI.println()
        self.myGUI.println("Our Distance Vector And Routes")
        self.myGUI.print("  dst    |")

        for i in range(self.sim.NUM_NODES):
            self.myGUI.print(f"{i:5}")

        self.myGUI.println()
        self.myGUI.println("---------------------------------")
        self.myGUI.print("  cost   |")

        for i in range(self.sim.NUM_NODES):
            self.myGUI.print(f"{self.routercosts[self.myID][i]:5}")

        self.myGUI.println()
        self.myGUI.print("  route  |")

        for i in range(self.sim.NUM_NODES):
            if self.routes[i] == 999:
                self.myGUI.print(f"-    ")
            else:
                self.myGUI.print(f"{self.routes[i]:5}")

        self.myGUI.println()

    def updateLinkCost(self, dest, newcost):
        self.costs[dest] = newcost

        bUpdated = self.recalcRoutes()

        self.printDistanceTable()

        if bUpdated:
            self.sendUpdateToNeighbors()




        
