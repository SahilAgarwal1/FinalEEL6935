"""

"""
import itertools
from ggsolver.models import Game, TSys
from ggsolver.interfaces.i_spot import SpotAutomaton
import numpy as np
import math
import pickle
class GridWorld(Game):
    def __init__(self, dim, A, B, C):
        """
        Parameters for the gridworld.
        :param dim: Dimensions of gridworld (row, col)
        :param A  : Position of first landmark to reach
        :param B  : Position of second landmark to reach
        """
        assert dim[0] > 0 and dim[1] > 0
        super(GridWorld, self).__init__(is_deterministic=True)
        self.dim = dim
        self.A = A
        self.B = B
        self.C = C
        print("A =", A)
        print("B =",B )
    def states(self):
        """
        Returns a list of states in gridworld.
        A state is represented as (tom.row, tom.col, tom.dir, jerry.row,
jerry.col).
        """
        rows = self.dim[0]
        cols = self.dim[1]
        return [
            (x,y)
            for x,y in itertools.product(range(cols), range(rows))
        ]
    def actions(self):
        """
        Return a list of actions. Each action is identified by a string label.
        In each round, both Tom and Jerry must select an action from ["N", "E",
"S", "W"].
        Thus, each action is represented as a tuple of
        """
        return ["N", "E", "S", "W", "NE", "NW", "SE", "SW"]
    def delta(self, state, inp):
        """
        Implement the transition function.
        :param state: A state from the list returned by states().
        :param inp: An action from the list returned by actions().
        :return: The next state, which is the result of applying the action `inp`
to `state`.
        """
        x, y = state
        # If jerry is caught or trapped, game end (no state change).
        next_x, next_y = self.apply_action(x,y, inp)
        return next_x, next_y
    def atoms(self):
        """
        Returns a list of atomic propositions. Each atomic proposition is a string.
        """
        return ["A", "B", "C"]
    def label(self, state):
        """
        Returns a list of atoms that are true in the `state`.
        :param state: A state from the list returned by states().
        :return: List of atoms.
        """
        if(state in self.A):
            return "A"
        if(state in self.B):
            return "B"
        if(state in self.C):
            return "C"
        return list()
    def apply_action(self, x,y,act):
        if act == "N":
            return (x, y+1) if 0 <= y + 1 < self.dim[0] else (x, y)
        elif act == "E":
            return (x+1, y) if 0 <= x + 1 < self.dim[1] else (x, y)
        elif act == "S":
            return (x, y-1) if 0 <= y - 1 < self.dim[0] else (x, y)
        elif act == "NE": # act == "W":
            return (x+1, y+1) if 0 <= x + 1 < self.dim[1] and  0 <= y + 1 < self.dim[0] else (x, y)
        elif act == "NW":  # act == "W":
            return (x - 1, y + 1) if 0 <= x - 1 < self.dim[1] and 0 <= y + 1 < self.dim[0] else (x, y)
        elif act == "SE":  # act == "W":
            return (x + 1, y - 1) if 0 <= x + 1 < self.dim[1] and 0 <= y - 1 < self.dim[0] else (x, y)
        else:  # act == "SW":
            return (x - 1, y - 1) if 0 <= x - 1 < self.dim[1] and 0 <= y - 1 < self.dim[0] else (x, y)





    def init_state(self):
        return [(0,0)]
class ProductTSysDFA(Game):
    def __init__(self, tsys, dfa):
        super(ProductTSysDFA, self).__init__(is_deterministic=True)
        self.tsys = tsys
        self.dfa = dfa
    def states(self):
        return tuple(itertools.product(self.tsys.states(), self.dfa.states()))
    def actions(self):
        return self.tsys.actions()
    def delta(self, state, act):
        s, q = state
        t = self.tsys.delta(s, act)
        label = list(set(self.tsys.label(t)))
        p = self.dfa.delta(q, label)
        return t, p
    def init_state(self):
        s0 = self.tsys.init_state()
        q0 = self.dfa.init_state()
        return tuple([tuple([0,0]),3])
    def final(self, state):
        return True if self.dfa.final(state[1]) != [] else False
        # final = [st for st in self.dfa.states() if self.dfa.final(st) == 0]
        # return list(itertools.product(self.tsys.states(), final))
class AStarAlgorithm:
    def __init__(self, game):
        self.game = game
    def algorithm(self):

        # open_list is a list of nodes which have been visited, but who's neighbors
        # haven't all been inspected, starts off with the start node
        # closed_list is a list of nodes which have been visited
        # and who's neighbors have been inspected
        start_node = self.game.init_state()
        open_list = [start_node]
        closed_list = []

        # g contains current distances from start_node to all other nodes
        # the default value (if it's not found in the map) is +infinity
        g = {}

        g[start_node] = 0

        # parents contains an adjacency map of all nodes
        parents = {}
        parents[start_node] = start_node

        # Compute levels for each of the automata states

        # Start with the final states
        states = self.game.states()

        # entries in level set are automata states!!
        level_sets = []
        level_sets.append(list(set([state[1] for state in self.game.states() if self.game.final(state)])))
        #level_sets = list(set([state[1] for state in self.game.states() if self.game.final(state)]))
        while True:
            # Attractor : For prod states with automata states not already in attractor, see if there is an action they can take to reach
            attr = {st[1] for st in [states for states in self.game.states() if states[1] not in level_sets[-1]]
                    if any(self.game.delta(st, act)[1] in level_sets[-1] for act in
                           self.game.actions())}
            level_sets.append(list(set.union(set(level_sets[-1]), attr)))
            if level_sets[-1] == level_sets[-2]:
                break

        print("Level Sets for automata states:")
        for i in range(len(level_sets)):
            print(level_sets[i])
        print("")
        # Any automata state not in the attractor will be a sink state
        rank = {}
        for q in self.game.dfa.states():
            rank.update({q : float("inf")})
        for q in self.game.dfa.states():
            for idx in range(len(level_sets)):
                if q in level_sets[idx]:
                    rank[q] = idx
                    break

        print("Level for automata states: ",rank)

        while len(open_list) > 0:
            n = None
            print("Open list: ",open_list)
            print("Closed list: ", closed_list)
            # find a node with the lowest value of f() - evaluation function
            for v in open_list:
                if n == None or g[v] + self.computeHeuristic(v, level_sets, rank) < g[n] + self.computeHeuristic(n, level_sets, rank):
                    n = v;

            if n == None:
                print('Path does not exist!')
                return None

            # if the current node is the stop_node
            # then we begin reconstructin the path from it to the start_node
            if self.isGoal(n):
                reconst_path = []

                while parents[n] != n:
                    reconst_path.append(n)
                    n = parents[n]

                reconst_path.append(start_node)

                reconst_path.reverse()

                print('Path found: {}'.format(reconst_path))
                with open("path.pkl", "wb") as fp:
                    pickle.dump(reconst_path, fp)
                return reconst_path

            # for all neighbors of the current node do
            for (m, weight) in self.getSuccessors(n, rank):
                # if the current node isn't in both open_list and closed_list
                # add it to open_list and note n as it's parent
                print("M:", m)
                print("weight:", weight)
                print(open_list)
                print(closed_list)
                if m not in open_list and m not in closed_list:
                    open_list.append(m)
                    parents[m] = n
                    g[m] = g[n] + weight

                # otherwise, check if it's quicker to first visit n, then m
                # and if it is, update parent data and g data
                # and if the node was in the closed_list, move it to open_list
                else:
                    if g[m] > g[n] + weight:
                        g[m] = g[n] + weight
                        parents[m] = n

                        if m in closed_list:
                            closed_list.remove(m)
                            if m in open_list:
                                open_list.remove(m)

            # remove n from the open_list, and add it to closed_list
            # because all of his neighbors were inspected
            open_list.remove(n)
            closed_list.append(n)

        print('Path does not exist!')
        return None

    def computeHeuristic(self, state, level_sets, rank):

        # If q is a sink state, return an estimated cost of infinity
        if self.isSink(state[1], rank):
            return float("inf")

        # If q is the final state, return zero, you are done
        if self.getLevel(state, rank) == 0:
            return 0

        curr_level_set = level_sets[self.getLevel(state, rank)]
        future_level_set = level_sets[self.getLevel(state, rank) - 1]

        q = state[1]

        a = [atom for atom in self.game.dfa.atoms() if self.game.dfa.delta(q,[atom]) in (set.union(set(curr_level_set), set(future_level_set)) - set([q]))]
        b = [x_prime for x_prime in self.game.tsys.states() if self.game.tsys.label(x_prime) in a]

        print("A:",a)
        print("B:",b)


        x_old = state[0][0]
        y_old = state[0][1]
        min_dist = -1
        min_element = None
        for x_prime in b:
            x_new = x_prime[0]
            y_new = x_prime[1]

            # Using manhatten distance as the distance metric
            dist = abs(x_old - x_new) + abs(y_old - y_new)
            if min_dist == -1:
                min_dist = dist
                min_element = x_prime
            elif dist < min_dist:
                min_dist = dist
                min_element = x_prime

        print("MIN ELEMENT:",min_element)
        return min_dist

    def isGoal(self, state):
        if self.game.final(state):
            return True
        else:
            return False
    def getSuccessors(self, state,rank):
        succ = []
        cost = []
        combined = []
        for act in self.game.actions():
            x = state[0]
            q = state[1]
            x_new = self.game.tsys.delta(x, act)
            atom = self.game.tsys.label(x)
            q_new = self.game.dfa.delta(q, atom)

            if self.isSink(q_new, rank):
                continue
            succ.append(x_new)
            cost.append(0.001)
            combined.append(((x_new,q_new), 0.001))
        print(combined)
        return combined


    def getLevel(self, state, rank):
        return rank[state[1]]

    def isSink(self, state, rank):
        q = state
        if rank[q] == float("inf"):
            return True
        else:
            return False

def gen_automaton():
    aut = SpotAutomaton("F(B & F(A & F(C)))")
    graph = aut.graphify()
    graph.to_png("aut.png", nlabel=["state"], elabel=["input"])
    return aut

def main():
    tsys = GridWorld(dim=(20, 20), A=[(19, 5)], B=[(12, 3)], C=[(10, 12), (2,2)])
    aut = gen_automaton()
    print("Gridworld setup")
    print("Automata states:", aut.states())

    prod = ProductTSysDFA(tsys, aut)
    astar = AStarAlgorithm(prod)
    astar.algorithm()


main()
