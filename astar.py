'''Prototype a star algo with ltl check'''


import heapq

# Define LTL function
def ltl(state):
    # Check if state satisfies LTL constraints

    # define ltl transistion here
    return True

# Define heuristic function
def heuristic(state, goal):
    # Return estimated cost to reach goal from state
    # define heuristic here
    return 0

def astar(start, goal):
    # Create priority queue for storing unexplored states
    queue = []
    heapq.heappush(queue, (0, start))

    # Create dictionary for storing explored states
    explored = {}

    # Create dictionary for storing came-from information
    came_from = {}

    # Create dictionary for storing cost-so-far information
    cost_so_far = {}
    cost_so_far[start] = 0

    # While there are unexplored states in the queue
    while queue:
        # Get next state from queue
        _, current = heapq.heappop(queue)

        # Check if current state is the goal
        if current == goal:
            # Return came-from information
            return came_from

        # Mark current state as explored
        explored[current] = True

        # Get next states and their costs
        next_states = [(1, 'a'), (3, 'b'), (5, 'c')]

        # Loop through next states
        for cost, next_state in next_states:
            # Check if next state has already been explored
            if next_state in explored:
                continue

            # Check if next state satisfies LTL constraints
            if not ltl(next_state):
                continue

            # Calculate new cost to reach next state
            new_cost = cost_so_far[current] + cost

            # Check if next state is in cost-so-far dictionary
            if next_state not in cost_so_far or new_cost < cost_so_far[next_state]:
                # Update came-from and cost-so-far information
                came_from[next_state] = current
                cost_so_far[next_state] = new_cost

                # Calculate priority for next state
                priority = new_cost + heuristic(next_state, goal)

                # Add next state to queue with calculated priority
                heapq.heappush(queue, (priority, next_state))

    # Return None if no path to goal was found
    return None

# Use A* algorithm to find path from start to goal
path = astar('start', 'goal')
