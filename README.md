# A* search
A* is a best-first search: starting from a specific starting node of a graph, it aims to find a path to the given goal node having 
the smallest cost (least distance travelled). It does this by maintaining a tree of paths originating at the start node and extending 
those paths one edge at a time until the goal node is reached (or if there are no paths eligible to be extended).

At each iteration of its main loop, A* needs to determine which of its paths to extend.
It does so based on the cost of the path and an estimate of the cost required to extend the path all the way to the goal.
Specifically, A* selects the path that minimizes: f = g + h. <br>

- g is the cost of the path from the start node to current
- h is the heuristic function that estimates the cost of the cheapest path from current to the goal.

Under an admissible heuristic function which never overestimates the actual cost to get to the goal, A* is guaranteed to return 
a least-cost path from start to goal.

Typical implementations of A* use a priority queue to perform the repeated selection of minimum (estimated) cost nodes to expand.
This priority queue is known as the open set. At each step of the algorithm, the node with the lowest f value is removed from the queue, 
the g and h values of its neighbors are updated, and these neighbors are added to the queue.
