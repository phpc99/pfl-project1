# PFL - Project no. 1

Functions 1 to 7 were developed by both students.
<br>Function 8 was developed by Pedro.
<br>Function 9 was developed by Nuno.

## Function no. 8, ``shortestPath``
The ``shortestPath`` function computes all the shortest paths between two cities in an undirected graph represented by a ``RoadMap``.

### Data Structures Used

**1. Adjacency List**: The graph is represented using an adjacency list, which is a list of tuples where each tuple contains a city and a list of its neighboring cities along with the distances to them.
<br>It is an efficient way to represent sparse graphs. It allows quick access to a city's neighbors and is suitable for graphs where not all cities are connected to all others.

``type AdjList = [(City, [(City, Distance)])]``

**2. Priority Queue**: The paths to be explored are stored in a list of tuples containing the total distance and the path taken so far.
<br>Since we cannot use priority queue implementations from other libraries, we simulate one by sorting the queue based on the total distance using ``sortOn`` from ``Data.List``.

``type Queue = [(Distance, Path)]``

**3. Results List**: A list of paths that are the shortest paths found between the start and target cities.
<br>Since we need to collect all paths that have the minimal total distance between the two cities, we use a list.

``type Results = [Path]``

**4. Visited Paths**: Instead of a separate visited set, we check if a city is already in the current path to avoid cycles.
<br>This method works for graphs without negative cycles and avoids revisiting nodes, which can lead to infinite loops.

### Algorithm Used
The function uses a modified version of Dijkstra's algorithm to find all shortest paths. It follows these steps:

**1. Building the Adjacency List**
<br>The ``buildAdjacencyList`` function constructs an adjacency list from the given ``RoadMap``.

```
buildAdjacencyList :: RoadMap -> AdjList
buildAdjacencyList roadMap = foldl updateAdj [] roadMap
```
For each road (edge) in the ``RoadMap``, we add the connected cities to each other's adjacency lists.

* ``updateAdj``: Adds both directions of the edge to the adjacency list.
* ``updateAdjacencyList``: Updates the adjacency list for a specific city.

**2. Initializing the Search**
<br>In ``shortestPath``, we initialize: 
* Adjacency List constructed from the ``RoadMap``.
* Initial Queue that contains a single path starting from the ``startCity`` with a total distance of 0 (``initialQueue = [(0, [startCity])]``).

**3. The Search Function**
<br>The ``search`` function recursively explores paths to find all shortest paths to the ``targetCity``.

```
search :: AdjList -> Queue -> Maybe Distance -> Results -> City -> Results
search adjList queue minDist results targetCity = ...
```

* ``adjList``: The adjacency list.
* ``queue``: The list of paths to explore.
* ``minDist``: The minimal distance found so far to the ``targetCity``.
* ``results``: The list of shortest paths found.
* ``targetCity``: The destination city.

Terminal Conditions:

* If the ``queue`` is empty, return the ``results``.
* If all remaining paths have a total distance greater than ``minDist``, return the ``results``.

**4. Main Loop of the Search Function**
<br>Priority Queue Simulation: The queue is sorted based on the total distance to prioritize paths with the shortest total distance.
<br>``sortedQueue = Data.List.sortOn fst queue``

<br>Exploring Paths:
```
Dequeue the path with the smallest total distance.
Check if the current city is the targetCity.
	If Yes:
		First Time Reaching Target:
			Update minDist with the total distance.
			Add the reversed path to results.
		If Total Distance Equals minDist:
			Add the reversed path to results.
		If Total Distance Greater Than minDist:
			Ignore the path.
	If No:
		Expand the current path to neighboring cities.
		For each neighbor:
			If the neighbor is not already in the current path.
			If the new total distance is less than or equal to minDist.
			Add the new path to the queue.
```

**5. Avoiding Cycles**
<br>When expanding paths, we ensure that we do not revisit cities already in the current path.
This check prevents infinite loops and ensures that all paths are simple paths (no cycles).
<br>``neighbor `notElem` pathSoFar``

**6. Collecting Shortest Paths**
<br>When we reach the ``targetCity``, we:
* Update ``minDist`` if it's the first time.
* Add the path to ``results`` if its total distance equals ``minDist``.

**7. Returning the Results**
<br>After the search completes, we return the list of all shortest paths found.

# Authors

- Nuno Simão Nunes Rios, up202206272@up.pt
- Pedro Henrique Pessôa Camargo, up202102365@up.pt
