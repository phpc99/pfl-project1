import qualified Data.List
import qualified Data.Array
import qualified Data.Bits

-- PFL 2024/2025 Practical assignment 1

type City = String
type Path = [City]
type Distance = Int

type RoadMap = [(City,City,Distance)]

type AdjList = [(City,[(City,Distance)])]
--type AdjMatrix = Data.Array.Array (Int, Int) (Maybe Distance)

-- Get an Array of all nodes in a graph
-- RoadMap -> graph
cities :: RoadMap -> [City]
cities [] = [] -- base case
cities ((c1, c2, _) : xs) = Data.List.nub (c1 : c2 : cities xs)

-- Check whether two nodes are adjacent in a graph
-- RoadMap -> graph
-- City (1) and City (2) -> nodes to check for adjacency
areAdjacent :: RoadMap -> City -> City -> Bool
areAdjacent [] _ _ = False
areAdjacent ((c1, c2, _) : xs) tc1 tc2 =
    (((c1 == tc1) && (c2 == tc2)) || ((c1 == tc2) && (c2 == tc1))) || areAdjacent xs tc1 tc2

-- Calculate the distance between two nodes in a graph
-- RoadMap -> graph
-- City (1) and City (2) -> nodes to check distance
-- Return Nothing if they are not connected/ do not exist
distance :: RoadMap -> City -> City -> Maybe Distance
distance [] _ _ = Nothing
distance ((c1, c2, d) : xs) tc1 tc2 =
    if ((c1 == tc1) && (c2 == tc2)) || ((c1 == tc2) && (c2 == tc1)) then Just d else distance xs tc1 tc2

-- Get nodes adjacent to another node - along with the distance
-- RoadMap -> graph
-- City -> city to find adjacent nodes
adjacent :: RoadMap -> City -> [(City,Distance)]
adjacent [] _ = []
adjacent ((c1, c2, d) : xs) c
        | c1 == c = (c2, d) : adjacent xs c
        | c2 == c = (c1, d) : adjacent xs c
        | otherwise = adjacent xs c

-- Get nodes adjacent to another node
-- Same as function above, but without distance
-- RoadMap -> graph
-- City -> city to find adjacent nodes
_adjacent :: RoadMap -> City -> [City]
_adjacent [] _ = []
_adjacent ((c1, c2, _) : xs) c
  | c1 == c = c2 : _adjacent xs c
  | c2 == c = c1 : _adjacent xs c
  | otherwise = _adjacent xs c

-- Get the distance of a path between nodes
-- RoadMap -> graph
-- Path -> path to find the distance
-- Return Nothing if these nodes are not connected
pathDistance :: RoadMap -> Path -> Maybe Distance
pathDistance [] _ = Nothing -- empty graph
pathDistance _ [] = Just 0 -- empty path
pathDistance _ [_] = Just 0 -- single node path 
pathDistance g (c1 : c2 : xs) = do
    dist <- distance g c1 c2
    restDist <- pathDistance g (c2 : xs)
    return (dist + restDist)

-- Count number of edges of a node
-- RoadMap -> graph
-- City -> target node
countEdges :: RoadMap -> City -> Int
countEdges g c = length $ filter (\(c1, c2, _) -> c1 == c || c2 == c) g

-- Get nodes with the highest number of edges
-- RoadMap -> graph
rome :: RoadMap -> [City]
rome [] = [] -- empty graph
rome g =
  let
    allCities = cities g
    edgeCounts = map (countEdges g) allCities
    maxEdges = Prelude.maximum edgeCounts
   in
    -- filter all cities with the highest no. of edges
    filter (\city -> countEdges g city == maxEdges) allCities

-- Perform a DFS - depth first search to the
-- graph and return all visited nodes
-- RoadMap -> graph
-- [City] (1) -> array to keep track of visited nodes
-- City -> current node being visited
dfs :: RoadMap -> [City] -> City -> [City]
dfs g visited city
  | city `elem` visited = visited -- already visited
  | otherwise = foldl (dfs g) (city : visited) (_adjacent g city)

-- Perform a BFS - breadth first search to the graph
-- and return an Array of all visited nodes
-- RoadMap -> graph
-- [City] (1) -> array to keep track of visited nodes
-- City -> current node being visited
bfs :: RoadMap -> [City] -> City -> [City]
bfs g visited currentCity =
  if currentCity `elem` visited -- base case
    then visited
  else
    let newVisited = currentCity : visited
    in foldr (\city acc -> bfs g acc city) newVisited (_adjacent g currentCity)

-- Check if the graph is strongly connected, i.e,
-- if all nodes within it are connected
-- RoadMap -> graph
isStronglyConnected :: RoadMap -> Bool
isStronglyConnected g =
  let
    allCities = cities g
    reachableFromStart = bfs g [] (head allCities)
   in
    all (`elem` reachableFromStart) allCities

-- Create an adjacency list representation of the graph (utility)
-- complexity of N^2 ??
-- RoadMap -> graph
makeAdjacencyList :: RoadMap -> AdjList
makeAdjacencyList g = map (\city -> (city, adjacent g city)) (cities g)

{--
 1  function Dijkstra(Graph, source):
 2      
 3      for each vertex v in Graph.Vertices:
 4          dist[v] ← INFINITY
 5          prev[v] ← UNDEFINED
 6          add v to Q
 7      dist[source] ← 0
 8      
 9      while Q is not empty:
10          u ← vertex in Q with min dist[u]
11          remove u from Q
12          
13          for each neighbor v of u still in Q:
14              alt ← dist[u] + Graph.Edges(u, v)
15              if alt < dist[v]:
16                  dist[v] ← alt
17                  prev[v] ← u
18
19      return dist[], prev[]
--}

-- Get all shortest paths between two nodes in a graph
-- RoadMap -> graph
-- City (1) and City (2) -> nodes to find shortest path
shortestPath :: RoadMap -> City -> City -> [Path]
shortestPath g startCity targetCity =
  let 
    adjList = makeAdjacencyList g
    initialQueue = [(0, [startCity])]
  in 
    search adjList initialQueue Nothing [] targetCity

search :: AdjList -> [(Distance, [City])] -> Maybe Distance -> [Path] -> City -> [Path]
search adjList queue minDist results targetCity
    | null queue = results
    | otherwise =
      let -- Sort the queue based on total distance to simulate a priority queue
          sortedQueue = Data.List.sortOn fst queue
          ((totalDist, pathSoFar) : restQueue) = sortedQueue
          currentCity = head pathSoFar
       in if maybe False (totalDist >) minDist
            then
              -- All remaining paths will have greater distance
              results
            else
              if currentCity == targetCity
                then case minDist of
                  Nothing ->
                    -- First time reaching the target city
                    search adjList restQueue (Just totalDist) [reverse pathSoFar] targetCity
                  Just dist ->
                    if totalDist == dist
                      then
                        -- Found another shortest path
                        search adjList restQueue minDist (reverse pathSoFar : results) targetCity
                      else
                        -- Found a longer path; ignore it
                        search adjList restQueue minDist results targetCity
                else
                  -- Expand the current path to neighboring cities
                  let neighbors = case lookup currentCity adjList of
                        Just ns -> ns
                        Nothing -> []
                      newPaths =
                        [ (totalDist + d, neighbor : pathSoFar)
                          | (neighbor, d) <- neighbors,
                            neighbor `notElem` pathSoFar,
                            maybe True (totalDist + d <=) minDist
                        ]
                      newQueue = restQueue ++ newPaths
                   in search adjList newQueue minDist results targetCity

-- Helper to find city index
cityToIndex :: [City] -> City -> Int
cityToIndex cities city =
  case Data.List.elemIndex city cities of
    Just i -> i
    Nothing -> error "City not found in city list"

-- Main TSP dynamic programming function
travelSales :: RoadMap -> Path
travelSales roadmap =
  let cities = Data.List.nub $ concat [[c1, c2] | (c1, c2, _) <- roadmap]
   in if not (isStronglyConnected roadmap)
        then [] -- Return empty path if the graph is not fully connected
        else
          let n = length cities
              startCity = head cities
              startIndex = cityToIndex cities startCity
              allVisited = (1 `Data.Bits.shiftL` n) - 1

              -- DP table: (current city, visited cities bitmask) -> (distance, path)
              dp =
                Data.Array.array
                  ((0, 0), (n - 1, allVisited))
                  [((i, visited), tsp i visited) | i <- [0 .. n - 1], visited <- [0 .. allVisited]]

              -- Function to compute minimum path cost
              tsp :: Int -> Int -> (Distance, Path)
              tsp i visited
                | visited == (1 `Data.Bits.shiftL` i) = (0, [cities !! i]) -- Base case
                | otherwise =
                    let unvisited = filter (\j -> Data.Bits.testBit visited j && j /= i) [0 .. n - 1]
                        possiblePaths =
                          [ (dist + nextDist, cities !! i : path)
                            | j <- unvisited,
                              Just dist <- [distance roadmap (cities !! i) (cities !! j)],
                              let (nextDist, path) = dp Data.Array.! (j, visited `Data.Bits.clearBit` i)
                          ]
                     in if null possiblePaths
                          then (maxBound, []) -- No valid path found
                          else minimum possiblePaths

              -- Retrieve the final minimum cost path starting and ending at startCity
              finalPaths =
                [ (cost + d, path ++ [startCity])
                  | i <- [0 .. n - 1],
                    let (cost, path) = dp Data.Array.! (i, allVisited `Data.Bits.clearBit` i),
                    Just d <- [distance roadmap (last path) startCity],
                    not (null path)
                ]
           in if null finalPaths
                then [] -- Return empty list if no valid TSP path
                else snd (minimum finalPaths)

-- not applicable --
tspBruteForce :: RoadMap -> Path
tspBruteForce = undefined
--------------------

-- Some graphs to test your work
gTest1 :: RoadMap
gTest1 = [("7","6",1),("8","2",2),("6","5",2),("0","1",4),("2","5",4),("8","6",6),("2","3",7),("7","8",7),("0","7",8),("1","2",8),("3","4",9),("5","4",10),("1","7",11),("3","5",14)]

gTest2 :: RoadMap
gTest2 = [("0","1",10),("0","2",15),("0","3",20),("1","2",35),("1","3",25),("2","3",30)]

gTest3 :: RoadMap -- unconnected graph
gTest3 = [("0","1",4),("2","3",2)]
