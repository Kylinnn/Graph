/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;


/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private Map<GeographicPoint, MapNode> vertices;
	private int numVertices;
	private int numEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		numVertices = 0;
		numEdges = 0;
		vertices = new HashMap<GeographicPoint, MapNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		Set<GeographicPoint> result = new HashSet<GeographicPoint>();
		for(GeographicPoint key : vertices.keySet())
		{
			result.add(key);
		}
		return result;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		if(location == null) return false;
		MapNode newVertex = new MapNode(location);
		if(vertices.get(location) != null)
			return false;
		else
		{
			vertices.put(location, newVertex);
			numVertices++;
			return true;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		if(from == null || to == null || vertices.get(from) == null || vertices.get(to) == null || roadName == null || roadType == null || length < 0)
		{
			this.addVertex(from);
			this.addVertex(to);
		}
		else
		{
			MapEdge newEdge = new MapEdge(from, to, roadName, roadType, length);
			List<MapEdge> Edges = vertices.get(from).getEdges();
			Edges.add(newEdge);
			numEdges++;
		}
	}
	
	public void printGraph() {
//		for(MapNode vertex: vertices.values())
//		{
//			List<MapEdge> Edges = vertex.getEdges();
//			System.out.println(vertex.getLocation() + ", Neighbors:");
//			List<GeographicPoint> neighbors = new ArrayList<GeographicPoint>();
//			for(int i=0; i<Edges.size(); i++)
//			{
//				neighbors.add(Edges.get(i).getDest());
//			}
//			System.out.println(neighbors);
//		}
		System.out.println("Vertices: " + numVertices);
		System.out.println("Edges: " + numEdges);
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		Map<GeographicPoint, Boolean> visited = new HashMap<GeographicPoint, Boolean>();
		Map<GeographicPoint, GeographicPoint> discover = new HashMap<GeographicPoint, GeographicPoint>();
		for(GeographicPoint key : vertices.keySet())
		{
			visited.put(key, false);
		}
		Queue<MapNode> Q = new LinkedList<MapNode>();
		Q.add(vertices.get(start));
		visited.put(start, true);
		while(!Q.isEmpty())
		{
			MapNode curNode = Q.peek();
			nodeSearched.accept(curNode.getLocation());
			Q.remove();
			if(curNode.getLocation() == goal) break;
			List<MapEdge> curEdges = curNode.getEdges();
			for(int i=0; i<curEdges.size(); i++)
			{
				GeographicPoint neighbor = curEdges.get(i).getDest();
				if(visited.get(neighbor) == false)
				{
					if(discover.get(neighbor) == null)
					{
						discover.put(neighbor, curNode.getLocation());
					}
					Q.add(vertices.get(neighbor));
					visited.put(neighbor, true);
				}
			}
		}
		GeographicPoint pathFind = goal;
		while(pathFind != null)
		{
			path.add(0, pathFind);
			pathFind = discover.get(pathFind);
		}
		// Hook for visualization.  See writeup.
//		nodeSearched.accept(next.getLocation());
		return path;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		for(MapNode node : vertices.values())
		{
			node.setDistance(Double.MAX_VALUE);
			node.setPre(null);
		}
		vertices.get(start).setDistance(0);
		Comparator<MapNode> comp = new MapNodeCompare();
		PriorityQueue<MapNode> PQ = new PriorityQueue<MapNode>(1, comp);
		Map<GeographicPoint, Boolean> visited = new HashMap<GeographicPoint, Boolean>();
		for(GeographicPoint key : vertices.keySet())
		{
			visited.put(key, false);
		}
		PQ.add(vertices.get(start));
		while(!PQ.isEmpty())
		{
			MapNode curNode = PQ.peek();
			visited.put(curNode.getLocation(), true);
			PQ.poll();
			nodeSearched.accept(curNode.getLocation());
			if(curNode.getLocation() == goal) break;
			List<MapEdge> curEdges = curNode.getEdges();
			for(int i=0; i<curEdges.size(); i++)
			{
				MapNode AdjNode = vertices.get(curEdges.get(i).getDest());
				if(visited.get(AdjNode.getLocation()) == true) continue;
				if(!PQ.contains(AdjNode))
				{
					PQ.add(AdjNode);
				}
				if(AdjNode.getDistance() > curNode.getDistance() + curEdges.get(i).getLength())
				{
					AdjNode.setDistance(curNode.getDistance() + curEdges.get(i).getLength());
					AdjNode.setPre(curNode);
					PQ.remove(AdjNode);
					PQ.add(AdjNode);
				}
			}
		}
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		GeographicPoint point = goal;
		while(point != null)
		{
			path.add(0, point);
			if(vertices.get(point).getPre() == null) break;
			point = vertices.get(point).getPre().getLocation();
		}
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return path;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
//		firstMap.printGraph();
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/simpletest.map", simpleTestMap);
		simpleTestMap.printGraph();
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.bfs(testStart,testEnd);
		List<GeographicPoint> testroute1 = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		System.out.println(testroute);
		System.out.println(testroute1);
		System.out.println(testroute2);
	
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		testMap.printGraph();
		
//		 A very simple test using real data
		testStart = new GeographicPoint(32.866743, -117.2136249);
		testEnd = new GeographicPoint(32.863053, -117.229059);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.bfs(testStart,testEnd);
		testroute1 = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		System.out.println(testroute);
		System.out.println(testroute1);
		System.out.println(testroute2);
		
		
		// A slightly more complex test using real data
//		testStart = new GeographicPoint(32.8674388, -117.2190213);
//		testEnd = new GeographicPoint(32.8697828, -117.2244506);
//		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
////		testroute = simpleTestMap.bfs(testStart,testEnd);
//		testroute1 = testMap.dijkstra(testStart,testEnd);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);
//		
////		System.out.println(testroute);
//		System.out.println(testroute1);
//		System.out.println(testroute2);
		
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
