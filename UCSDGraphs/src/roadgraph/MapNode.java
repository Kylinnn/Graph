package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class MapNode {
	private GeographicPoint location;
	private List<MapEdge> AdjList;
	private double dis;
	private double totalDis;
	private MapNode pre;
	
	public MapNode(GeographicPoint local) {
		location = local;
		AdjList = new ArrayList<MapEdge>();
		dis = Double.MAX_VALUE;
		totalDis = Double.MAX_VALUE;
		pre = null;
	}
	
	public GeographicPoint getLocation() {
		return this.location;
	}
	
	public List<MapEdge> getEdges() {
		return this.AdjList;
	}
	
	public double getDistance() {
		return this.dis;
	}
	
	public double getTotalDis() {
		return this.totalDis;
	}
	
	public MapNode getPre() {
		return this.pre;
	}
	
	public void setDistance(double val) {
		this.dis = val;;
	}
	
	public void setTotalDistance(double val) {
		this.totalDis = val;
	}
	
	public void setPre(MapNode n) {
		this.pre = n;
	}
}
