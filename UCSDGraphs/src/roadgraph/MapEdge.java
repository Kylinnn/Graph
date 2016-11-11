package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	private GeographicPoint start;
	private GeographicPoint end;
	private String streetName;
	private String streetType;
	private double length;
	
	public MapEdge(GeographicPoint local1, GeographicPoint local2, String name, String type, double len) {
		start = local1;
		end = local2;
		streetName = name;
		streetType = type;
		length = len;
	}
	
	public GeographicPoint getDest() {
		return this.end;
	}
	
	public double getLength() {
		return this.length;
	}
}
