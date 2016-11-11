package roadgraph;

import java.util.Comparator;

public class MapNodeCompareTotal implements Comparator<MapNode> {

	@Override
	public int compare(MapNode n1, MapNode n2) {
		// TODO Auto-generated method stub
		if(n1.getPreDis() > n2.getPreDis())
			return 1;
		else if(n1.getPreDis() < n2.getPreDis())
			return -1;
		else
			return 0;
	}
	
}
