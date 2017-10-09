// Feel free to use this java file as a template and extend it to write your solver.
// ---------------------------------------------------------------------------------

import world.Robot;
import world.World;

import java.awt.*;
import java.util.*;

public class MyRobot extends Robot {
    boolean isUncertain;
    static World myWorld;
    
	@Override	
    public void travelToDestination() {
        // get robot location
        Point start = myWorld.getStartPos();
        Point end = myWorld.getEndPos();
        
        if (isUncertain) {
			// call function to deal with uncertainty
        	ArrayList<Point> blocked = new ArrayList<Point>();
        	ArrayList<Point> path = AStarIsUncertain(start, end, blocked);
            for(int i = path.size()-1; i >=0; i--){
            	Point moveTo = path.get(i);
            	this.move(moveTo);
            }
        }
        else {
			// call function to deal with certainty
        	ArrayList<Point> path = AStar(start, end);
        	// System.out.println(path.toString());
            // move robot to end
            for(int i = path.size()-1; i >=0; i--){
            	Point moveTo = path.get(i);
            	this.move(moveTo);
            }
        }
    }
    
	// code for certainty ---------------------------------------------------------------------------------
	
    // sudo code taken from wikipedia
    public ArrayList<Point> AStar(Point start, Point end){
    	
        // The set of nodes already evaluated
        ArrayList<Point> closedSet = new ArrayList<Point>();

        // The set of currently discovered nodes that are not evaluated yet.
        // Initially, only the start node is known.
        ArrayList<Point> openSet = new ArrayList<Point>();
        openSet.add(start);

        // For each node, which node it can most efficiently be reached from.
        // If a node can be reached from many nodes, cameFrom will eventually contain the
        // most efficient previous step.
        Map<Point, Point> cameFrom = new HashMap<Point, Point>();

        // For each node, the cost of getting from the start node to that node.
        Map<Point, Double> gScore = new HashMap<>();

        // The cost of going from start to start is zero.
        gScore.put(start, 0.0);

        // For each node, the total cost of getting from the start node to the goal
        // by passing by that node. That value is partly known, partly heuristic.
        Map<Point, Double> fScore = new HashMap<>();
        fScore.put(start, Double.MAX_VALUE);

        // For the first node, that value is completely heuristic.
        fScore.put(start, HeuristicCostEstimate(start, end));

        while (!openSet.isEmpty()){
        	System.out.println(openSet);
            //current := the node in openSet having the lowest fScore[] value
        	double minimum = Double.MAX_VALUE;
        	Point current = new Point();
        	for (int i = 0; i < openSet.size(); i++) {
        		if (fScore.get(openSet.get(i)) < minimum){
        			current = openSet.get(i);
        			minimum = fScore.get(openSet.get(i));
        		}
        	}
        	
            if ( (current.getX() == end.getX()) && (current.getY() == end.getY()) ){
                return reconstructPath(cameFrom, current);
            }
            
            openSet.remove(current);
            closedSet.add(current);

            // find neighbors
            ArrayList<Point> neighbors = new ArrayList<Point>();
            
            Point topleft = new Point((int) current.getX()-1, (int) current.getY()-1);
            if (isValid(topleft)) {
            	neighbors.add(topleft);
            	if(!gScore.keySet().contains(topleft)) gScore.put(topleft, Double.MAX_VALUE);
            }
            
            Point topmid = new Point((int) current.getX()-1, (int) current.getY());
            if (isValid(topmid)) {
            	neighbors.add(topmid);
            	if(!gScore.keySet().contains(topmid)) gScore.put(topmid, Double.MAX_VALUE);
            }
            
            Point topright = new Point((int) current.getX()-1, (int) current.getY()+1);
            if (isValid(topright)) {
            	neighbors.add(topright);
            	if(!gScore.keySet().contains(topright)) gScore.put(topright, Double.MAX_VALUE);
            }
            
            Point midleft = new Point((int) current.getX(), (int) current.getY()-1);
            if (isValid(midleft)) {
            	neighbors.add(midleft);
            	if(!gScore.keySet().contains(midleft)) gScore.put(midleft, Double.MAX_VALUE);
            }
            
            Point midright = new Point((int) current.getX(), (int) current.getY()+1);
            if (isValid(midright)) {
            	neighbors.add(midright);
            	if(!gScore.keySet().contains(midright)) gScore.put(midright, Double.MAX_VALUE);
            }
            
            Point botleft = new Point((int) current.getX()+1, (int) current.getY()-1);
            if (isValid(botleft)) {
            	neighbors.add(botleft);
            	if(!gScore.keySet().contains(botleft)) gScore.put(botleft, Double.MAX_VALUE);
            }
            
            Point botmid = new Point((int) current.getX()+1, (int) current.getY());
            if (isValid(botmid)) {
            	neighbors.add(botmid);
            	if(!gScore.keySet().contains(botmid)) gScore.put(botmid, Double.MAX_VALUE);
            }
            
            Point botright = new Point((int) current.getX()+1, (int) current.getY()+1);
            if (isValid(botright)) {
            	neighbors.add(botright);
            	if(!gScore.keySet().contains(botright)) gScore.put(botright, Double.MAX_VALUE);
            }
 
            for (int i = 0; i < neighbors.size(); i++){
            	if (closedSet.contains(neighbors.get(i))){
            		continue;		// Ignore the neighbor which is already evaluated.
            	}

            	if (!openSet.contains(neighbors.get(i))){	// Discover a new node
	                openSet.add(neighbors.get(i));
            	}
	            
	            // The distance from start to a neighbor
	            double tentative_gScore = gScore.get(current) + 1;
	            
	            if (tentative_gScore >= gScore.get(neighbors.get(i))){
	                continue;		// This is not a better path.
	            }
	            // This path is the best until now. Record it!
	            cameFrom.put(neighbors.get(i), current);
	            gScore.put(neighbors.get(i), tentative_gScore);
	            fScore.put(neighbors.get(i), gScore.get(neighbors.get(i))+HeuristicCostEstimate(neighbors.get(i),end));

            }
        }   
        return null;
    }
        
    public boolean isValid(Point p) {
    	int x = myWorld.numRows();
    	int y = myWorld.numCols();
    	if (p.getX() < x && p.getX() >= 0 && p.getY()< y && p.getY() >=0){
    		// make sure you can move to that location on the map
    		String positionStatus = pingMap(p).trim();
    		// okay if position is O, not okay if position is X
    		if(positionStatus.equals("O") || positionStatus.equalsIgnoreCase("F")){
    			return true;
    		}
    	}
    	return false;
    }
    
    // code for uncertainty -------------------------------------------------------------------------------------------
  public ArrayList<Point> AStarIsUncertain(Point start, Point end, ArrayList<Point> blocked){
    	
        // The set of nodes already evaluated
        ArrayList<Point> closedSet = new ArrayList<Point>();

        // The set of currently discovered nodes that are not evaluated yet.
        // Initially, only the start node is known.
        ArrayList<Point> openSet = new ArrayList<Point>();
        openSet.add(start);

        // For each node, which node it can most efficiently be reached from.
        // If a node can be reached from many nodes, cameFrom will eventually contain the
        // most efficient previous step.
        Map<Point, Point> cameFrom = new HashMap<Point, Point>();

        // For each node, the cost of getting from the start node to that node.
        Map<Point, Double> gScore = new HashMap<>();

        // The cost of going from start to start is zero.
        gScore.put(start, 0.0);

        // For each node, the total cost of getting from the start node to the goal
        // by passing by that node. That value is partly known, partly heuristic.
        Map<Point, Double> fScore = new HashMap<>();
        fScore.put(start, Double.MAX_VALUE);

        // For the first node, that value is completely heuristic.
        fScore.put(start, HeuristicCostEstimate(start, end));

        while (!openSet.isEmpty()){
            //current := the node in openSet having the lowest fScore[] value
        	double minimum = Double.MAX_VALUE;
        	Point current = new Point();
        	for (int i = 0; i < openSet.size(); i++) {
        		if (fScore.get(openSet.get(i)) < minimum){
        			current = openSet.get(i);
        			minimum = fScore.get(openSet.get(i));
        		}
        	}
        	
            if ( (current.getX() == end.getX()) && (current.getY() == end.getY()) ){
                return reconstructPath(cameFrom, current);
            }
            
            openSet.remove(current);
            closedSet.add(current);

            // find neighbors
            ArrayList<Point> neighbors = new ArrayList<Point>();
            
            Point topleft = new Point((int) current.getX()-1, (int) current.getY()-1);
            if (isValidIsUncertain(topleft, blocked)) {
            	neighbors.add(topleft);
            	if(!gScore.keySet().contains(topleft)) gScore.put(topleft, Double.MAX_VALUE);
            }
            
            Point topmid = new Point((int) current.getX()-1, (int) current.getY());
            if (isValidIsUncertain(topmid, blocked)) {
            	neighbors.add(topmid);
            	if(!gScore.keySet().contains(topmid)) gScore.put(topmid, Double.MAX_VALUE);
            }
            
            Point topright = new Point((int) current.getX()-1, (int) current.getY()+1);
            if (isValidIsUncertain(topright, blocked)) {
            	neighbors.add(topright);
            	if(!gScore.keySet().contains(topright)) gScore.put(topright, Double.MAX_VALUE);
            }
            
            Point midleft = new Point((int) current.getX(), (int) current.getY()-1);
            if (isValidIsUncertain(midleft, blocked)) {
            	neighbors.add(midleft);
            	if(!gScore.keySet().contains(midleft)) gScore.put(midleft, Double.MAX_VALUE);
            }
            
            Point midright = new Point((int) current.getX(), (int) current.getY()+1);
            if (isValidIsUncertain(midright, blocked)) {
            	neighbors.add(midright);
            	if(!gScore.keySet().contains(midright)) gScore.put(midright, Double.MAX_VALUE);
            }
            
            Point botleft = new Point((int) current.getX()+1, (int) current.getY()-1);
            if (isValidIsUncertain(botleft, blocked)) {
            	neighbors.add(botleft);
            	if(!gScore.keySet().contains(botleft)) gScore.put(botleft, Double.MAX_VALUE);
            }
            
            Point botmid = new Point((int) current.getX()+1, (int) current.getY());
            if (isValidIsUncertain(botmid, blocked)) {
            	neighbors.add(botmid);
            	if(!gScore.keySet().contains(botmid)) gScore.put(botmid, Double.MAX_VALUE);
            }
            
            Point botright = new Point((int) current.getX()+1, (int) current.getY()+1);
            if (isValidIsUncertain(botright, blocked)) {
            	neighbors.add(botright);
            	if(!gScore.keySet().contains(botright)) gScore.put(botright, Double.MAX_VALUE);
            }
 
            for (int i = 0; i < neighbors.size(); i++){
            	if (closedSet.contains(neighbors.get(i))){
            		continue;		// Ignore the neighbor which is already evaluated.
            	}

            	// neighbor is blocked location, ignore the neighbor
            	if (blocked.contains(neighbors.get(i))){
            		continue;
            	}
            	
            	if (!openSet.contains(neighbors.get(i))){	// Discover a new node
	                openSet.add(neighbors.get(i));
            	}
	            
	            // The distance from start to a neighbor
	            double tentative_gScore = gScore.get(current) + 1;
	            
	            if (tentative_gScore >= gScore.get(neighbors.get(i))){
	                continue;		// This is not a better path.
	            }
	            // This path is the best until now, move the robot and record it
	            // System.out.print(this.getPosition());
	            
	            this.move(neighbors.get(i));
	            cameFrom.put(neighbors.get(i), current);
	            gScore.put(neighbors.get(i), tentative_gScore);
	            fScore.put(neighbors.get(i), gScore.get(neighbors.get(i))+HeuristicCostEstimate(neighbors.get(i),end) + moreEstimate(neighbors.get(i), blocked));
	            System.out.println(openSet);
            }
        }   
        
        // robot is stuck, reset and try again (assumes that a path from S to F always exists)
        blocked.add(this.getPosition());
        backtrack(cameFrom, this.getPosition());
        return AStarIsUncertain(start, end, blocked);
    }
    
  public double moreEstimate(Point p, ArrayList<Point> blocked) {
  	int x = myWorld.numRows();
  	int y = myWorld.numCols();
  	if (p.getX() < x && p.getX() >= 0 && p.getY()< y && p.getY() >=0){
  		
	    	int xPt = 0;
	    	for (int i = 0; i < 200; i++){
	       		String positionStatus = pingMap(p).trim();
	       		// record if it's X
	    		if (positionStatus.equals("X")){
	    			xPt++;
	    		}
	    	}
	    	return xPt;
  	}
  	return 1000;
  }

    public boolean isValidIsUncertain(Point p, ArrayList<Point> blocked) {
    	int x = myWorld.numRows();
    	int y = myWorld.numCols();
    	
    	// valid location on map 
    	if (p.getX() < x && p.getX() >= 0 && p.getY()< y && p.getY() >=0){
    		
        	// neighbor is blocked location, ignore the neighbor
        	if (blocked.contains(p)){
        		return false;
        	}
        	return true;
        	/*
    		// robot current location 
    		Point robotLocation = this.getPosition();
    		// make sure you can move to that location on the map
    		Point robotNeighborLocation = this.move(p);
    		// robot was able to move to that location
    		if(robotNeighborLocation.equals(p)){
    			this.move(robotLocation);
    			return true;
    		}
    		*/
        	
        	
        	/*
        	int xPt = 0;
        	for (int i = 0; i < 200; i++){
           		String positionStatus = pingMap(p).trim();
           		// record if it's X
        		if (positionStatus.equals("X")){
        			xPt++;
        		}
        	}
        	//if it's a lot of X return false
        	if (xPt > 100){
        		return false;
        	}else {
        		return true;
        	}
        	*/
        	
    		/*
    		// robot was unable to move to that location, X
    		else{
    			blocked.add(p);
    			return false;
    		}*/
    	}
    	return false;
    }
    
    public double HeuristicCostEstimate(Point current, Point end){
    	// diagonal distance
    	double cost = Math.max(Math.abs(current.getX() - end.getX()), Math.abs(current.getY() - end.getY()));
    	return cost;
    }
    
    public void backtrack(Map<Point, Point> cameFrom, Point current){
    	while(cameFrom.keySet().contains(current)){
    		current = cameFrom.get(current);
    		this.move(current);
    	}
    }
    
    
    
    
    public ArrayList<Point> reconstructPath(Map<Point, Point> cameFrom, Point current) {
    	ArrayList<Point> totalPath = new ArrayList<>();
    	totalPath.add(current);
    	while(cameFrom.keySet().contains(current)){
    		current = cameFrom.get(current);
    		totalPath.add(current);
    	}
    	return totalPath;
    }

    @Override
    public void addToWorld(World world) {
        isUncertain = world.getUncertain();
        super.addToWorld(world);
    }

    public static void main(String[] args) {
        try {
			myWorld = new World("./src/TestCases/myInputFile3.txt", true);
			
            MyRobot robot = new MyRobot();
            robot.addToWorld(myWorld);
			// myWorld.createGUI(400, 400, 200); // uncomment this and create a GUI; the last parameter is delay in msecs

			robot.travelToDestination();
        }

        catch (Exception e) {
            e.printStackTrace();
        }
    }

}