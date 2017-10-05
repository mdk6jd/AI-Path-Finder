// Feel free to use this java file as a template and extend it to write your solver.
// ---------------------------------------------------------------------------------

import world.Robot;
import world.World;

import java.awt.*;
import java.util.*;

public class MyRobot extends Robot {
    boolean isUncertain;
	
    @Override
    public void travelToDestination() {
        if (isUncertain) {
			// call function to deal with uncertainty
        	super.pingMap(new Point(5, 3));
        }
        else {
			// call function to deal with certainty
        	//super.move(new Point(3, 7));
        	
        	
        	/*initialize the open list
        	initialize the closed list
        	put the starting node on the open list (you can leave its f at zero)

        	while the open list is not empty
        	    find the node with the least f on the open list, call it "q"
        	    pop q off the open list
        	    generate q's 8 successors and set their parents to q
        	    for each successor
        	    	if successor is the goal, stop the search
        	        successor.g = q.g + distance between successor and q
        	        successor.h = distance from goal to successor
        	        successor.f = successor.g + successor.h

        	        if a node with the same position as successor is in the OPEN list \
        	            which has a lower f than successor, skip this successor
        	        if a node with the same position as successor is in the CLOSED list \ 
        	            which has a lower f than successor, skip this successor
        	        otherwise, add the node to the open list
        	    end
        	    push q on the closed list
        	end
        	*/
        }
    }

    @Override
    public void addToWorld(World world) {
        isUncertain = world.getUncertain();
        super.addToWorld(world);
    }

    public static void main(String[] args) {
        try {
			World myWorld = new World("E:\\eclipse\\workspace eclipse\\AI_HW_2\\src\\TestCases\\myInputFile1.txt", true);
			
            MyRobot robot = new MyRobot();
            robot.addToWorld(myWorld);
			//myWorld.createGUI(400, 400, 200); // uncomment this and create a GUI; the last parameter is delay in msecs
			

			robot.travelToDestination();
        }

        catch (Exception e) {
            e.printStackTrace();
        }
    }
}
