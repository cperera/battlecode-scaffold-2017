package seedplayer;
import battlecode.common.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public strictfp class RobotPlayer {
    static RobotController rc;

    /**
     * run() is the method that is called when a robot is instantiated in the Battlecode world.
     * If this method returns, the robot dies!
    **/
    @SuppressWarnings("unused")
    public static void run(RobotController rc) throws GameActionException {

        // This is the RobotController object. You use it to perform actions from this robot,
        // and to get information on its current status.
        RobotPlayer.rc = rc;

        // Here, we've separated the controls into a different method for each RobotType.
        // You can add the missing ones or rewrite this into your own control structure.
        switch (rc.getType()) {
            case ARCHON:
                runArchon();
                break;
            case GARDENER:
                runGardener();
                break;
            case SOLDIER:
                runSoldier();
                break;
            case LUMBERJACK:
                runLumberjack();
                break;
            case SCOUT:
                runScout();
                break;
        }
	}

    static void runArchon() throws GameActionException {
        System.out.println("I'm an archon! v2");

        // The code you want your robot to perform every round should be in this loop
        while (true) {

            // Try/catch blocks stop unhandled exceptions, which cause your robot to explode
            try {

                // Generate a random direction
                Direction dir = randomDirection();

                // Randomly attempt to build a gardener in this direction
                if (rc.canHireGardener(dir) && rc.getRobotCount()<40) {
                    rc.hireGardener(dir);
                }

                // Move randomly
                tryMove(randomDirection());

                // Broadcast archon's location for other robots on the team to know
                MapLocation myLocation = rc.getLocation();
                rc.broadcast(0,(int)myLocation.x);
                rc.broadcast(1,(int)myLocation.y);

                // Clock.yield() makes the robot wait until the next turn, then it will perform this loop again
                Clock.yield();

            } catch (Exception e) {
                System.out.println("Archon Exception");
                e.printStackTrace();
            }
        }
    }

	static void runGardener() throws GameActionException {
        System.out.println("I'm a gardener! v0");
        Direction currentTree = new Direction(0);

        int moveCount = 8;
        int treeCount = 5;
        int scoutCount = 0;
        int locationChannel = 6;
        float MESHMULTIPLIER = 50f;
        float MIN_DISTANCE = 6;

        ArrayList<MapLocation> otherGardenerLocations = new ArrayList<MapLocation>();

        while (true) {
            int otherLoc = rc.readBroadcast(locationChannel);
            if (otherLoc == 0) {
                break;
            }
            else {
                locationChannel++;
                float otherLocY = (otherLoc % 65536) / MESHMULTIPLIER;
                float otherLocX = (otherLoc >> 16) / MESHMULTIPLIER;
                MapLocation otherLocParsed = new MapLocation(otherLocX, otherLocY);
                otherGardenerLocations.add(otherLocParsed);

                System.out.println("Detected other at");
                System.out.println(otherLocX);
                System.out.println(otherLocY);
            }
        }
        System.out.println("My location channel is:");
        System.out.println(locationChannel);

//        try {
//            if (locationChannel != 6) {
//                // I am not the first gardener! move away!
//                while (true) {
//                    float minDist = -1;
//                    MapLocation myLocation = rc.getLocation();
//                    for (MapLocation otherGardener: otherGardenerLocations) {
//                        float dist = myLocation.distanceTo(otherGardener);
//                        if (dist < minDist || minDist == -1) {
//                            minDist = dist;
//                        }
//                    }
//                    System.out.println("Min dist:");
//                    System.out.println(minDist);
//
//                    if (minDist > MIN_DISTANCE) {
//                        System.out.println("Finally far enough away!");
//                        break;
//                    } else {
//                        Direction randomMoveOut = new Direction((float)Math.random()* (float)Math.PI*4.0f);
//                        tryMove(randomMoveOut);
//                    }
//                    Clock.yield();
//                }
//
//            }
//        }
//        catch (Exception e) {
//            e.printStackTrace();
//        }

        Direction moveOut = new Direction((float)Math.random()* (float)Math.PI*4.0f);
        if (locationChannel != 6) {
            for (int i=0;i<moveCount;i++) {
                tryMove(moveOut);
                Clock.yield();
            }
        }


        // The code you want your robot to perform every round should be in this loop
        while (true) {
            // Try/catch blocks stop unhandled exceptions, which cause your robot to explode
            try {

                // Listen for home archon's location
                int xPos = rc.readBroadcast(0);
                int yPos = rc.readBroadcast(1);
                MapLocation archonLoc = new MapLocation(xPos,yPos);

                // Generate a random direction
                Direction dir = currentTree;

                if (scoutCount < 2 && rc.canBuildRobot(RobotType.SCOUT, dir)) {
                    scoutCount++;
                    rc.buildRobot(RobotType.SCOUT, dir);
                }

                if (treeCount==0) {
                    // Randomly attempt to build a soldier or lumberjack in this direction
                    if (rc.canBuildRobot(RobotType.SOLDIER, dir) && Math.random() < .00) {
                        rc.buildRobot(RobotType.SOLDIER, dir);
                    } else if (rc.canBuildRobot(RobotType.LUMBERJACK, dir) && Math.random() < .15 && rc.isBuildReady()) {
                        rc.buildRobot(RobotType.LUMBERJACK, dir);
                    } else if (rc.canBuildRobot(RobotType.SCOUT, dir) && Math.random() < .6 && rc.isBuildReady()) {
                        rc.buildRobot(RobotType.SCOUT, dir);
                    }
                }

                if (rc.canPlantTree(currentTree) && treeCount >0){
                    rc.plantTree(currentTree);
                    treeCount--;
                }
                currentTree = new Direction(currentTree.radians + (float) Math.PI/3);


                // water planted trees
                TreeInfo[] nearbyTrees = rc.senseNearbyTrees();
                boolean watering =  false;

                ArrayList<TreeInfo> shuffleTrees = new ArrayList<TreeInfo>(Arrays.asList(nearbyTrees));
                Collections.shuffle(shuffleTrees);
                for (TreeInfo t: shuffleTrees){
                    if (t.getTeam() == rc.getTeam() && t.getHealth()<t.maxHealth){
                        if (rc.canWater(t.location) && !watering){
                            watering = true;
                            rc.water(t.location);
                        }
                    }
                }

                // Move randomly
                // tryMove(randomDirection());

                // Try donation!
                if (rc.getTeamBullets()>150){
                    rc.donate( 10.0f);
                }

                MapLocation myLocation = rc.getLocation();
                int locationPacked = (int)(myLocation.x * MESHMULTIPLIER) * 65536 + (int)(myLocation.y * MESHMULTIPLIER);
                rc.broadcast(locationChannel, locationPacked);

                // Clock.yield() makes the robot wait until the next turn, then it will perform this loop again
                Clock.yield();

            } catch (Exception e) {
                System.out.println("Gardener Exception");
                e.printStackTrace();
            }
        }
    }

    static void runSoldier() throws GameActionException {
        System.out.println("I'm a soldier!");
        Team enemy = rc.getTeam().opponent();

        // The code you want your robot to perform every round should be in this loop
        while (true) {

            // Try/catch blocks stop unhandled exceptions, which cause your robot to explode
            try {
                MapLocation myLocation = rc.getLocation();

                // See if there are any nearby enemy robots
                RobotInfo[] robots = rc.senseNearbyRobots(-1, enemy);

                // If there are some...
                if (robots.length > 0) {
                    // And we have enough bullets, and haven't attacked yet this turn...
                    if (rc.canFireSingleShot()) {
                        // ...Then fire a bullet in the direction of the enemy.
                        rc.fireSingleShot(rc.getLocation().directionTo(robots[0].location));
                    }
                }

                // Move randomly
                tryMove(randomDirection());

                // Clock.yield() makes the robot wait until the next turn, then it will perform this loop again
                Clock.yield();

            } catch (Exception e) {
                System.out.println("Soldier Exception");
                e.printStackTrace();
            }
        }
    }

    static void runLumberjack() throws GameActionException {
        System.out.println("I'm a lumberjack!");
        Team enemy = rc.getTeam().opponent();

        // The code you want your robot to perform every round should be in this loop
        while (true) {

            // Try/catch blocks stop unhandled exceptions, which cause your robot to explode
            try {

                // See if there are any enemy robots within striking range (distance 1 from lumberjack's radius)
                RobotInfo[] robots = rc.senseNearbyRobots(RobotType.LUMBERJACK.bodyRadius+GameConstants.LUMBERJACK_STRIKE_RADIUS, enemy);

                if(robots.length > 0 && !rc.hasAttacked()) {
                    // Use strike() to hit all nearby robots!
                    rc.strike();
                } else {
                    // No close robots, so search for robots within sight radius
                    robots = rc.senseNearbyRobots(-1,enemy);

                    // If there is a robot, move towards it
                    if(robots.length > 0) {
                        MapLocation myLocation = rc.getLocation();
                        MapLocation enemyLocation = robots[0].getLocation();
                        Direction toEnemy = myLocation.directionTo(enemyLocation);

                        tryMove(toEnemy);
                    } else {
                        TreeInfo[] nearbyTrees = rc.senseNearbyTrees();
                        boolean chopped = false;
                        for (TreeInfo t: nearbyTrees){
                            if (t.getTeam() != rc.getTeam() && rc.canChop(t.location)){
                                rc.chop(t.location);
                                chopped = true;
                            }
                        }
                        // Move Randomly
                        if (!chopped){
                            ArrayList<TreeInfo> enemyTrees = new ArrayList<TreeInfo>();
                            ArrayList<TreeInfo> neutrTrees = new ArrayList<TreeInfo>();
                            for (TreeInfo t: nearbyTrees){
                                if (t.getTeam() == enemy){
                                    enemyTrees.add(t);
                                }else if (t.getTeam()!= rc.getTeam()){
                                    neutrTrees.add(t);
                                }
                            }
                            if (enemyTrees.size()>0){
                                tryMove(rc.getLocation().directionTo(enemyTrees.get(0).location));
                            }else if (neutrTrees.size()>0){
                                tryMove(rc.getLocation().directionTo(neutrTrees.get(0).location));
                            }
                            tryMove(randomDirection());
                        }
                    }
                }

                // Clock.yield() makes the robot wait until the next turn, then it will perform this loop again
                Clock.yield();

            } catch (Exception e) {
                System.out.println("Lumberjack Exception");
                e.printStackTrace();
            }
        }
    }

    static void runScout() throws GameActionException {
        System.out.println("I'm a Scout v3.4!");
        Team enemy = rc.getTeam().opponent();
        Direction myMove = new Direction((float) Math.PI * 2 * (float) Math.random());
        while (true) {
            try {
                boolean moved = false;

                // See if there are any nearby enemy robots
                RobotInfo[] robots = rc.senseNearbyRobots(-1, enemy);

                // Broadcast enemy Archon position if seen, on channels 9 and 10
//                for (RobotInfo r: robots){
//                    if (r.getType() == RobotType.ARCHON){
//                        MapLocation enemyArch = r.getLocation();
//                        rc.broadcast(9,(int)enemyArch.x);
//                        rc.broadcast(10,(int)enemyArch.y);
//                    }
//                }
                // move towards nearby gardener
                for (RobotInfo r: robots){
                    if (r.getType() == RobotType.GARDENER){
                        myMove = rc.getLocation().directionTo(r.location);
                        if (rc.canMove(myMove, 1.4f)){
                            rc.move(myMove,1.4f);
                            moved = true;
                        }
                        else if (rc.canMove(myMove, .6f)){
                            rc.move(myMove,.6f);
                            moved = true;
                        }
                        else if (rc.canMove(myMove, .1f)){
                            rc.move(myMove,.1f);
                            moved = true;
                        }
                        else {
                            moved = true;
                        }
                        break;
                    }
                }

                // Sense trees and shake if possible
                TreeInfo[] trees = rc.senseNearbyTrees(-1,Team.NEUTRAL);

                for (TreeInfo t: trees) {
                    boolean canShake = rc.canShake(t.location);
                    int numBullets = t.getContainedBullets();
                    if (numBullets > 0) {
                        if (canShake) {
                            rc.shake(t.location);
                            break;
                        }
                        else {
                            myMove = rc.getLocation().directionTo(t.location);
                            break;
                        }
                    }
                }

                if (!rc.canMove(myMove)){
                    myMove = new Direction((float) Math.PI * 2 * (float) Math.random());
                }

                // Move in my direction
                if (!moved) {
                    tryMove(myMove);
                }

                // If there are some robots to shoot ...
                if (robots.length > 0) {
                    // And we have enough bullets, and haven't attacked yet this turn...
                    if (rc.canFireSingleShot()) {
                        // ...Then fire a bullet in the direction of the enemy.
                        rc.fireSingleShot(rc.getLocation().directionTo(robots[0].location));
                    }
                }

                Clock.yield();
            } catch (Exception e) {
                System.out.println("Scout Exception");
                e.printStackTrace();
            }
        }
    }

    /**
     * Returns a random Direction
     * @return a random Direction
     */
    static Direction randomDirection() {
        return new Direction((float)Math.random() * 2 * (float)Math.PI);
    }

    /**
     * Attempts to move in a given direction, while avoiding small obstacles directly in the path.
     *
     * @param dir The intended direction of movement
     * @return true if a move was performed
     * @throws GameActionException
     */
    static boolean tryMove(Direction dir) throws GameActionException {
        return tryMove(dir,20,3);
    }

    /**
     * Attempts to move in a given direction, while avoiding small obstacles direction in the path.
     *
     * @param dir The intended direction of movement
     * @param degreeOffset Spacing between checked directions (degrees)
     * @param checksPerSide Number of extra directions checked on each side, if intended direction was unavailable
     * @return true if a move was performed
     * @throws GameActionException
     */
    static boolean tryMove(Direction dir, float degreeOffset, int checksPerSide) throws GameActionException {

        // First, try intended direction
        if (rc.canMove(dir)) {
            rc.move(dir);
            return true;
        }

        // Now try a bunch of similar angles
        boolean moved = false;
        int currentCheck = 1;

        while(currentCheck<=checksPerSide) {
            // Try the offset of the left side
            if(rc.canMove(dir.rotateLeftDegrees(degreeOffset*currentCheck))) {
                rc.move(dir.rotateLeftDegrees(degreeOffset*currentCheck));
                return true;
            }
            // Try the offset on the right side
            if(rc.canMove(dir.rotateRightDegrees(degreeOffset*currentCheck))) {
                rc.move(dir.rotateRightDegrees(degreeOffset*currentCheck));
                return true;
            }
            // No move performed, try slightly further
            currentCheck++;
        }

        // A move never happened, so return false.
        return false;
    }

    /**
     * A slightly more complicated example function, this returns true if the given bullet is on a collision
     * course with the current robot. Doesn't take into account objects between the bullet and this robot.
     *
     * @param bullet The bullet in question
     * @return True if the line of the bullet's path intersects with this robot's current position.
     */
    static boolean willCollideWithMe(BulletInfo bullet) {
        MapLocation myLocation = rc.getLocation();

        // Get relevant bullet information
        Direction propagationDirection = bullet.dir;
        MapLocation bulletLocation = bullet.location;

        // Calculate bullet relations to this robot
        Direction directionToRobot = bulletLocation.directionTo(myLocation);
        float distToRobot = bulletLocation.distanceTo(myLocation);
        float theta = propagationDirection.radiansBetween(directionToRobot);

        // If theta > 90 degrees, then the bullet is traveling away from us and we can break early
        if (Math.abs(theta) > Math.PI/2) {
            return false;
        }

        // distToRobot is our hypotenuse, theta is our angle, and we want to know this length of the opposite leg.
        // This is the distance of a line that goes from myLocation and intersects perpendicularly with propagationDirection.
        // This corresponds to the smallest radius circle centered at our location that would intersect with the
        // line that is the path of the bullet.
        float perpendicularDist = (float)Math.abs(distToRobot * Math.sin(theta)); // soh cah toa :)

        return (perpendicularDist <= rc.getType().bodyRadius);
    }
}

/*

Channel directory:
    Archons broadcasting their locations: 0, 1, 2, 3, 4, 5, where even numbers are x and odd numbers are y

    Gardeners broadcasting their locations: 6-46
 */
