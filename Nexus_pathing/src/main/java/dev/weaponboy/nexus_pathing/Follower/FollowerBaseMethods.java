package dev.weaponboy.nexus_pathing.Follower;

import dev.weaponboy.nexus_pathing.PathGeneration.PathBuilder;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingVelocity;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

public class FollowerBaseMethods {

    /**
     * Helper class objects
     * */
    private PathOperator pathoperator;
    static RobotConfig config;

    /**
     * PID controllers
     * */
    PIDController onPathXPID;
    PIDController onPathYPID;

    PIDController finalAdjustmentXPID;
    PIDController finalAdjustmentYPID;

    PIDController fastHeadingPID;
    PIDController slowHeadingPID;

    /**
     * Heading variables
     * */
    boolean robotHeadingBackwards = false;
    double offset = 0;
    double headingOffset = 0;
    int headingLookAheadDistance = 20;
    double addedHeadingErrorToOvercomeFriction = 0;
    private int lookAheadIndex = 0;

    /**Toggles*/
    boolean disableGlobalFollowing = false;
    boolean slowerHeading = false;
    boolean forceStop = false;
    boolean useHeadingFrictionError = false;
    boolean holdPosition = false;
    boolean usePathHeadings = false;

    /**
     * Conditions and indicators
     * */
    boolean pathFinished = false;

    /**
     * Robot data
     * */
    double velocityX = 0;
    double velocityY = 0;
    Vector2D robotPositionVector = new Vector2D();
    PathingVelocity targetVelocity = new PathingVelocity();
    Vector2D forceStopPoint = null;

    /**
     * Pathing Variables
     * */
    double yErrorToOvercomeFriction = 0;
    double xErrorToOvercomeFriction = 0;
    int currentIndex = 0;

    public void resetClosestPoint(Vector2D robotPos){
        pathoperator.getRobotPositionOnPathFullPath(robotPos);
    }

    public void createNewPathOperator(PathBuilder path){
        pathoperator = new PathOperator(path.followablePath, path.pathingVelocity, path.headingTargets);
    }

    public int clipLookAheadIndex(int index){
        if (index > pathoperator.targetHeadings.size()-1){
            index = pathoperator.targetHeadings.size()-1;
        }
        lookAheadIndex = index;
        return index;
    }

    public double calculateTargetHeading (){
        return ((pathoperator.targetHeadings.get(lookAheadIndex) + headingOffset) + offset);
    }

    /**
     * Pathing ending/error to end methods
     * */
    public void finishPath(){
        pathFinished = true;
        forceStopPoint = robotPositionVector;
    }

    public boolean isFinished(){
        Vector2D error = getErrorToPointOnPath(robotPositionVector.getX(), robotPositionVector.getY());
        return error.getX() < 1 && error.getY() < 1 || pathFinished;
    }

    public boolean isFinished(double xTolerance, double yTolerance){
        Vector2D error = getErrorToPointOnPath(robotPositionVector.getX(), robotPositionVector.getY());
        if (!pathFinished && error.getX() < xTolerance && error.getY() < yTolerance){
            pathFinished = true;
            forceStopPoint = robotPositionVector;
        }
        return error.getX() < xTolerance && error.getY() < yTolerance || pathFinished;
    }

    public double getErrorToEnd(){
        Vector2D error = getErrorToPointOnPath(robotPositionVector.getX(), robotPositionVector.getY());
        return Math.hypot(error.getX(), error.getY());
    }

    public double getXError(){
        Vector2D endPoint = pathoperator.getPointOnFollowable(pathoperator.getLastPoint());
        return Math.abs(endPoint.getX() - robotPositionVector.getX());
    }

    public double getYError(){
        Vector2D endPoint = pathoperator.getPointOnFollowable(pathoperator.getLastPoint());
        return Math.abs(endPoint.getY() - robotPositionVector.getY());
    }

    public Vector2D getErrorToPointOnPath(double currentX, double currentY){
        Vector2D endPoint = pathoperator.getPointOnFollowable(pathoperator.getLastPoint());
        double XError = endPoint.getX() - currentX;
        double YError = endPoint.getY() - currentY;
        return new Vector2D(XError, YError);
    }

    public Vector2D getErrorToPath(){
        currentIndex = pathoperator.getRobotPositionOnPath(robotPositionVector);
        pathoperator.getTargetVelocity(currentIndex);
        return pathoperator.getErrorToPath(robotPositionVector, currentIndex);
    }


}
