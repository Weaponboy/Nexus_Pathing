package dev.weaponboy.nexus_pathing.PathGeneration;

import java.util.ArrayList;
import java.util.List;

import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingVelocity;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

public class pathBuilder {

    private SegmentGenerator segmentGenerator = new SegmentGenerator();

    private ArrayList<Vector2D> bezierPoints = new ArrayList<>();

    public ArrayList<Vector2D> followablePath = new ArrayList<>();

    public ArrayList<Double> headingTargets = new ArrayList<>();

    public ArrayList<PathingVelocity> pathingVelocity = new ArrayList<>();

    RobotConfig robotConfig = new RobotConfig();

    Vector2D secondPoint = new Vector2D();

    public void buildPath(sectionBuilder[] commands){

        for (int i = 0; i < commands.length; i++){
            commands[i].buildSection();
        }
        
        smoothPath(bezierPoints);

        motionProfile();

        calculateHeadings();

    }

    public void buildPath(sectionBuilder[] commands, double newAccelMax){

        for (int i = 0; i < commands.length; i++){
            commands[i].buildSection();
        }

        smoothPath(bezierPoints);

        motionProfile(newAccelMax);

        calculateHeadings();

    }

    private void smoothPath(ArrayList<Vector2D> originalPath){

        Vector2D onTheCurve = new Vector2D();

        followablePath.clear();

        int oldindex = 0;

        double XFirst = originalPath.get(oldindex).getX();
        double YFirst = originalPath.get(oldindex).getY();

        oldindex += 6;

        secondPoint = originalPath.get(oldindex);

        double xChange;
        double hypotenuse;

        while (oldindex != originalPath.size()){

            secondPoint = originalPath.get(oldindex);

            //assumes that the curve goes in the positive direction
            hypotenuse = Math.hypot(secondPoint.getX() - XFirst, secondPoint.getY() - YFirst);

            xChange = (secondPoint.getX() - XFirst);
            double yChange = (secondPoint.getY() - YFirst);

            double angle = 0;

            angle = Math.atan2(yChange, xChange);

            double newY = 0.25 * Math.sin(angle);
            double newX = 0.25 * Math.cos(angle);

            if (hypotenuse > 0.25){

                XFirst += newX;
                YFirst += newY;

                onTheCurve = new Vector2D(XFirst , YFirst);

                followablePath.add(onTheCurve);

            }else{
                oldindex++;
            }
        }

        System.out.println("processed and smoothed path");

    }

    private void motionProfile(){

        PathingVelocity pathVelo;

        int decelerationNumber = 0;

        double pathLength = calculateTotalDistance(followablePath);

        double accelDistance = (robotConfig.MAX_X_VELOCITY() * robotConfig.MAX_X_VELOCITY()) / (robotConfig.MAX_X_ACCELERATION()*2);

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = pathLength/2;

        if (accelDistance > halfway_distance){
            accelDistance = (halfway_distance);
        }

        double max_velocity = Math.sqrt(2 * robotConfig.MAX_X_ACCELERATION() * accelDistance);

        double deceleration_dt = accelDistance;

        int decIndex = (int) (deceleration_dt/0.25);

        System.out.println("accelDistance: " + accelDistance);
        System.out.println("max_velocity: " + max_velocity);
        System.out.println("decIndex: " + decIndex);

        int range;

        for (int i = 0; i < followablePath.size() - 1; i++) {

            if (i + decIndex >= followablePath.size()){

                decelerationNumber += 1;

                range = Math.abs(decIndex - decelerationNumber);

                double DecSlope = (double) range / (double) Math.abs(decIndex) * 100;

                DecSlope = DecSlope*0.01;

                Vector2D currentPoint = followablePath.get(i);
                Vector2D nextPoint = followablePath.get(i + 1);

                double deltaX = nextPoint.getX() - currentPoint.getX();
                double deltaY = nextPoint.getY() - currentPoint.getY();

                double percentSum = (Math.abs(deltaX)/0.25)+(Math.abs(deltaY)/0.25);

                double Xfactor = (deltaX/0.25) * (1/percentSum);
                double Yfactor = (deltaY/0.25) * (1/percentSum);

                double velocityXValue = (Xfactor * max_velocity) * DecSlope;
                double velocityYValue = (Yfactor * max_velocity) * DecSlope;

                pathVelo = new PathingVelocity(velocityXValue,velocityYValue);

                pathingVelocity.add(pathVelo);

                System.out.println("pathVelo deccel velocityXValue: " + Math.abs(velocityXValue + velocityYValue));

            }else {

                Vector2D currentPoint = followablePath.get(i);
                Vector2D nextPoint = followablePath.get(i + 1);

                double deltaX = nextPoint.getX() - currentPoint.getX();
                double deltaY = nextPoint.getY() - currentPoint.getY();

                double percentSum = (Math.abs(deltaX)/0.25)+(Math.abs(deltaY)/0.25);

                double Xfactor = (deltaX/0.25) * (1/percentSum);
                double Yfactor = (deltaY/0.25) * (1/percentSum);

                double velocityXValue = (Xfactor) * max_velocity;
                double velocityYValue = (Yfactor) * max_velocity;

                pathVelo = new PathingVelocity(velocityXValue, velocityYValue);

                pathingVelocity.add(pathVelo);

                System.out.println("pathVelo velocityXValue: " + Math.abs(velocityXValue + velocityYValue));

            }

        }

        System.out.println("generated motion profile");

    }

    private void motionProfile(double newAccelMax){

        PathingVelocity pathVelo;

        int decelerationNumber = 0;

        double pathLength = calculateTotalDistance(followablePath);

        double accelDistance = (robotConfig.MAX_X_VELOCITY() * robotConfig.MAX_X_VELOCITY()) / (newAccelMax*2);

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = pathLength/2;

        if (accelDistance > halfway_distance){
            accelDistance = (halfway_distance);
        }

        double max_velocity = Math.sqrt(2 * newAccelMax * accelDistance);

        double deceleration_dt = accelDistance;

        int decIndex = (int) (deceleration_dt/0.25);

        System.out.println("accelDistance: " + accelDistance);
        System.out.println("max_velocity: " + max_velocity);
        System.out.println("decIndex: " + decIndex);

        int range;

        for (int i = 0; i < followablePath.size() - 1; i++) {

            if (i + decIndex >= followablePath.size()){

                decelerationNumber += 1;

                range = Math.abs(decIndex - decelerationNumber);

                double DecSlope = (double) range / (double) Math.abs(decIndex) * 100;

                DecSlope = DecSlope*0.01;

                Vector2D currentPoint = followablePath.get(i);
                Vector2D nextPoint = followablePath.get(i + 1);

                double deltaX = nextPoint.getX() - currentPoint.getX();
                double deltaY = nextPoint.getY() - currentPoint.getY();

                double percentSum = (Math.abs(deltaX)/0.25)+(Math.abs(deltaY)/0.25);

                double Xfactor = (deltaX/0.25) * (1/percentSum);
                double Yfactor = (deltaY/0.25) * (1/percentSum);

                double velocityXValue = (Xfactor * max_velocity) * DecSlope;
                double velocityYValue = (Yfactor * max_velocity) * DecSlope;

                pathVelo = new PathingVelocity(velocityXValue,velocityYValue);

                pathingVelocity.add(pathVelo);

                System.out.println("pathVelo deccel velocityXValue: " + Math.abs(velocityXValue + velocityYValue));

            }else {

                Vector2D currentPoint = followablePath.get(i);
                Vector2D nextPoint = followablePath.get(i + 1);

                double deltaX = nextPoint.getX() - currentPoint.getX();
                double deltaY = nextPoint.getY() - currentPoint.getY();

                double percentSum = (Math.abs(deltaX)/0.25)+(Math.abs(deltaY)/0.25);

                double Xfactor = (deltaX/0.25) * (1/percentSum);
                double Yfactor = (deltaY/0.25) * (1/percentSum);

                double velocityXValue = (Xfactor) * max_velocity;
                double velocityYValue = (Yfactor) * max_velocity;

                pathVelo = new PathingVelocity(velocityXValue, velocityYValue);

                pathingVelocity.add(pathVelo);

                System.out.println("pathVelo velocityXValue: " + Math.abs(velocityXValue + velocityYValue));

            }

        }

        System.out.println("generated motion profile");

    }

    public void calculateHeadings() {

        for (int i = 1; i < followablePath.size(); i++) {
            Vector2D prevPoint = followablePath.get(i - 1);
            Vector2D currentPoint = followablePath.get(i);

            double dx = currentPoint.getX() - prevPoint.getX();
            double dy = currentPoint.getY() - prevPoint.getY();

            double heading = Math.atan2(dy, dx);
            headingTargets.add(Math.toDegrees(heading));
        }

    }

    private double calculateTotalDistance(List<Vector2D> path) {
        double totalDistance = 0.0;
        for (int i = 0; i < path.size() - 1; i++) {
            Vector2D point1 = path.get(i);
            Vector2D point2 = path.get(i + 1);
            totalDistance += calculateDistance(point1, point2);
        }
        return totalDistance;
    }

    private double calculateDistance(Vector2D point1, Vector2D point2) {
        double dx = point2.getX() - point1.getX();
        double dy = point2.getY() - point1.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public void buildPathSegment(Vector2D start, Vector2D end){
        segmentGenerator.buildPath(start, end);
        bezierPoints.addAll(segmentGenerator.copyPath());
        System.out.println("Built two point line");
    }

    public void buildPathSegment(Vector2D start, Vector2D control, Vector2D end){
        segmentGenerator.buildPath(start, control, end);
        bezierPoints.addAll(segmentGenerator.copyPath());
        System.out.println("Built three point curve");
    }

    public void buildPathSegment(Vector2D start, Vector2D control1, Vector2D control2, Vector2D end){
        segmentGenerator.buildPath(start, control1, control2, end);
        bezierPoints.addAll(segmentGenerator.copyPath());
        System.out.println("Built four point curve");
    }

    public void clearAll(){
        followablePath.clear();
        bezierPoints.clear();
        pathingVelocity.clear();
    }

}
