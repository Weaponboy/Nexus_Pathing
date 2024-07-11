package dev.weaponboy.nexus_pathing.PathGeneration;

import java.util.ArrayList;
import java.util.List;

import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingVelocity;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

public class pathBuilder {

    private SegmentGenerator segmentGenerator = new SegmentGenerator();

    private ArrayList<Vector2D> bezierPoints = new ArrayList<>();

    public ArrayList<Vector2D> followablePath = new ArrayList<>();

    public ArrayList<PathingVelocity> pathingVelocity = new ArrayList<>();

    Vector2D secondPoint = new Vector2D();

    public void buildPath(sectionBuilder[] commands){

        for (int i = 0; i < commands.length; i++){
            commands[i].buildSection();
        }
        
        smoothPath(bezierPoints);

        motionProfile();

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

        double acceleration_dt = (double) 190 / 146;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = pathLength / 1.2;
        double acceleration_distance;

        acceleration_distance = 0.5 * 146 * acceleration_dt * 2;

        if (acceleration_distance > halfway_distance){
            acceleration_dt = (halfway_distance / 146);
        }

        acceleration_distance = 0.5 * 146 * acceleration_dt * 2;

        double max_velocity = 146 * acceleration_dt;

        double deceleration_dt = acceleration_distance;

        int decIndex = (int) (deceleration_dt/0.25);

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

//                System.out.println("Vector dec " + (Math.abs(velocityYValue)+Math.abs(velocityXValue)));

                pathVelo = new PathingVelocity(velocityXValue,velocityYValue);

                pathingVelocity.add(pathVelo);

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

//                System.out.println("Vector " + (Math.abs(velocityYValue)+Math.abs(velocityXValue)));

                pathVelo = new PathingVelocity(velocityXValue, velocityYValue);

                pathingVelocity.add(pathVelo);

            }

        }

        System.out.println("generated motion profile");

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
