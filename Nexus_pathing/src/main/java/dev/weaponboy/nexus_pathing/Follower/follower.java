package dev.weaponboy.nexus_pathing.Follower;

import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.nexus_pathing.PathGeneration.pathBuilder;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingPower;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingVelocity;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

public class follower {

    boolean pathFinished = false;

    boolean forceStop = false;
    Vector2D forceStopPoint = null;

    double velocityAtStopX = 0;
    double velocityAtStopY = 0;

    double velocityX = 0;
    double velocityY = 0;

    ElapsedTime forceStopTimer = new ElapsedTime();

    public void setExtendoHeading(boolean extendoHeading) {
        this.extendoHeading = extendoHeading;
    }

    boolean extendoHeading = false;

    pathOperator pathoperator;

    static RobotConfig config = new RobotConfig();

    double yI = 0;
    double xI = 0;

    double headingI = 0;

    boolean gotToEnd = false;

    PIDController xerror = new PIDController(config.getX_P_PATH_COR(), 0, config.getX_D_PATH_COR());
    PIDController yerror = new PIDController(config.getY_P_PATH_COR(), 0, config.getY_D_PATH_COR());

    static PIDController correctiveXFinalAdjustment = new PIDController(config.getX_P_END_COR(), 0, config.getX_D_END_COR());
    static PIDController correctiveYFinalAdjustment = new PIDController(config.getY_P_END_COR(), 0, config.getY_D_END_COR());

    PIDController largeHeadingPID = new PIDController(config.getHEADING_P_LARGE(), 0, config.getHEADING_D_LARGE());
    PIDController smallHeadingPID = new PIDController(config.getHEADING_P_SMALL(), 0, config.getHEADING_D_SMALL());

    Vector2D robotPositionVector = new Vector2D();
    int currentIndex = 0;

    public void usePathHeadings(boolean usePathHeadings) {
        this.usePathHeadings = usePathHeadings;
    }

    boolean usePathHeadings = false;

    public void setPath(pathBuilder path){
        pathFinished = false;
        forceStop = false;
        pathoperator = new pathOperator(path.followablePath, path.pathingVelocity, path.headingTargets);
    }

    public void resetClosestPoint(Vector2D robotPos){
        pathoperator.getRobotPositionOnPathFullPath(robotPos);
    }

    public RobotPower followPathAuto(double targetHeading, double H, double X, double Y, double XV, double YV){

        double Xpower;
        double Ypower;

        velocityX = XV;
        velocityY = YV;

        robotPositionVector.set(X, Y);

        PathingPower correctivePower = new PathingPower();
        PathingPower pathingPower;

        Vector2D endPoint = pathoperator.getPointOnFollowable(pathoperator.getLastPoint());
        double XError = endPoint.getX() - X;
        double YError = endPoint.getY() - Y;

        if (pathFinished){

            if (forceStopTimer.milliseconds() > 600){

                XError = forceStopPoint.getX() - X;
                YError = forceStopPoint.getY() - Y;

                double XErrorGlobal = (YError) * Math.sin(Math.toRadians(H)) + (XError) * Math.cos(Math.toRadians(H));
                double YErrorGlobal = (YError) * Math.cos(Math.toRadians(H)) - (XError) * Math.sin(Math.toRadians(H));

                Xpower = correctiveXFinalAdjustment.calculate(XErrorGlobal);
                Ypower = correctiveYFinalAdjustment.calculate(YErrorGlobal);

            }else {
                double kyfull = 1/config.MAX_Y_VELOCITY();
                double kxfull = 1/config.MAX_X_VELOCITY();

                Xpower = -(velocityAtStopX * kxfull);
                Ypower = -(velocityAtStopY * kyfull);
            }


        }else {

            if (!isFinished() && Math.abs(XV) < 3 && Math.abs(YV) < 3) {
                if (Math.abs(XV) < 3 && Math.abs(XError) > 1) {
                    xI += 0.002;
                } else {
                    xI = 0;
                }

                if (Math.abs(YV) < 3 && Math.abs(YError) > 1) {
                    yI += 0.002;
                } else {
                    yI = 0;
                }
            } else {
                xI = 0;
                yI = 0;
            }

            if (Math.hypot(XError, YError) < 3) {
                correctiveXFinalAdjustment.setI(xI);
                correctiveYFinalAdjustment.setI(yI);

                double XErrorGlobal = (YError) * Math.sin(Math.toRadians(H)) + (XError) * Math.cos(Math.toRadians(H));
                double YErrorGlobal = (YError) * Math.cos(Math.toRadians(H)) - (XError) * Math.sin(Math.toRadians(H));

                pathingPower = new PathingPower(correctiveXFinalAdjustment.calculate(XErrorGlobal), correctiveYFinalAdjustment.calculate(YErrorGlobal));
            } else {
                pathingPower = getPathingPower(robotPositionVector, XV, YV, H);
            }

            Xpower = correctivePower.getVertical() + pathingPower.getVertical();
            Ypower = correctivePower.getHorizontal() + pathingPower.getHorizontal();
        }

        int lookAheadIndex = currentIndex += 20;

        if (lookAheadIndex > pathoperator.targetHeadings.size()-1){
            lookAheadIndex = pathoperator.targetHeadings.size()-1;
        }

        if (usePathHeadings){
            return new RobotPower(Xpower, Ypower, getTurnPower(pathoperator.targetHeadings.get(lookAheadIndex), H));
        }else {
            return new RobotPower(Xpower, Ypower, getTurnPower(targetHeading, H));
        }

    }

    public void finishPath(){
        pathFinished = true;
        forceStopPoint = robotPositionVector;
        velocityAtStopX = velocityX;
        velocityAtStopY = velocityY;
        forceStopTimer.reset();
    }

    public boolean isFinished(){
        Vector2D endPoint = pathoperator.getPointOnFollowable(pathoperator.getLastPoint());
        double XError = Math.abs(endPoint.getX() - robotPositionVector.getX());
        double YError = Math.abs(endPoint.getY() - robotPositionVector.getY());

        return XError < 1 && YError < 1 || pathFinished;
    }

    public boolean isFinished(double XTol, double YTol){
        Vector2D endPoint = pathoperator.getPointOnFollowable(pathoperator.getLastPoint());
        double XError = Math.abs(endPoint.getX() - robotPositionVector.getX());
        double YError = Math.abs(endPoint.getY() - robotPositionVector.getY());

        if (!pathFinished && XError < XTol && YError < YTol){
            pathFinished = true;
            forceStopPoint = robotPositionVector;
        }

        return XError < XTol && YError < YTol || pathFinished;
    }

    public double getErrorToEnd(){
        Vector2D endPoint = pathoperator.getPointOnFollowable(pathoperator.getLastPoint());
        double XError = Math.abs(endPoint.getX() - robotPositionVector.getX());
        double YError = Math.abs(endPoint.getY() - robotPositionVector.getY());

        return Math.hypot(XError, YError);
    }

    public double getXError(){
        Vector2D endPoint = pathoperator.getPointOnFollowable(pathoperator.getLastPoint());

        return Math.abs(endPoint.getX() - robotPositionVector.getX());
    }

    public double getYError(){
        Vector2D endPoint = pathoperator.getPointOnFollowable(pathoperator.getLastPoint());

        return Math.abs(endPoint.getY() - robotPositionVector.getY());
    }

    public double getTurnPower(double targetHeading, double currentHeading){

        double turnPower;

        double rotdist = (targetHeading - currentHeading);

        if (Math.abs(rotdist) > 5 && Math.abs(velocityX) < 3 && Math.abs(velocityY) < 3){
            headingI += 0.0001;
        }else {
            headingI = 0;
        }

        smallHeadingPID.setI(headingI);
        largeHeadingPID.setI(headingI);

        if (rotdist < -180) {
            rotdist = (360 + rotdist);
        } else if (rotdist > 180) {
            rotdist = (rotdist - 360);
        }

        if (extendoHeading){
            turnPower = largeHeadingPID.calculate(-rotdist);
        }else {
            turnPower = smallHeadingPID.calculate(-rotdist);
        }

//        if (Math.abs(rotdist) > 20){
//            c
//        }else {
//            turnPower = smallHeadingPID.calculate(-rotdist);
//        }

        return turnPower;
    }

    private PathingPower getPathingPower(Vector2D robotPos, double XVelo, double YVelo, double heading){

        Vector2D error;
        PathingPower actualPathingPower = new PathingPower();

        double kyfull = 1/config.MAX_Y_VELOCITY();
        double kxfull = 1/config.MAX_X_VELOCITY();

        double ky = 1/config.MAX_X_VELOCITY();
        double kx = 1/(config.MAX_X_VELOCITY()*(config.MAX_X_VELOCITY()/ config.MAX_Y_VELOCITY()));

        int closestPos = pathoperator.getRobotPositionOnPath(robotPos);
        PathingVelocity targetVelocity = pathoperator.getTargetVelocity(closestPos);

        currentIndex = closestPos;

        double curveY = Math.abs(YVelo*1.2)/2;
        double curveX = Math.abs(XVelo*1.2)/2;

        int index = closestPos + 40;

        if (index > pathoperator.pathCurve.size()-1){
            index = pathoperator.pathCurve.size()-1;
        }

        double XpowerCurve = pathoperator.pathCurve.get(index).getX()*curveX;
        double YpowerCurve = pathoperator.pathCurve.get(index).getY()*curveY;

        double relativeXCurve = (YpowerCurve) * Math.sin(Math.toRadians(heading)) + (XpowerCurve) * Math.cos(Math.toRadians(heading));
        double relativeYCurve = (YpowerCurve) * Math.cos(Math.toRadians(heading)) - (XpowerCurve) * Math.sin(Math.toRadians(heading));

        error = pathoperator.getErrorToPath(robotPos, closestPos);

        double xDist = error.getX();
        double yDist = error.getY();

        xerror.setI(xI);
        yerror.setI(yI);

        double xPowerC = xerror.calculate(xDist);
        double yPowerC = yerror.calculate(yDist);

        double relativeXCorrective = (yPowerC) * Math.sin(Math.toRadians(heading)) + (xPowerC) * Math.cos(Math.toRadians(heading));
        double relativeYCorrective = (yPowerC) * Math.cos(Math.toRadians(heading)) - (xPowerC) * Math.sin(Math.toRadians(heading));

        double veloXDef = targetVelocity.getXVelocity() - XVelo;
        double veloYDef = targetVelocity.getYVelocity() - YVelo;

//        double veloXDef = 0;
//        double veloYDef = 0;

        double relativeXVelo = (targetVelocity.getYVelocity()+veloYDef) * Math.sin(Math.toRadians(heading)) + (targetVelocity.getXVelocity()+veloXDef) * Math.cos(Math.toRadians(heading));
        double relativeYVelo = (targetVelocity.getYVelocity()+veloYDef) * Math.cos(Math.toRadians(heading)) - (targetVelocity.getXVelocity()+veloXDef) * Math.sin(Math.toRadians(heading));

        double vertical = kxfull * relativeXVelo;
        double horizontal = kyfull * relativeYVelo;

        if(horizontal > 1){
            vertical = kx * relativeXVelo;
            horizontal = ky * relativeYVelo;
        }

        double Xdenominator = Math.max(Math.abs(vertical) + Math.abs(relativeXCorrective), 1);
        double Ydenominator = Math.max(Math.abs(horizontal) + Math.abs(relativeYCorrective), 1);

        actualPathingPower.set((vertical+relativeXCorrective)/Xdenominator, (horizontal+relativeYCorrective)/Ydenominator);
//        actualPathingPower.set(vertical, horizontal);

        return actualPathingPower;
    }

    private PathingPower getCorrectivePowerAtEnd(Vector2D robotPos, Vector2D targetPos, double heading){

        Vector2D error;
        PathingPower correctivePower = new PathingPower();

        correctiveXFinalAdjustment.setI(xI);
        correctiveYFinalAdjustment.setI(yI);

        error = new Vector2D( targetPos.getX() - robotPos.getX(),  targetPos.getY() - robotPos.getY());

        double xDist = error.getX();
        double yDist = error.getY();

        double robotRelativeXError = yDist * Math.sin(Math.toRadians(heading)) + xDist * Math.cos(Math.toRadians(heading));
        double robotRelativeYError = yDist * Math.cos(Math.toRadians(heading)) - xDist * Math.sin(Math.toRadians(heading));

        double xPower = correctiveXFinalAdjustment.calculate(robotRelativeXError);
        double yPower = correctiveYFinalAdjustment.calculate(robotRelativeYError);

        correctivePower.set(xPower, yPower);

        return correctivePower;
    }

}
