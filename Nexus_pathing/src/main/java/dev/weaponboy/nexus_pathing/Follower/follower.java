package dev.weaponboy.nexus_pathing.Follower;

import dev.weaponboy.nexus_pathing.PathGeneration.PathBuilder;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingPower;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingVelocity;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

public class Follower {

    PIDController xerror;
    PIDController yerror;

    static PIDController correctiveXFinalAdjustment;
    static PIDController correctiveYFinalAdjustment;

    PIDController largeHeadingPID;
    PIDController smallHeadingPID;

    boolean pathFinished = false;

    boolean forceStop = false;
    Vector2D forceStopPoint = null;

    double velocityAtStopX = 0;
    double velocityAtStopY = 0;

    double velocityX = 0;
    double velocityY = 0;

    boolean holdPosition = false;

    public void set180Front(boolean front180) {
        this.front180 = front180;
        if (front180){
            offset = 180;
        }else {
            offset = 0;
        }
    }

    boolean front180 = false;
    double offset = 0;

    double headingOffset = 0;

    public void disableGlobalFollowing(boolean disableGlobalFollowing) {
        this.disableGlobalFollowing = disableGlobalFollowing;
    }

    boolean disableGlobalFollowing = false;

    public void setHeadingOffset(double headingOffset) {
        this.headingOffset = headingOffset;
    }

    public void setHeadingLookAheadDistance(int headingLookAheadDistance) {
        this.headingLookAheadDistance = headingLookAheadDistance;
    }

    int headingLookAheadDistance = 20;

    public void setExtendoHeading(boolean extendoHeading) {
        this.extendoHeading = extendoHeading;
    }

    boolean extendoHeading = false;

    PathOperator pathoperator;

    static RobotConfig config;

    public Follower(){
        config = new RobotConfig();
        xerror = new PIDController(config.getX_P_PATH_COR(), 0, config.getX_D_PATH_COR());
        yerror = new PIDController(config.getY_P_PATH_COR(), 0, config.getY_D_PATH_COR());
        correctiveXFinalAdjustment = new PIDController(config.getX_P_END_COR(), 0, config.getX_D_END_COR());
        correctiveYFinalAdjustment = new PIDController(config.getY_P_END_COR(), 0, config.getY_D_END_COR());
        largeHeadingPID = new PIDController(config.getHEADING_P_LARGE(), 0, config.getHEADING_D_LARGE());
        smallHeadingPID = new PIDController(config.getHEADING_P_SMALL(), 0, config.getHEADING_D_SMALL());
    }

    public Follower(RobotConfig customConfig){
        config = customConfig;
        xerror = new PIDController(config.getX_P_PATH_COR(), 0, config.getX_D_PATH_COR());
        yerror = new PIDController(config.getY_P_PATH_COR(), 0, config.getY_D_PATH_COR());
        correctiveXFinalAdjustment = new PIDController(config.getX_P_END_COR(), 0, config.getX_D_END_COR());
        correctiveYFinalAdjustment = new PIDController(config.getY_P_END_COR(), 0, config.getY_D_END_COR());
        largeHeadingPID = new PIDController(config.getHEADING_P_LARGE(), 0, config.getHEADING_D_LARGE());
        smallHeadingPID = new PIDController(config.getHEADING_P_SMALL(), 0, config.getHEADING_D_SMALL());
    }

    double yOvercomeFriction = 0;
    double xOvercomeFriction = 0;

    double headingAddedIError = 0;

    boolean useHeadingI = false;

    boolean gotToEnd = false;

    Vector2D robotPositionVector = new Vector2D();
    int currentIndex = 0;

    public void usePathHeadings(boolean usePathHeadings) {
        this.usePathHeadings = usePathHeadings;
    }

    boolean usePathHeadings = false;

    public void setPath(PathBuilder path){
        pathFinished = false;
        forceStop = false;
        pathoperator = new PathOperator(path.followablePath, path.pathingVelocity, path.headingTargets);
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

        if (!isFinished() && Math.abs(XV) < 3 && Math.abs(YV) < 3) {
            xOvercomeFriction += getOvercomeFrictionValue(XError);
            yOvercomeFriction += getOvercomeFrictionValue(YError);
        } else {
            xOvercomeFriction = 0;
            yOvercomeFriction = 0;
        }

        if (pathFinished){

            if (holdPosition){

                double xErrorToEnd;
                double yErrorToEnd;

                if (disableGlobalFollowing){
                    xErrorToEnd = XError + xOvercomeFriction;
                    yErrorToEnd = YError + yOvercomeFriction;
                }else {
                    xErrorToEnd = (YError) * Math.sin(Math.toRadians(H)) + (XError) * Math.cos(Math.toRadians(H));
                    yErrorToEnd = (YError) * Math.cos(Math.toRadians(H)) - (XError) * Math.sin(Math.toRadians(H));
                }

                Xpower = correctiveXFinalAdjustment.calculate(xErrorToEnd);
                Ypower = correctiveYFinalAdjustment.calculate(yErrorToEnd);
            }else {
                Xpower = 0;
                Ypower = 0;
            }

        }else {

            if (Math.hypot(XError, YError) < 3) {

                double XErrorGlobal;
                double YErrorGlobal;

                if (disableGlobalFollowing){
                    XErrorGlobal = XError + xOvercomeFriction;
                    YErrorGlobal = YError + yOvercomeFriction;
                }else {
                    XErrorGlobal = (YError) * Math.sin(Math.toRadians(H)) + (XError) * Math.cos(Math.toRadians(H));
                    YErrorGlobal = (YError) * Math.cos(Math.toRadians(H)) - (XError) * Math.sin(Math.toRadians(H));
                }

                pathingPower = new PathingPower(correctiveXFinalAdjustment.calculate(XErrorGlobal), correctiveYFinalAdjustment.calculate(YErrorGlobal));
            } else {
                pathingPower = getPathingPower(robotPositionVector, XV, YV, H);
            }

            Xpower = correctivePower.getVertical() + pathingPower.getVertical();
            Ypower = correctivePower.getHorizontal() + pathingPower.getHorizontal();
        }

        int lookAheadIndex = currentIndex += headingLookAheadDistance;

        if (lookAheadIndex > pathoperator.targetHeadings.size()-1){
            lookAheadIndex = pathoperator.targetHeadings.size()-1;
        }

        if (usePathHeadings){
            double targetHeadingCalc = (pathoperator.targetHeadings.get(lookAheadIndex) + headingOffset) + offset;
            if (targetHeadingCalc > 360){
                targetHeadingCalc = targetHeadingCalc - 360;
            }

            return new RobotPower(Xpower, Ypower, getTurnPower(targetHeadingCalc, H, XV, YV));
        }else {
            return new RobotPower(Xpower, Ypower, getTurnPower(targetHeading, H, XV, YV));
        }

    }

    public void finishPath(){
        pathFinished = true;
        forceStopPoint = robotPositionVector;
        velocityAtStopX = velocityX;
        velocityAtStopY = velocityY;
    }

    public boolean isFinished(){
        Vector2D endPoint = pathoperator.getPointOnFollowable(pathoperator.getLastPoint());
        double XError = Math.abs(endPoint.getX() - robotPositionVector.getX());
        double YError = Math.abs(endPoint.getY() - robotPositionVector.getY());

        return XError < 1 && YError < 1 || pathFinished;
    }

    public void setI(double I){
        largeHeadingPID.setI(I);
        smallHeadingPID.setI(I);
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

    public double getTurnPower(double targetHeading, double currentHeading, double Xvelo, double Yvelo){

        double turnPower;

        double rotdist = (targetHeading - currentHeading);

        if (useHeadingI){
            if (rotdist > 0 && Math.abs(Xvelo) < 3 && Math.abs(Yvelo) < 3){
                headingAddedIError += 0.4;
            }else if (rotdist < 0 && Math.abs(Xvelo) < 3 && Math.abs(Yvelo) < 3){
                headingAddedIError -= 0.4;
            }else {
                headingAddedIError = 0;
            }
        }else{
            headingAddedIError = 0;
        }

        if (rotdist < -180) {
            rotdist = (360 + rotdist);
        } else if (rotdist > 180) {
            rotdist = (rotdist - 360);
        }

        rotdist += headingAddedIError;

        if (extendoHeading){
            turnPower = smallHeadingPID.calculate(-rotdist);
        }else {
            turnPower = largeHeadingPID.calculate(-rotdist);
        }

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

//        double curveY = Math.abs(YVelo*1.2)/2;
//        double curveX = Math.abs(XVelo*1.2)/2;
//
//        int index = closestPos + 40;
//
//        if (index > pathoperator.pathCurve.size()-1){
//            index = pathoperator.pathCurve.size()-1;
//        }
//
//        double XpowerCurve = pathoperator.pathCurve.get(index).getX()*curveX;
//        double YpowerCurve = pathoperator.pathCurve.get(index).getY()*curveY;
//
//        double relativeXCurve = (YpowerCurve) * Math.sin(Math.toRadians(heading)) + (XpowerCurve) * Math.cos(Math.toRadians(heading));
//        double relativeYCurve = (YpowerCurve) * Math.cos(Math.toRadians(heading)) - (XpowerCurve) * Math.sin(Math.toRadians(heading));

        error = pathoperator.getErrorToPath(robotPos, closestPos);

        double xDist = error.getX();
        double yDist = error.getY();

        double xPowerC = xerror.calculate(xDist + xOvercomeFriction);
        double yPowerC = yerror.calculate(yDist + yOvercomeFriction);

        double relativeXCorrective;
        double relativeYCorrective;

        if (disableGlobalFollowing){
            relativeXCorrective = xPowerC;
            relativeYCorrective = yPowerC;
        }else {
            relativeXCorrective = (yPowerC) * Math.sin(Math.toRadians(heading)) + (xPowerC) * Math.cos(Math.toRadians(heading));
            relativeYCorrective = (yPowerC) * Math.cos(Math.toRadians(heading)) - (xPowerC) * Math.sin(Math.toRadians(heading));
        }

        double veloXDef = targetVelocity.getXVelocity() - XVelo;
        double veloYDef = targetVelocity.getYVelocity() - YVelo;

//        double veloXDef = 0;
//        double veloYDef = 0;

        double relativeXVelo;
        double relativeYVelo;

        if (disableGlobalFollowing){
            relativeXVelo = (targetVelocity.getXVelocity()+veloXDef);
            relativeYVelo = (targetVelocity.getYVelocity()+veloYDef);
        }else {
            relativeXVelo = (targetVelocity.getYVelocity()+veloYDef) * Math.sin(Math.toRadians(heading)) + (targetVelocity.getXVelocity()+veloXDef) * Math.cos(Math.toRadians(heading));
            relativeYVelo = (targetVelocity.getYVelocity()+veloYDef) * Math.cos(Math.toRadians(heading)) - (targetVelocity.getXVelocity()+veloXDef) * Math.sin(Math.toRadians(heading));
        }

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

    public PathingPower pidToPoint(Vector2D robotPos, Vector2D targetPos, double heading, double XVelocity, double YVelocity){

        Vector2D error;
        PathingPower correctivePower = new PathingPower();

        error = new Vector2D( targetPos.getX() - robotPos.getX(),  targetPos.getY() - robotPos.getY());

        if (!isFinished() && Math.abs(XVelocity) < 3 && Math.abs(YVelocity) < 3) {
            xOvercomeFriction += getOvercomeFrictionValue(error.getX());
            yOvercomeFriction += getOvercomeFrictionValue(error.getY());
        } else {
            xOvercomeFriction = 0;
            yOvercomeFriction = 0;
        }

        double xDist = error.getX() + xOvercomeFriction;
        double yDist = error.getY() + yOvercomeFriction;

        double robotRelativeXError = yDist * Math.sin(Math.toRadians(heading)) + xDist * Math.cos(Math.toRadians(heading));
        double robotRelativeYError = yDist * Math.cos(Math.toRadians(heading)) - xDist * Math.sin(Math.toRadians(heading));

        double xPower = correctiveXFinalAdjustment.calculate(robotRelativeXError);
        double yPower = correctiveYFinalAdjustment.calculate(robotRelativeYError);

        correctivePower.set(xPower, yPower);

        return correctivePower;
    }

    private double getOvercomeFrictionValue(double error){
        if (error > 1) {
            return -1;
        }else if (error < -1) {
            return 1;
        } else {
            return 0;
        }
    }

    public double getPathLength(){
        return pathoperator.pathLength();
    }

    public boolean isHoldPosition() {
        return holdPosition;
    }

    public void setHoldPosition(boolean holdPosition) {
        this.holdPosition = holdPosition;
    }

    public boolean isUseHeadingI() {
        return useHeadingI;
    }

    public void setUseHeadingI(boolean useHeadingI) {
        this.useHeadingI = useHeadingI;
    }
}
