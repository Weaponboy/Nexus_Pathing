package dev.weaponboy.nexus_pathing.Follower;

import android.util.Range;

import dev.weaponboy.nexus_pathing.PathGeneration.PathBuilder;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingPower;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingVelocity;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

public class Follower extends FollowerBaseMethods{

    public Follower(){
        config = new RobotConfig();
        onPathXPID = new PIDController(config.getX_P_PATH_COR(), 0, config.getX_D_PATH_COR());
        onPathYPID = new PIDController(config.getY_P_PATH_COR(), 0, config.getY_D_PATH_COR());
        finalAdjustmentXPID = new PIDController(config.getX_P_END_COR(), 0, config.getX_D_END_COR());
        finalAdjustmentYPID = new PIDController(config.getY_P_END_COR(), 0, config.getY_D_END_COR());
        fastHeadingPID = new PIDController(config.getHEADING_P_LARGE(), 0, config.getHEADING_D_LARGE());
        slowHeadingPID = new PIDController(config.getHEADING_P_SMALL(), 0, config.getHEADING_D_SMALL());
    }

    public Follower(RobotConfig customConfig){
        config = customConfig;
        onPathXPID = new PIDController(config.getX_P_PATH_COR(), 0, config.getX_D_PATH_COR());
        onPathYPID = new PIDController(config.getY_P_PATH_COR(), 0, config.getY_D_PATH_COR());
        finalAdjustmentXPID = new PIDController(config.getX_P_END_COR(), 0, config.getX_D_END_COR());
        finalAdjustmentYPID = new PIDController(config.getY_P_END_COR(), 0, config.getY_D_END_COR());
        fastHeadingPID = new PIDController(config.getHEADING_P_LARGE(), 0, config.getHEADING_D_LARGE());
        slowHeadingPID = new PIDController(config.getHEADING_P_SMALL(), 0, config.getHEADING_D_SMALL());
    }

    public void robotHeadingBackwards(boolean robotHeadingBackwards) {
        this.robotHeadingBackwards = robotHeadingBackwards;
        if (robotHeadingBackwards){
            offset = 180;
        }else {
            offset = 0;
        }
    }

    public void disableGlobalFollowing(boolean disableGlobalFollowing) {
        this.disableGlobalFollowing = disableGlobalFollowing;
    }

    public void setHeadingOffset(double headingOffset) {
        this.headingOffset = headingOffset;
    }

    public void setHeadingLookAheadDistance(int headingLookAheadDistance) {
        this.headingLookAheadDistance = headingLookAheadDistance;
    }

    public void slowerHeading(boolean slowerHeading) {
        this.slowerHeading = slowerHeading;
    }

    public void usePathHeadings(boolean usePathHeadings) {
        this.usePathHeadings = usePathHeadings;
    }

    public void setPath(PathBuilder path){
        pathFinished = false;
        forceStop = false;
        currentIndex = 0;
        xErrorToOvercomeFriction = 0;
        yErrorToOvercomeFriction = 0;
        addedHeadingErrorToOvercomeFriction = 0;
        onPathYPID.reset();
        onPathXPID.reset();
        finalAdjustmentXPID.reset();
        finalAdjustmentYPID.reset();
        fastHeadingPID.reset();
        slowHeadingPID.reset();
        createNewPathOperator(path);
    }

    public RobotPower followPathAuto(double targetHeading, double H, double X, double Y, double XV, double YV){

        double Xpower;
        double Ypower;

        velocityX = XV;
        velocityY = YV;

        robotPositionVector.set(X, Y);

        PathingPower correctivePower = new PathingPower();
        PathingPower pathingPower;

        Vector2D error = getErrorToPointOnPath(X, Y);

        if (!isFinished() && Math.abs(XV) < 3 && Math.abs(YV) < 3) {
            xErrorToOvercomeFriction += getOvercomeFrictionValue(error.getX());
            yErrorToOvercomeFriction += getOvercomeFrictionValue(error.getY());
        } else {
            xErrorToOvercomeFriction = 0;
            yErrorToOvercomeFriction = 0;
        }

        if (pathFinished){

            if (holdPosition){

                double xErrorToEnd;
                double yErrorToEnd;

                if (disableGlobalFollowing){
                    xErrorToEnd = error.getX() + xErrorToOvercomeFriction;
                    yErrorToEnd = error.getY() + yErrorToOvercomeFriction;
                }else {
                    xErrorToEnd = (error.getY()) * Math.sin(Math.toRadians(H)) + (error.getX()) * Math.cos(Math.toRadians(H));
                    yErrorToEnd = (error.getY()) * Math.cos(Math.toRadians(H)) - (error.getX()) * Math.sin(Math.toRadians(H));
                }

                Xpower = finalAdjustmentXPID.calculate(xErrorToEnd);
                Ypower = finalAdjustmentYPID.calculate(yErrorToEnd);
            }else {
                Xpower = 0;
                Ypower = 0;
            }

        }else {

            if (Math.hypot(error.getX(), error.getY()) < 3) {

                double XErrorGlobal;
                double YErrorGlobal;

                if (disableGlobalFollowing){
                    XErrorGlobal = error.getX() + xErrorToOvercomeFriction;
                    YErrorGlobal = error.getY() + yErrorToOvercomeFriction;
                }else {
                    XErrorGlobal = (error.getY()) * Math.sin(Math.toRadians(H)) + (error.getX()) * Math.cos(Math.toRadians(H));
                    YErrorGlobal = (error.getY()) * Math.cos(Math.toRadians(H)) - (error.getX()) * Math.sin(Math.toRadians(H));
                }

                pathingPower = new PathingPower(finalAdjustmentXPID.calculate(XErrorGlobal), finalAdjustmentYPID.calculate(YErrorGlobal));
            } else {
                pathingPower = getPathingPower(XV, YV, H);
            }

            Xpower = correctivePower.getVertical() + pathingPower.getVertical();
            Ypower = correctivePower.getHorizontal() + pathingPower.getHorizontal();
        }

        clipLookAheadIndex(currentIndex += headingLookAheadDistance);

        if (usePathHeadings){
            double targetPathHeading = calculateTargetHeading();

            if (targetPathHeading > 360){
                targetPathHeading = targetPathHeading - 360;
            }

            return new RobotPower(Xpower, Ypower, getTurnPower(targetPathHeading, H, XV, YV));
        }else {
            return new RobotPower(Xpower, Ypower, getTurnPower(targetHeading, H, XV, YV));
        }

    }

    public double getTurnPower(double targetHeading, double currentHeading, double Xvelo, double Yvelo){

        double turnPower;

        double rotdist = (targetHeading - currentHeading);

        if (rotdist < -180) {
            rotdist = (360 + rotdist);
        } else if (rotdist > 180) {
            rotdist = (rotdist - 360);
        }

        if (useHeadingFrictionError){
            if (rotdist > 0 && Math.abs(Xvelo) < 3 && Math.abs(Yvelo) < 3){
                addedHeadingErrorToOvercomeFriction += 0.4;
            }else if (rotdist < 0 && Math.abs(Xvelo) < 3 && Math.abs(Yvelo) < 3){
                addedHeadingErrorToOvercomeFriction -= 0.4;
            }else {
                addedHeadingErrorToOvercomeFriction = 0;
            }
        }else{
            addedHeadingErrorToOvercomeFriction = 0;
        }

        addedHeadingErrorToOvercomeFriction = Math.min(10, Math.max(-10, addedHeadingErrorToOvercomeFriction));

        rotdist += addedHeadingErrorToOvercomeFriction;

        if (rotdist < -180) {
            rotdist = (360 + rotdist);
        } else if (rotdist > 180) {
            rotdist = (rotdist - 360);
        }

        if (slowerHeading){
            turnPower = slowHeadingPID.calculate(-rotdist);
        }else {
            turnPower = fastHeadingPID.calculate(-rotdist);
        }

        return turnPower;
    }

    private PathingPower getPathingPower(double XVelo, double YVelo, double heading){

        Vector2D error;
        PathingPower actualPathingPower = new PathingPower();

        double kyfull = 1/config.MAX_Y_VELOCITY();
        double kxfull = 1/config.MAX_X_VELOCITY();

        double ky = 1/config.MAX_X_VELOCITY();
        double kx = 1/(config.MAX_X_VELOCITY()*(config.MAX_X_VELOCITY()/ config.MAX_Y_VELOCITY()));

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

        error = getErrorToPath();

        double xDist = error.getX();
        double yDist = error.getY();

        double xPowerC = onPathXPID.calculate(xDist + xErrorToOvercomeFriction);
        double yPowerC = onPathYPID.calculate(yDist + yErrorToOvercomeFriction);

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

        if(Math.abs(horizontal) > 1){
            vertical = kx * relativeXVelo;
            horizontal = ky * relativeYVelo;
        }

        double Xdenominator = Math.max(Math.abs(vertical) + Math.abs(relativeXCorrective), 1);
        double Ydenominator = Math.max(Math.abs(horizontal) + Math.abs(relativeYCorrective), 1);

        actualPathingPower.set((vertical+relativeXCorrective)/Xdenominator, (horizontal+relativeYCorrective)/Ydenominator);

        return actualPathingPower;
    }

    public PathingPower pidToPoint(Vector2D robotPos, Vector2D targetPos, double heading, double XVelocity, double YVelocity){

        Vector2D error;
        PathingPower correctivePower = new PathingPower();

        error = new Vector2D( targetPos.getX() - robotPos.getX(),  targetPos.getY() - robotPos.getY());

        if (!isFinished() && Math.abs(XVelocity) < 3 && Math.abs(YVelocity) < 3) {
            xErrorToOvercomeFriction += getOvercomeFrictionValue(error.getX());
            yErrorToOvercomeFriction += getOvercomeFrictionValue(error.getY());
        } else {
            xErrorToOvercomeFriction = 0;
            yErrorToOvercomeFriction = 0;
        }

        double xDist = error.getX() + xErrorToOvercomeFriction;
        double yDist = error.getY() + yErrorToOvercomeFriction;

        double robotRelativeXError = yDist * Math.sin(Math.toRadians(heading)) + xDist * Math.cos(Math.toRadians(heading));
        double robotRelativeYError = yDist * Math.cos(Math.toRadians(heading)) - xDist * Math.sin(Math.toRadians(heading));

        double xPower = finalAdjustmentXPID.calculate(robotRelativeXError);
        double yPower = finalAdjustmentYPID.calculate(robotRelativeYError);

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

    public boolean isHoldPosition() {
        return holdPosition;
    }

    public void holdPositionAtPathEnd(boolean holdPosition) {
        this.holdPosition = holdPosition;
    }

    public void headingErrorToOvercomeFriction(boolean useHeadingFrictionError) {
        this.useHeadingFrictionError = useHeadingFrictionError;
    }
}
