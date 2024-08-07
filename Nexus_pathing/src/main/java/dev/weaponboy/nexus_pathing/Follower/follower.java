package dev.weaponboy.nexus_pathing.Follower;

import dev.weaponboy.nexus_pathing.PathGeneration.pathBuilder;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingPower;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingVelocity;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

public class follower {

    pathOperator pathoperator;

    static RobotConfig config = new RobotConfig();

    double yI = 0;
    double xI = 0;

    boolean gotToEnd = false;

    PIDController xerror = new PIDController(config.getX_P_PATH_COR(), 0, config.getX_D_PATH_COR());
    PIDController yerror = new PIDController(config.getY_P_PATH_COR(), 0, config.getY_D_PATH_COR());

    PIDController correctiveXFinalAdjustment = new PIDController(config.getX_P_END_COR(), 0, config.getX_D_END_COR());
    PIDController correctiveYFinalAdjustment = new PIDController(config.getY_P_END_COR(), 0, config.getY_D_END_COR());

    PIDController largeHeadingPID = new PIDController(config.getHEADING_P_LARGE(), 0, config.getHEADING_D_LARGE());
    PIDController smallHeadingPID = new PIDController(config.getHEADING_P_SMALL(), 0, config.getHEADING_D_SMALL());

    public void setPath(pathBuilder path){
        pathoperator = new pathOperator(path.followablePath, path.pathingVelocity);
    }

    public void resetClosestPoint(Vector2D robotPos){
        pathoperator.getRobotPositionOnPathFullPath(robotPos);
    }

    public RobotPower followPathAuto(double targetHeading, double H, double X, double Y, double XV, double YV){

        Vector2D robotPositionVector = new Vector2D();

        Vector2D targetPoint = pathoperator.getPointOnFollowable(pathoperator.getLastPoint());

        robotPositionVector.set(X, Y);

        PathingPower correctivePower = new PathingPower();
        PathingPower pathingPower;

        pathingPower = getPathingPower(robotPositionVector, XV, YV);

        double Xpower = correctivePower.getVertical() + pathingPower.getVertical();
        double Ypower = correctivePower.getHorizontal() + pathingPower.getHorizontal();

        return new RobotPower(Xpower, Ypower, getTurnPower(targetHeading, H));
    }

    private double getTurnPower(double targetHeading, double currentHeading){

        double turnPower;

        double rotdist = (targetHeading - currentHeading);

        if (rotdist < -180) {
            rotdist = (360 + rotdist);
        } else if (rotdist > 180) {
            rotdist = (rotdist - 360);
        }

        if (Math.abs(rotdist) > 20){
            turnPower = largeHeadingPID.calculate(-rotdist);
        }else {
            turnPower = smallHeadingPID.calculate(-rotdist);
        }

        return turnPower;
    }

    private PathingPower getPathingPower(Vector2D robotPos, double XVelo, double YVelo){

        Vector2D error;
        PathingPower actualPathingPower = new PathingPower();

        double kyfull = 0.0079;
        double kxfull = 0.0053;

        double ky = 0.0053;
        double kx = 0.00346;

        int closestPos = pathoperator.getRobotPositionOnPath(robotPos);
        PathingVelocity targetVelocity = pathoperator.getTargetVelocity(closestPos);

        double curveY = Math.abs(YVelo*1.2)/2;
        double curveX = Math.abs(XVelo*1.2)/2;

        int index = closestPos + 40;

        if (index > pathoperator.pathCurve.size()-1){
            index = pathoperator.pathCurve.size()-1;
        }

        double XpowerCurve = pathoperator.pathCurve.get(index).getX()*curveX;
        double YpowerCurve = pathoperator.pathCurve.get(index).getY()*curveY;

        error = pathoperator.getErrorToPath(robotPos, closestPos);

        double xDist = error.getX();
        double yDist = error.getY();

        double xPowerC = xerror.calculate(xDist);
        double yPowerC = yerror.calculate(yDist);

        double veloXDef = targetVelocity.getXVelocity() - XVelo;
        double veloYDef = targetVelocity.getYVelocity() - YVelo;

        double vertical = kxfull * (targetVelocity.getXVelocity()+veloXDef);
        double horizontal = kyfull * (targetVelocity.getYVelocity()+veloYDef);

        if(horizontal > 1){
            vertical = kx * (targetVelocity.getXVelocity()+veloXDef);
            horizontal = ky * (targetVelocity.getYVelocity()+veloYDef);
        }

        double Xdenominator = Math.max(Math.abs(vertical) + Math.abs(xPowerC) + Math.abs(XpowerCurve), 1);
        double Ydenominator = Math.max(Math.abs(horizontal) + Math.abs(yPowerC) + Math.abs(YpowerCurve), 1);

        actualPathingPower.set((vertical+xPowerC+XpowerCurve)/Xdenominator, (horizontal+yPowerC+YpowerCurve)/Ydenominator);

        return actualPathingPower;
    }

    private PathingPower getCorrectivePowerAtEnd(Vector2D robotPos, Vector2D targetPos, double heading){

        Vector2D error;
        PathingPower correctivePower = new PathingPower();

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
