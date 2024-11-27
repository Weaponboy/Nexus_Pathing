package dev.weaponboy.nexus_pathing.RobotUtilities;

import static java.security.AccessController.getContext;

import android.content.res.AssetManager;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Objects;
import java.util.Properties;

public class RobotConfig {

    double X_P_END_COR = 0.12;
    double X_D_END_COR = 0.0005;

    double Y_P_END_COR = 0.2;
    double Y_D_END_COR = 0.0005;

    double X_P_PATH_COR = 0.065;
    double X_D_PATH_COR = 0.0001;

    double Y_P_PATH_COR = 0.08;
    double Y_D_PATH_COR = 0.0001;

    double HEADING_P_LARGE = 0.022;
    double HEADING_D_LARGE = 0.0005;

    double HEADING_P_SMALL = 0.01;
    double HEADING_D_SMALL = 0.0008;

    double MAX_X_VELOCITY = 183;
    double MAX_Y_VELOCITY = 136;

    double MAX_X_ACCELERATION = 700;
    double MAX_Y_ACCELERATION = 90;

    public RobotConfig() {}

    public RobotConfig(double X_P_END_COR, double X_D_END_COR, double Y_P_END_COR, double Y_D_END_COR, double X_P_PATH_COR, double X_D_PATH_COR, double Y_P_PATH_COR, double Y_D_PATH_COR, double HEADING_P_LARGE, double HEADING_D_LARGE, double HEADING_P_SMALL, double HEADING_D_SMALL, double MAX_X_VELOCITY, double MAX_Y_VELOCITY, double MAX_X_ACCELERATION, double MAX_Y_ACCELERATION) {

        this.X_P_END_COR = X_P_END_COR;
        this.X_D_END_COR = X_D_END_COR;

        this.Y_P_END_COR = Y_P_END_COR;
        this.Y_D_END_COR = Y_D_END_COR;

        this.X_P_PATH_COR = X_P_PATH_COR;
        this.X_D_PATH_COR = X_D_PATH_COR;

        this.Y_P_PATH_COR = Y_P_PATH_COR;
        this.Y_D_PATH_COR = Y_D_PATH_COR;

        this.HEADING_P_LARGE = HEADING_P_LARGE;
        this.HEADING_D_LARGE = HEADING_D_LARGE;

        this.HEADING_P_SMALL = HEADING_P_SMALL;
        this.HEADING_D_SMALL = HEADING_D_SMALL;

        this.MAX_X_VELOCITY = MAX_X_VELOCITY;
        this.MAX_Y_VELOCITY = MAX_Y_VELOCITY;

        this.MAX_X_ACCELERATION = MAX_X_ACCELERATION;
        this.MAX_Y_ACCELERATION = MAX_Y_ACCELERATION;

    }

    public double getX_P_END_COR() {
        return X_P_END_COR;
    }

    public double getX_D_END_COR() {
        return X_D_END_COR;
    }

    public double getY_P_END_COR() {
        return Y_P_END_COR;
    }

    public double getY_D_END_COR() {
        return Y_D_END_COR;
    }

    public double getX_P_PATH_COR() {
        return X_P_PATH_COR;
    }

    public double getX_D_PATH_COR() {
        return X_D_PATH_COR;
    }

    public double getY_P_PATH_COR() {
        return Y_P_PATH_COR;
    }

    public double getY_D_PATH_COR() {
        return Y_D_PATH_COR;
    }

    public double getHEADING_P_LARGE() {
        return HEADING_P_LARGE;
    }

    public double getHEADING_D_LARGE() {
        return HEADING_D_LARGE;
    }

    public double getHEADING_P_SMALL() {
        return HEADING_P_SMALL;
    }

    public double getHEADING_D_SMALL() {
        return HEADING_D_SMALL;
    }

    public double MAX_X_VELOCITY() {
        return MAX_X_VELOCITY;
    }

    public double MAX_Y_VELOCITY() {
        return MAX_Y_VELOCITY;
    }

    public double MAX_X_ACCELERATION() {
        return MAX_X_ACCELERATION;
    }

    public double MAX_Y_ACCELERATION() {
        return MAX_Y_ACCELERATION;
    }
}
