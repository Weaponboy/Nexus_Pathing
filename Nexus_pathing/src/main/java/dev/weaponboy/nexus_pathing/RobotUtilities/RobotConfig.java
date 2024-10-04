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

    private Properties properties = new Properties();

    public RobotConfig(String configFilePath) {
        try (InputStream input = new FileInputStream(configFilePath)) {
            properties.load(input);
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }

    public RobotConfig() {

        Properties properties = new Properties();
        try (InputStream inputStream = getClass().getClassLoader().getResourceAsStream("default-robot-config.properties")) {
            if (inputStream != null) {
                properties.load(inputStream);
            } else {
                throw new FileNotFoundException("Property file not found in the classpath");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        this.properties = properties;
    }

    public double getX_P_END_COR() {
        return Double.parseDouble(properties.getProperty("X_P_END_COR"));
    }

    public double getX_D_END_COR() {
        return Double.parseDouble(properties.getProperty("X_D_END_COR"));
    }

    public double getY_P_END_COR() {
        return Double.parseDouble(properties.getProperty("Y_P_END_COR"));
    }

    public double getY_D_END_COR() {
        return Double.parseDouble(properties.getProperty("Y_D_END_COR"));
    }

    public double getX_P_PATH_COR() {
        return Double.parseDouble(properties.getProperty("X_P_PATH_COR"));
    }

    public double getX_D_PATH_COR() {
        return Double.parseDouble(properties.getProperty("X_D_PATH_COR"));
    }

    public double getY_P_PATH_COR() {
        return Double.parseDouble(properties.getProperty("Y_P_PATH_COR"));
    }

    public double getY_D_PATH_COR() {
        return Double.parseDouble(properties.getProperty("Y_D_PATH_COR"));
    }

    public double getHEADING_P_LARGE() {
        return Double.parseDouble(properties.getProperty("HEADING_P_LARGE"));
    }

    public double getHEADING_D_LARGE() {
        return Double.parseDouble(properties.getProperty("HEADING_D_LARGE"));
    }

    public double getHEADING_P_SMALL() {
        return Double.parseDouble(properties.getProperty("HEADING_P_SMALL"));
    }

    public double getHEADING_D_SMALL() {
        return Double.parseDouble(properties.getProperty("HEADING_D_SMALL"));
    }

    public double MAX_X_VELOCITY() {
        return Double.parseDouble(properties.getProperty("MAX_X_VELOCITY"));
    }

    public double MAX_Y_VELOCITY() {
        return Double.parseDouble(properties.getProperty("MAX_Y_VELOCITY"));
    }

    public double MAX_X_ACCELERATION() {
        return Double.parseDouble(properties.getProperty("MAX_X_ACCELERATION"));
    }

    public double MAX_Y_ACCELERATION() {
        return Double.parseDouble(properties.getProperty("MAX_Y_ACCELERATION"));
    }
}
