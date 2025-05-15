package dev.weaponboy.nexus_pathing.PathGeneration;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingVelocity;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

public class pathsManager {

    private Map<String, pathBuilder> objectMap = new HashMap<>();

    public ArrayList<String> paths = new ArrayList<>();

    String currentPath = "null";

    RobotConfig robotConfig = new RobotConfig();

    public pathsManager(){}

    public pathsManager(RobotConfig CustomConfig){
        robotConfig = CustomConfig;
    }

    public void addNewPath(String pathName){

        pathBuilder newBuilder = new pathBuilder(robotConfig);

        objectMap.put(pathName, newBuilder);

        paths.add(pathName);

        currentPath = pathName;
    }

    public void addNewPath(String pathName, sectionBuilder[] pathSections) {

        pathBuilder newBuilder = new pathBuilder(robotConfig);

        objectMap.put(pathName, newBuilder);

        paths.add(pathName);

        currentPath = pathName;

        for (String s : paths) {
            if (s.equals(pathName)) {
                currentPath = pathName;

                if (robotConfig.logDebugging()){
                    System.out.println("");
                    System.out.println("trying: " + currentPath);
                }

                Objects.requireNonNull(objectMap.get(currentPath)).buildPath(pathSections);

                if (robotConfig.logDebugging()){
                    System.out.println("built: " + currentPath);
                }

                break;
            }
        }
    }

    public void buildPath(sectionBuilder[] pathSections, String pathName){
        for (String s : paths) {
            if (s.equals(pathName)) {
                currentPath = pathName;

                if (robotConfig.logDebugging()){
                    System.out.println("");
                    System.out.println("trying: " + currentPath);
                }

                Objects.requireNonNull(objectMap.get(currentPath)).buildPath(pathSections);

                if (robotConfig.logDebugging()){
                    System.out.println("built: " + currentPath);
                }

                break;
            }
        }
    }

    public void buildPath(sectionBuilder[] pathSections, String pathName, double newAccelMax){
        for (String s : paths) {
            if (s.equals(pathName)) {
                currentPath = pathName;

                if (robotConfig.logDebugging()){
                    System.out.println("");
                    System.out.println("trying: " + currentPath);
                }

                Objects.requireNonNull(objectMap.get(currentPath)).buildPath(pathSections, newAccelMax);

                if (robotConfig.logDebugging()){
                    System.out.println("built: " + currentPath);
                }

                break;
            }
        }
    }

    public void buildPath(sectionBuilder[] pathSections){

        if (!(currentPath == "null")) {

            if (robotConfig.logDebugging()){
                System.out.println("");
                System.out.println("trying: " + currentPath);
            }

            Objects.requireNonNull(objectMap.get(currentPath)).buildPath(pathSections);

            if (robotConfig.logDebugging()){
                System.out.println("built: " + currentPath);
            }

        }

    }

    public void buildPath(sectionBuilder[] pathSections, double newAccelMax){

        if (!(currentPath == "null")) {

            if (robotConfig.logDebugging()){
                System.out.println("");
                System.out.println("trying: " + currentPath);
            }

            Objects.requireNonNull(objectMap.get(currentPath)).buildPath(pathSections, newAccelMax);

            if (robotConfig.logDebugging()){
                System.out.println("built: " + currentPath);
            }

        }

    }

    public void addPoints(Vector2D start, Vector2D end){
        Objects.requireNonNull(objectMap.get(currentPath)).buildPathSegment(start, end);
    }

    public void addPoints(Vector2D start, Vector2D control, Vector2D end){
        Objects.requireNonNull(objectMap.get(currentPath)).buildPathSegment(start, control, end);
    }

    public void addPoints(Vector2D start, Vector2D control1, Vector2D control2, Vector2D end){
            Objects.requireNonNull(objectMap.get(currentPath)).buildPathSegment(start, control1, control2, end);
    }

    public String getCurrentPath() {
        return currentPath;
    }

    public void setCurrentPath(String currentPath) {
        for (String s : paths) {
            if (s.equals(currentPath)) {
                this.currentPath = currentPath;
                break;
            }
        }
    }

    public ArrayList<Vector2D> returnPathingPoints(){
        return Objects.requireNonNull(objectMap.get(currentPath)).followablePath;
    }

    public ArrayList<Vector2D> returnPathingPoints(String pathName){
        currentPath = pathName;
        return Objects.requireNonNull(objectMap.get(currentPath)).followablePath;
    }

    public ArrayList<PathingVelocity> returnVectorField(){
        return Objects.requireNonNull(objectMap.get(currentPath)).pathingVelocity;
    }

    public ArrayList<PathingVelocity> returnVectorField(String pathName){
        currentPath = pathName;
        return Objects.requireNonNull(objectMap.get(currentPath)).pathingVelocity;
    }

    public pathBuilder returnPath(String pathName){
        return Objects.requireNonNull(objectMap.get(pathName));
    }

    public pathBuilder returnCurrentPath(){
        return Objects.requireNonNull(objectMap.get(currentPath));
    }


}
