package frc.robot.utils;

public class Utils {

    //method that returns the input if it is outside the deadband parameter, or 0 if it is inside
    public static double deadband(double x, double d) {
        return ((x > d) || (x < -d)) ? x : 0;
    }
}