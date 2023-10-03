package frc.robot.utils;

public class Conversions {

    public static double CanCoderToDegrees(double positionCounts, double gearRatio){
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }
    
    public static double degreesToCanCoder(double degrees, double gearRatio){
        return degrees / (360.0 / (gearRatio * 4096.0));
    }

    public static double degreesToFalcon(double degrees, double gearRatio){
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    public static double falconToDegrees(double positionCounts, double gearRatio){
        return positionCounts * (360.0 / (gearRatio * 2048.0));
    }
}