package raidzero.robot.utils;

import raidzero.robot.Constants.SwerveConstants;

public class EncoderUtils {

    public static double ticksToInches(double ticks) {
        return ticks / SwerveConstants.SENSOR_UNITS_PER_INCH;
    }

    public static double inchesToTicks(double inches) {
        return inches * SwerveConstants.SENSOR_UNITS_PER_INCH;
    }
}