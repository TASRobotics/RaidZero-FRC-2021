package raidzero.robot.utils;

import raidzero.robot.Constants.DriveConstants;

public class EncoderUtils {

    public static double ticksToInches(double ticks, boolean highGear) {
        if (highGear) {
            return ticks / DriveConstants.SENSOR_UNITS_PER_INCH_HIGH_GEAR;
        }
        return ticks / DriveConstants.SENSOR_UNITS_PER_INCH_LOW_GEAR;
    }

    public static double inchesToTicks(double inches, boolean highGear) {
        if (highGear) {
            return inches * DriveConstants.SENSOR_UNITS_PER_INCH_HIGH_GEAR;
        }
        return inches * DriveConstants.SENSOR_UNITS_PER_INCH_LOW_GEAR;
    }
}