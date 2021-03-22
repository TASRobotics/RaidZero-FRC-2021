package raidzero.robot.utils;

import edu.wpi.first.wpiutil.math.MathUtil;

public class MathTools {

    public static double wrapDegrees(double degrees) {
        return MathUtil.inputModulus(degrees, -180.0, 180.0);
    }
}