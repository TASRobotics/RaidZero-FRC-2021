package raidzero.robot;

public class Constants {
    /**
     * Swerve Constants
     */
    
    public static final class SwerveConstants {
        // motor ids
        public static final int[] SWERVE_IDS = {
            0,1,2,3,4,5,6,7
        };

        // unit conversions
        private static final double FALCON_TICKS = 2048;
        private static final double ROTOR_RATIO = 12;
        private static final double ROTOR_ANGLE_RATIO = ROTOR_RATIO * FALCON_TICKS;
        private static final double DEGREES_IN_REV = 360;

        // motor speed limits
        private static final double MAX_MOTOR_RPM = 6300;
        private static final double SECONDS_IN_MINUTE = 60;
        private static final double MAX_MOTOR_SPEED = FALCON_TICKS*MAX_MOTOR_RPM/(10*SECONDS_IN_MINUTE);

    }

    /**
     * Universal constants
     */
    public static final double JOYSTICK_DEADBAND = 0.06;

    public static final int TIMEOUT_MS = 10;

    /**
     * Drivetrain Constants
     * 
     * 
     *public static final class DriveConstants {
     *
     *    public static final double HIGH_GEAR_RATIO = 0;
     *    public static final double LOW_GEAR_RATIO = 0;
     *
     *    public static final double WHEEL_DIAMETER_INCHES = 0;
     *
     *    // Closed-loop constants
     *    public static final double DRIVE_NEUTRAL_DEADBAND = 0.06;
     *    public static final int PID_PRIMARY_SLOT = 0;
     *    public static final int PID_AUX_SLOT = 1;
     *    public static final double PIGEON_SCALE = 3600.0 / 8192.0;
     *
     *    public static final double PRIMARY_F = 0;
     *    public static final double PRIMARY_P = 0; // 1023 / (30 * 2000)
     *    public static final double PRIMARY_I = 0;
     *    public static final double PRIMARY_D = 0;
     *    public static final int PRIMARY_INT_ZONE = 0;
     *
     *    public static final double AUX_F = 0;
     *    public static final double AUX_P = 8;
     *    public static final double AUX_I = 0;
     *    public static final double AUX_D = 0;//4.0;
     *    public static final int AUX_INT_ZONE = 0;
     *    public static final boolean AUX_POLARITY = false;
     *
     *    public static final int BASE_TRAJ_PERIOD_MS = 0;
     *    public static final int SENSOR_UNITS_PER_ROTATION = 0;
     *    public static final double SENSOR_UNITS_PER_INCH_LOW_GEAR = 
     *        SENSOR_UNITS_PER_ROTATION * LOW_GEAR_RATIO / (WHEEL_DIAMETER_INCHES * Math.PI);
     *    public static final double SENSOR_UNITS_PER_INCH_HIGH_GEAR = 
     *        SENSOR_UNITS_PER_ROTATION * HIGH_GEAR_RATIO / (WHEEL_DIAMETER_INCHES * Math.PI);
     *    public static final int MIN_POINTS_IN_TALON = 10;
     *    public static final int TRANSMIT_PERIOD_MS = 3;
     *
     *    public static final double DEFAULT_CRUISE_VELOCITY = 8;
     *    public static final double DEFAULT_TARGET_ACCELERATION = 8;
     *
     *    // Joystick to Output mapping
     *    public static final double JOYSTICK_EXPONENT = 1;
     *    public static final double JOYSTICK_COEFFICIENT = 1;
     *
     *}
     */
} 