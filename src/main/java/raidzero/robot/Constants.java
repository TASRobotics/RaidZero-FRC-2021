package raidzero.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import java.lang.Math;

public class Constants {
    /**
     * Swerve Constants
     */
    
    public static final class SwerveConstants {
        // motor ids
        public static final int[] SWERVE_IDS = {
            0,1,2,3,4,5,6,7
        };
        public static final double[] INIT_MODULES_DEGREES = new double[] {30,205,41,92};

        // unit conversions and constants
        public static final double FALCON_TICKS = 2048;
        public static final double ROTOR_RATIO = 12;
        public static final double ROTOR_REVOLUTION_RATIO = ROTOR_RATIO * FALCON_TICKS;
        public static final double MOTOR_RATIO = 10;
        public static final double DEGREES_IN_REV = 360;
        public static final double RADIANS_IN_REV = 2*Math.PI;
        public static final double RAD_TO_DEG = DEGREES_IN_REV / RADIANS_IN_REV;
        public static final double SENSOR_UNITS_PER_INCH = 7.2;

        // motor speed limits
        public static final double MAX_MOTOR_RPM = 6300;
        public static final double SECONDS_IN_MINUTE = 60;
        public static final double MAX_MOTOR_SPEED = FALCON_TICKS*MAX_MOTOR_RPM/(10*SECONDS_IN_MINUTE);

        // motor setup constants
        public static final FeedbackDevice FEEDBACKDEVICE = FeedbackDevice.IntegratedSensor;
        public static final int MOTOR_POSITION_SLOT = 0;
        public static final int MOTOR_VELOCITY_SLOT = 1;
        public static final int ROTOR_PID_SLOT = 0;
        public static final boolean DEFAULT_MOTOR_INVERSION = false;
        public static final boolean ROTOR_INVERSION = true;
        
        //PID constants
        public static final int PID_PRIMARY_SLOT = 0;
        public static final int PID_AUX_SLOT = 1;

        public static final double MOTOR_POSI_KP = 0;
        public static final double MOTOR_POSI_KD = 0;
        public static final double MOTOR_VELO_KF = 0;//0.05;
        public static final double MOTOR_VELO_KP = 0.08;
        public static final double MOTOR_VELO_KD = 0.2;
        public static final double DEFAULT_TARG_ACCEL = 100000;
        public static final double DEFAULT_TARG_VELO = 22000;

        public static final double ROTOR_KP= 0.8;
        public static final double ROTOR_KD= 1.3;
        public static final double ROTOR_TARG_ACCEL = 100000; 
        public static final double ROTOR_TARG_VELO = 80000; 

        public static final double HEADING_KP = 0.01;
        public static final double HEADING_KI = 0;
        public static final double HEADING_KD = 0;
    }

    /**
     * Path Constants
     */
    public class PathConstants {

        public static final int BASE_TRAJ_PERIOD_MS = 0;
        public static final int MIN_POINTS_IN_TALON = 10;
        public static final int TRANSMIT_PERIOD_MS = 3;
   
    
    }

    /**
     * Universal constants
     */
    public static final double JOYSTICK_DEADBAND = 0.06;

    public static final int TIMEOUT_MS = 10;
    public static final double SQRTTWO = 1.4142135623730950488016887242097;

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