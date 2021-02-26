package raidzero.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

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

        //Robot Radius
        public static final double ROBOT_RADIUS = 30./Math.sqrt(2);

        //Module Angles
        public static final double[] MODULE_ANGLES = {Math.PI/4, Math.PI/4+Math.PI/2, Math.PI/4+Math.PI, Math.PI/4+3*Math.PI/2};
        

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
        public static final double MOTOR_SPEED_COEF = 0.8;

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
        public static final double MOTOR_VELO_KF = 0.05;
        public static final double MOTOR_VELO_KP = 0.08;
        public static final double MOTOR_VELO_KD = 0.2;
        public static final double DEFAULT_TARG_ACCEL = 100000;
        public static final double DEFAULT_TARG_VELO = 22000;

        public static final double ROTOR_KP = 0.8;
        public static final double ROTOR_KD = 0.1;
        public static final double ROTOR_KI = 0.005;
        public static final double ROTOR_IZONE = 6000;
        public static final double ROTOR_TARG_ACCEL = 100000; 
        public static final double ROTOR_TARG_VELO = 10000; 

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
     * Intake Constants
     */
    public static final class IntakeConstants {
        public static final int TOP_MOTOR_ID = 2;
        public static final int BOTTOM_MOTOR_ID = 3;

        public static final IdleMode TOP_NEUTRAL_MODE = IdleMode.kCoast;
        public static final boolean TOP_MOTOR_INVERSION = false;

        public static final IdleMode BOTTOM_NEUTRAL_MODE = IdleMode.kCoast;
        public static final boolean BOTTOM_MOTOR_INVERSION = false;

        public static final double CONTROL_SCALING_FACTOR = 1.0;
    }

    /**
     * Conveyor Constants
     */
    public static final class ConveyorConstants {
        public static final int MOTOR_ID = 1;

        public static final IdleMode NEUTRAL_MODE = IdleMode.kCoast;
        public static final boolean MOTOR_INVERSION = false;
    }

    public static final class TurretConstants {
        public static final int MOTOR_ID = 12;

        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
        public static final InvertType INVERSION = InvertType.InvertMotorOutput;
        public static final boolean INVERT_PHASE = false;

        public static final double TICKS_PER_DEGREE = 10732 / 90;

        public static final double MAX_INPUT_PERCENTAGE = 0.4;

        public static final double K_F = 0.0;
        public static final double K_P = 0.07;
        public static final double K_I = 0.0;
        public static final double K_D = 0.001;
        public static final int K_INTEGRAL_ZONE = 0;

        public static final int TOLERANCE = 1000;
        public static final double AT_SETPOINT_DURATION = 0.05;
    }

    public static final class HoodConstants {
        public static final int MOTOR_ID = 20;

        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
        public static final InvertType INVERSION = InvertType.InvertMotorOutput;
        public static final boolean INVERT_SENSOR_PHASE = true;

        public static final double GEAR_RATIO = 45.0;

        public static final int FULLY_EXTENDED_TICKS = 6200;

        // The names refer to the angle of ball release
        public static enum HoodAngle {
            // +-500 for extra tolerance, limit switches should do its thing
            RETRACTED(-500),
            HIGH(FULLY_EXTENDED_TICKS / 3), 
            MEDIUM(2 * FULLY_EXTENDED_TICKS / 3), 
            LOW(FULLY_EXTENDED_TICKS + 500);

            public final int ticks;

            private HoodAngle(int ticks) {
                this.ticks = ticks;
            }
        }
        public static final double K_F = 0;
        public static final double K_P = 0.6;
        public static final double K_I = 0;
        public static final double K_D = 0.001;
        public static final int K_INTEGRAL_ZONE = 0;

        public static final int TOLERANCE = 400;
        public static final double AT_SETPOINT_DURATION = 0.2;

        // distance to hood angle regression
        public static final double ATAN_COEFFICIENT = -600670000;//6.0067*10^8
        public static final double DISTANCE_COEFFICIENT = -624343.7;
        public static final double ANGLE_CONSTANT = -943521338;
    }

    public static final class ShooterConstants {
        public static final int MOTOR_ID = 30;

        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
        public static final InvertType INVERSION = InvertType.None;

        public static final double MAX_SPEED = 20000; // in ticks per 100ms
        public static final double FAKE_MAX_SPEED = 17000; // in ticks per 100ms
        public static final double ERROR_TOLERANCE = 250;
        public static final double UP_TO_SPEED_DURATION = 0.5; // in seconds

        public static final double K_F = 1023.0 / MAX_SPEED;
        public static final double K_P = 0.6;
        public static final double K_I = 0; // Shouldn't be touched
        public static final double K_D = 5.0; // Shouldn't be touched
        public static final int K_INTEGRAL_ZONE = 0; // Shouldn't be touched
    }

    /**
     * Limelight Constants
     */
    public static final class LimelightConstants {
        public static final String NAME = "limelight";

        public static final double MOUNTING_ANGLE = 31.4; // in degrees
        public static final double MOUNTING_HEIGHT = 0.56; // in meters

        // TODO: Improve the constants
        public static final double AIM_KP = 0.035;
        public static final double AIM_KI = 0.004;
        public static final double AIM_KD = 0.001;
        public static final double ANGLE_ADJUST_THRESHOLD = 2.0;

        public static final double AIM_ON_TARGET_DURATION = 0.2;
    }

    /**
     * Spindexer Constants
     */
    public static final class SpindexerConstants {
        public static final int MOTOR_ID = 15;

        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
        public static final InvertType MOTOR_INVERSION = InvertType.None;
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