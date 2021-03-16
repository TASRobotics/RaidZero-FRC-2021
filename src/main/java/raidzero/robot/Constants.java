package raidzero.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import org.apache.commons.math3.util.FastMath;

public class Constants {
    /**
     * Swerve Constants
     */

    public static final class SwerveConstants {
        // Motor IDs in the order of motor, rotor, motor, rotor etc.
        public static final int[] SWERVE_IDS = {0, 1, 2, 3, 4, 5, 6, 7};
        public static final double[] INIT_MODULES_DEGREES = new double[] {32, 205, 41, 92};// new double[] {30, 205, 41, 92};

        // Robot dimensions
        // public static final double ROBOT_RADIUS = 30.0 / Math.sqrt(2);
        public static final double ROBOT_WIDTH = 23.0; // 23.0 inches
        public static final double ROBOT_RADIUS = FastMath.hypot(ROBOT_WIDTH, ROBOT_WIDTH) / 2.0;

        // Module Angles
        public static final double[] MODULE_ANGLES = {
            Math.PI / 4, Math.PI / 4 + Math.PI / 2,
            Math.PI / 4 + Math.PI, Math.PI / 4 + 3 * Math.PI / 2
        };

        // unit conversions and constants
        public static final double FALCON_TICKS = 2048;
        public static final double ROTOR_RATIO = 12;
        public static final double ROTOR_REVOLUTION_RATIO = ROTOR_RATIO * FALCON_TICKS;
        public static final double MOTOR_RATIO = 7.2;
        public static final double WHEEL_DIAMETER_INCHES = 4.0;
        public static final double SENSOR_UNITS_PER_INCH_MOTOR =
                FALCON_TICKS * MOTOR_RATIO / (WHEEL_DIAMETER_INCHES * Math.PI);

        // motor speed limits
        public static final double MAX_MOTOR_RPM = 6300;
        public static final double SECONDS_IN_MINUTE = 60;
        // TODO(louis): Clarify the next 2 constants
        public static final double MAX_MOTOR_SPEED_TICKS = 13500;
        public static final double MAX_MOTOR_SPEED_DRIVING =
                FALCON_TICKS * MAX_MOTOR_RPM / (10 * SECONDS_IN_MINUTE);
        public static final double MOTOR_SPEED_COEF = 0.8;

        // motor setup constants
        public static final FeedbackDevice FEEDBACKDEVICE = FeedbackDevice.IntegratedSensor;
        public static final int MOTOR_POSITION_SLOT = 0;
        public static final int MOTOR_VELOCITY_SLOT = 1;
        public static final int ROTOR_PID_SLOT = 0;
        public static final boolean DEFAULT_MOTOR_INVERSION = false;
        public static final boolean ROTOR_INVERSION = true;

        // PID constants
        public static final int PID_PRIMARY_SLOT = 0;
        public static final int PID_AUX_SLOT = 1;

        public static final double MOTOR_POSI_KP = 0.1;
        public static final double MOTOR_POSI_KD = 0;

        public static final double MOTOR_VELO_KF = 0.8 * 1023.0 / 13480;
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
        public static final int TRANSMIT_PERIOD_MS = 20;
    }

    /**
     * Intake Constants
     */
    public static final class IntakeConstants {
        public static final int TOP_MOTOR_ID = 10;

        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
        public static final boolean MOTOR_INVERSION = true;

        public static final double CONTROL_SCALING_FACTOR = 0.5;
    }

    /**
     * Conveyor Constants
     */
    public static final class ConveyorConstants {
        public static final int MOTOR_ID = 31;

        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
        public static final boolean MOTOR_INVERSION = true;

        public static final double KF = 0.010;
        public static final double KP = 0.00022;
        public static final double KI = 0;
        public static final double KD = 0.003;
    
        public static final double IZONE = 0;
        public static final double MINOUT = -1;
        public static final double MAXOUT = 1;

        public static final double MAXSPEED = 5000 * 2048;
    }

    public static final class TurretConstants {
        public static final int MOTOR_ID = 22;

        public static final IdleMode NEUTRAL_MODE = IdleMode.kBrake;
        public static final boolean INVERSION = false;
        public static final boolean INVERT_PHASE = false;

        public static final double TICKS_PER_DEGREE = 10732 / 90;

        public static final double MAX_INPUT_PERCENTAGE = 0.4;

        public static final double KF = 0.0;
        public static final double KP = 0.07;
        public static final double KI = 0.0;
        public static final double KD = 0.001;
        public static final int IZONE = 0;

        public static final double MINOUT = -1;
        public static final double MAXOUT = 1;
        public static final double MAXRPM = 5000;

        public static final int TOLERANCE = 1000;
        public static final double AT_SETPOINT_DURATION = 0.05;
    }

    public static final class HoodConstants {
        public static final int MOTOR_ID = 21;

        public static final IdleMode IDLE_MODE = IdleMode.kBrake;
        public static final boolean INVERSION = true;
        public static final boolean INVERT_SENSOR_PHASE = true;

        public static final double GEAR_RATIO = 45.0;

        public static final int FULLY_EXTENDED_TICKS = 6200;

        // The names refer to the angle of ball release
        public static enum HoodAngle {
            // +-500 for extra tolerance, limit switches should do its thing
            RETRACTED(-500), HIGH(FULLY_EXTENDED_TICKS / 3), MEDIUM(
                    2 * FULLY_EXTENDED_TICKS / 3), LOW(FULLY_EXTENDED_TICKS + 500);

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
        public static final double ATAN_COEFFICIENT = -600670000;// 6.0067*10^8
        public static final double DISTANCE_COEFFICIENT = -624343.7;
        public static final double ANGLE_CONSTANT = -943521338;
    }

    public static final class ShooterConstants {
        public static final int MOTOR_ID = 40;

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
        public static final double MIN_I = -0.08;
        public static final double MAX_I = 0.08;
        public static final double ANGLE_ADJUST_THRESHOLD = 2.0;

        public static final double AIM_ON_TARGET_DURATION = 0.2;
    }

    /**
     * Spindexer Constants
     */
    public static final class SpindexerConstants {
        public static final int MOTOR_ID = 20;
        public static final int SERVO_ID = 0;

        public static final double SERVO_UP = 0;
        public static final double SERVO_DOWN = 0.75;

        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
        public static final InvertType MOTOR_INVERSION = InvertType.InvertMotorOutput;
        public static final double SHOOT_SPEED = 0.6;
    }

    /**
     * Universal constants
     */
    public static final double JOYSTICK_DEADBAND = 0.06;

    public static final int TIMEOUT_MS = 10;

    public static final double SQRTTWO = Math.sqrt(2);
}
