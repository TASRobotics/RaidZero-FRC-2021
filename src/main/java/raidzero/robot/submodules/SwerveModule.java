package raidzero.robot.submodules;

import raidzero.robot.Constants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.dashboard.Tab;
import raidzero.robot.utils.MathTools;
import raidzero.robot.wrappers.LazyTalonFX;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import org.apache.commons.math3.util.FastMath;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class SwerveModule extends Submodule implements Sendable {

    private enum ControlState {
        VELOCITY, PATHING, TESTING
    };

    public LazyTalonFX motor;
    public LazyTalonFX rotor;

    private CANCoder rotorExternalEncoder;

    // The quadrant the module is in (cartesian plane)
    private int quadrant;
    private double forwardAngle = 0.0;

    private double outputMotorVelocity = 0.0;
    private double outputRotorAngle = 0.0;

    private ControlState controlState = ControlState.VELOCITY;

    private NetworkTableEntry motorVelocityEntry;

    @Override
    public void onInit() {
        throw new RuntimeException("Cannot initialize SwerveModule by itself.");
    }

    /**
     * Called once when the submodule is initialized.
     */
    public void onInit(int motorId, int rotorId, double forwardAngle, int quadrant) {
        this.quadrant = quadrant;
        this.forwardAngle = forwardAngle;

        motor = new LazyTalonFX(motorId);
        initMotor(motor);

        rotorExternalEncoder = new CANCoder(quadrant);
        rotorExternalEncoder.configFactoryDefault();
        rotorExternalEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360, Constants.TIMEOUT_MS);
        rotorExternalEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, Constants.TIMEOUT_MS);
        // rotorExternalEncoder.configMagnetOffset(forwardAngle, Constants.TIMEOUT_MS);

        rotor = new LazyTalonFX(rotorId);
        initRotor(rotor, rotorExternalEncoder);

        int column = 0;
        int row = 0;
        if (quadrant == 1) {
            column = 3;
            row = 0;
        } else if (quadrant == 2) {
            column = 1;
            row = 0;
        } else if (quadrant == 3) {
            column = 1;
            row = 2;
        } else {
            column = 3;
            row = 2;
        }

        Shuffleboard.getTab(Tab.MAIN).add("Rotor" + quadrant, this)
                .withSize(2, 2).withPosition(column, row);
        motorVelocityEntry = Shuffleboard.getTab(Tab.MAIN).add("Motor" + quadrant, 0)
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -10.0, "max", 10.0))
                .withSize(1, 1).withPosition(column + 4, row).getEntry();

        stop();
    }

    public void initMotor(TalonFX motor) {
        motor.configFactoryDefault();
        motor.setInverted(SwerveConstants.MOTOR_INVERSION);
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        // motor.configClosedloopRamp(0.1);
        // motor.configSelectedFeedbackCoefficient(SwerveConstants.MOTOR_TICKS_TO_METERS, SwerveConstants.PID_PRIMARY_SLOT, 20);
        motor.selectProfileSlot(0, SwerveConstants.PID_PRIMARY_SLOT);
        motor.config_kF(0, SwerveConstants.MOTOR_KF);
        motor.config_kP(0, SwerveConstants.MOTOR_KP);
        motor.config_kD(0, SwerveConstants.MOTOR_KD);
    }

    public void initRotor(TalonFX rotor, CANCoder encoder) {
        rotor.configFactoryDefault();
        rotor.setInverted(SwerveConstants.ROTOR_INVERSION);
        rotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        rotor.setSensorPhase(SwerveConstants.ROTOR_INVERT_SENSOR_PHASE);
        rotor.configRemoteFeedbackFilter(encoder, 0);
        rotor.selectProfileSlot(0, SwerveConstants.PID_PRIMARY_SLOT);
        rotor.config_kF(0, SwerveConstants.ROTOR_KF);
        rotor.config_kP(0, SwerveConstants.ROTOR_KP);
        rotor.config_kD(0, SwerveConstants.ROTOR_KD);
        rotor.configMotionAcceleration(SwerveConstants.ROTOR_TARG_ACCEL);
        rotor.configMotionCruiseVelocity(SwerveConstants.ROTOR_TARG_VELO);
    }

    /**
     * Called at the start of autonomous or teleop.
     * 
     * @param timestamp
     */
    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.VELOCITY;
        outputMotorVelocity = 0.0;
        outputRotorAngle = 0.0;
    }

    /**
     * Reads cached inputs & calculate outputs.
     */
    public void update(double timestamp) {
        double v = getMotorVelocity();
        // System.out.println(v);
        motorVelocityEntry.setDouble(v);
    }

    /**
     * Runs components in the submodule that have continuously changing inputs.
     */
    public void run() {
        switch (controlState) {
            case VELOCITY:
                motor.set(ControlMode.Velocity, outputMotorVelocity);
                rotor.set(ControlMode.MotionMagic, outputRotorAngle);
                break;
            case PATHING:
                break;
            case TESTING:
                motor.set(ControlMode.Velocity, outputMotorVelocity);
                rotor.set(ControlMode.MotionMagic, outputRotorAngle);
                break;
        }
    }

    /**
     * Returns the velocity of the motor in meters per second.
     * 
     * @return the velocity of the motor in meters per second.
     */
    public double getMotorVelocity() {
        return motor.getSelectedSensorVelocity(SwerveConstants.PID_PRIMARY_SLOT)
             * SwerveConstants.MOTOR_TICKS_TO_METERS * 10.0;
    }

    /**
     * Sets the velocity of the motor.
     * 
     * @param velocity target motor velocity in meters per second.
     */
    public void setMotorVelocity(double velocity) {
        controlState = ControlState.VELOCITY;

        outputMotorVelocity = velocity / (SwerveConstants.MOTOR_TICKS_TO_METERS * 10.0);

        double angleError = getRotorAngleError();
        if (angleError >= 10.0) {
            System.out.println("Angle error: " + angleError);
        }
        // System.out.println("Q" + quadrant + ": error=" + angleError);
        // outputMotorVelocity = outputMotorVelocity * FastMath.exp((1 / 5.0) * -angleError);
        // outputMotorVelocity = outputMotorVelocity * FastMath.pow(1 - Math.abs(angleError / 360.0), 4);
    }

    /**
     * Returns the angle of the rotor (+ is ccw) in degrees.
     * 
     * @return angle of rotor in degrees.
     */
    public double getRotorAngle() {
        return rotorExternalEncoder.getPosition();
    }

    /**
     * Returns the negated angle of the rotor (+ is cw) in degrees.
     * 
     * @return angle of rotor in degrees.
     */
    public double getNegatedRotorAngle() {
        return -getRotorAngle();
    }

    /**
     * Sets the angle of the rotor.
     * 
     * @param angle target rotor angle in degrees.
     */
    public void setRotorAngle(double angle) {
        controlState = ControlState.VELOCITY;

        // TODO(louis): Modulo required?
        // System.out.println("Q" + quadrant + ": current=" + getRotorAngle() + " target=" + angle);
        // outputRotorAngle = MathTools.wrapDegrees(angle) / SwerveConstants.CANCODER_TO_DEGREES;
        outputRotorAngle = angle / SwerveConstants.CANCODER_TO_DEGREES;
    }

    /**
     * Returns the current state of the swerve module.
     * 
     * @return state of swerve module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getMotorVelocity(), Rotation2d.fromDegrees(getRotorAngle()));
    }

    /**
     * Sets the target state of the module.
     * 
     * @param targetState target state of the module
     */
    public void setTargetState(SwerveModuleState targetState) {
        setTargetState(targetState, false, true);
    }

    /**
     * Sets the target state of the swerve module to the provided one.
     * 
     * @param targetState the target state
     * @param ignoreAngle whether to ignore the target angle
     * @param optimize whether to optimize the target angle
     */
    public void setTargetState(SwerveModuleState targetState, boolean ignoreAngle, boolean optimize) {
        SwerveModuleState state = targetState;
        if (optimize) {
            // Optimize the reference state to avoid spinning further than 90 degrees
            state =
                SwerveModuleState.optimize(targetState, Rotation2d.fromDegrees(MathTools.wrapDegrees(getRotorAngle())));
        }
        // System.out.println("Q" + quadrant + ": state=" + state);
        setMotorVelocity(state.speedMetersPerSecond);
        if (!ignoreAngle) {
            setRotorAngle(state.angle.getDegrees());
        }
    }

    /**
     * Returns the smallest rotor angle error.
     * 
     * @return error in degrees
     */
    public double getRotorAngleError() {
        double error = (rotorExternalEncoder.getPosition() - outputRotorAngle * SwerveConstants.CANCODER_TO_DEGREES) % 360.0;
        if (Math.abs(error) > 180) {
            if (error > 0) {
                return error - 180;
            } else {
                return error + 180;
            }
        }
        return error;
    }

    public void testMotorAndRotor(double motorOutput, double rotorOutput) {
        controlState = ControlState.TESTING;

        outputMotorVelocity = motorOutput / (SwerveConstants.MOTOR_TICKS_TO_METERS * 10.0);
        outputRotorAngle = MathTools.wrapDegrees(rotorOutput) / SwerveConstants.CANCODER_TO_DEGREES;
        // if (quadrant == 1) {
        //     System.out.println("Target angle: " + outputRotorAngle + ", actual: " + (getRotorAngle() / 360.0 * 4096));
        // }
    }

    /**
     * Resets the sensor(s) to zero.
     */
    @Override
    public void zero() {
        motor.setSelectedSensorPosition(0, SwerveConstants.PID_PRIMARY_SLOT, Constants.TIMEOUT_MS);
        double abs = rotorExternalEncoder.getAbsolutePosition();
        System.out.println("Q" + quadrant + " abs angle=" + abs + ", forward=" + forwardAngle);
        rotorExternalEncoder.setPosition(MathTools.wrapDegrees(abs - forwardAngle));
    }

    /**
     * Stops the swerve module motor while maintaining rotor position.
     */
    @Override
    public void stop() {
        outputMotorVelocity = 0.0;
        outputRotorAngle = getRotorAngle();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", this::getNegatedRotorAngle, null);
    }
}
