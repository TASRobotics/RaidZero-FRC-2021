package raidzero.robot.submodules;

import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.dashboard.Tab;
import raidzero.robot.pathing.HolonomicProfileFollower;
import raidzero.robot.pathing.Path;
import raidzero.robot.utils.EncoderUtils;

import java.util.Map;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import org.apache.commons.math3.util.FastMath;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SwerveModule extends Submodule {

    private enum ControlState {
        POSITION, VELOCITY, PATHING
    };

    private TalonFX motor;
    private TalonFX rotor;

    private CANCoder rotorExternalEncoder;

    // The default forward angle (from external rotor encoder)
    private double zeroAngle;

    // The quadrant the module is in (cartesian plane)
    private int quadrant;

    // Difference in rotor angle & target angle (in revolutions)
    private double dPos = 0;

    // Whether to reverse the motor becuase of constant angle adjustments
    private boolean angleAdjustmentMotorPolarity = false;

    private double outputMotorPosition = 0.0;
    private double outputMotorVelocity = 0.0;
    private double outputRotorPosition = 0;
    private int outputMotorProfile = 0;
    private int outputRotorProfile = 0;

    private ControlState controlState = ControlState.VELOCITY;
    private HolonomicProfileFollower profileFollower;

    private NetworkTableEntry positionEntry;

    @Override
    public void onInit() {
        throw new RuntimeException("Cannot initialize SwerveModule by itself.");
    }

    /**
     * Called once when the submodule is initialized.
     */
    public void onInit(int motorId, int rotorId, double initialAngle, int quadrant) {
        this.quadrant = quadrant;

        motor = new TalonFX(motorId);
        rotor = new TalonFX(rotorId);
        rotorExternalEncoder = new CANCoder(quadrant);
        profileFollower = new HolonomicProfileFollower(motor, rotor, 
            EncoderUtils::inchesToTicks, EncoderUtils::rotorDegreesToTicks);
        zeroAngle = initialAngle;

        motor.configFactoryDefault();
        rotor.setInverted(SwerveConstants.DEFAULT_MOTOR_INVERSION);
        motor.configSelectedFeedbackSensor(SwerveConstants.FEEDBACKDEVICE);
        motor.selectProfileSlot(SwerveConstants.MOTOR_VELOCITY_SLOT,
                SwerveConstants.PID_PRIMARY_SLOT);
        motor.config_kP(SwerveConstants.MOTOR_POSITION_SLOT, SwerveConstants.MOTOR_POSI_KP);
        motor.config_kD(SwerveConstants.MOTOR_POSITION_SLOT, SwerveConstants.MOTOR_POSI_KD);
        motor.config_kF(SwerveConstants.MOTOR_POSITION_SLOT, SwerveConstants.MOTOR_POSI_KF);
        motor.config_kP(SwerveConstants.MOTOR_VELOCITY_SLOT, SwerveConstants.MOTOR_VELO_KP);
        motor.config_kD(SwerveConstants.MOTOR_VELOCITY_SLOT, SwerveConstants.MOTOR_VELO_KD);
        motor.configMotionAcceleration(SwerveConstants.DEFAULT_TARG_ACCEL);
        motor.configMotionCruiseVelocity(SwerveConstants.DEFAULT_TARG_VELO);

        rotor.configFactoryDefault();
        rotor.setInverted(SwerveConstants.ROTOR_INVERSION);
        rotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rotor.selectProfileSlot(SwerveConstants.ROTOR_PID_SLOT, SwerveConstants.PID_PRIMARY_SLOT);
        rotor.config_kP(SwerveConstants.ROTOR_PID_SLOT, SwerveConstants.ROTOR_KP);
        rotor.config_kD(SwerveConstants.ROTOR_PID_SLOT, SwerveConstants.ROTOR_KD);
        rotor.config_kI(SwerveConstants.ROTOR_PID_SLOT, SwerveConstants.ROTOR_KI);
        rotor.config_IntegralZone(SwerveConstants.ROTOR_PID_SLOT, SwerveConstants.ROTOR_IZONE);
        rotor.configMotionAcceleration(SwerveConstants.ROTOR_TARG_ACCEL);
        rotor.configMotionCruiseVelocity(SwerveConstants.ROTOR_TARG_VELO);

        positionEntry = Shuffleboard.getTab(Tab.MAIN).add("Module" + quadrant, 0)
                .withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -0.5, "max", 0))
                .withSize(2, 2).withPosition(0, 0).getEntry();

        zero();
    }

    /**
     * Called at the start of autonomous or teleop.
     * 
     * @param timestamp
     */
    @Override
    public void onStart(double timestamp) {
        outputMotorPosition = 0.0;
        outputMotorVelocity = 0.0;
        outputRotorPosition = 0.0;
        outputMotorProfile = 0;
        outputRotorProfile = 0;
        motor.clearMotionProfileTrajectories();
        motor.selectProfileSlot(SwerveConstants.MOTOR_VELOCITY_SLOT,
                SwerveConstants.PID_PRIMARY_SLOT);
        rotor.clearMotionProfileTrajectories();
    }

    /**
     * Reads cached inputs & calculate outputs.
     */
    public void update(double timestamp) {
        if (controlState == ControlState.PATHING) {
            System.out.println("updating");
            profileFollower.update();
            outputMotorProfile = profileFollower.getMotorOutput();
            outputRotorProfile = profileFollower.getRotorOutput();
            // System.out.println("MP: " + outputMotorProfile + " | RP: " + outputRotorProfile);
        }
        positionEntry.setDouble(-((1 + (getRotorPosition() % 1)) % 0.5));
    }

    /**
     * Runs components in the submodule that have continuously changing inputs.
     */
    public void run() {
        switch (controlState) {
            case POSITION:
                motor.set(ControlMode.MotionMagic, outputMotorPosition);
                rotor.set(ControlMode.MotionMagic,
                    outputRotorPosition * SwerveConstants.ROTOR_REVOLUTION_RATIO);
                break;
            case VELOCITY:
                motor.set(ControlMode.Velocity, outputMotorVelocity);
                rotor.set(ControlMode.MotionMagic,
                    outputRotorPosition * SwerveConstants.ROTOR_REVOLUTION_RATIO);
                break;
            case PATHING:
                motor.set(ControlMode.MotionProfile, outputMotorProfile);
                rotor.set(ControlMode.MotionProfile, outputRotorProfile);
                break;
        }
    }

    /**
     * Sets the control state with additional changes for closed-loop profiles.
     * 
     * @param state the new control state
     */
    public void setControlState(ControlState state) {
        if (state == controlState) {
            return;
        }
        controlState = state;
        switch (controlState) {
            case POSITION:
                motor.selectProfileSlot(SwerveConstants.MOTOR_POSITION_SLOT,
                        SwerveConstants.PID_PRIMARY_SLOT);
                break;
            case VELOCITY:
                motor.selectProfileSlot(SwerveConstants.MOTOR_VELOCITY_SLOT,
                        SwerveConstants.PID_PRIMARY_SLOT);
                break;
            default:
                break;
        }
    }

    /**
     * Sets the rotor to a position.
     * 
     * @param pos the position to go to in units of degrees
     * 
     * @return the new target rotor position in units of revolutions
     */
    public double setRotorPos(double pos) {
        // convert degrees to revolutions
        pos /= 360.0;

        // get the difference in angle
        double cpos = getRotorPosition();
        dPos = pos - cpos;
        // get the positive adjusted angle
        dPos = dPos % 1; // |dpos| <= 1
        dPos += (dPos < -0.25 ? 1 : 0); // dPos >= -0.25
        dPos -= (dPos >= 0.75 ? 1 : 0); // dPos < 0.75

        if (dPos > 0.25) {
            dPos -= 0.5;
            angleAdjustmentMotorPolarity = true;
            outputRotorPosition = dPos + cpos;
            return outputRotorPosition;
        }

        angleAdjustmentMotorPolarity = false;
        outputRotorPosition = dPos + cpos;
        return outputRotorPosition;
    }

    public void setMotorPosition(double position) {
        setControlState(ControlState.POSITION);

        // TODO(jimmy): Sort out reversing
        outputMotorPosition = position;
    }

    /**
     * Sets the velocity of the motor to a target velocity.
     * 
     * @param velocity target velocity in encoder ticks / 100ms
     */
    public void setMotorVelocity(double velocity) {
        setControlState(ControlState.VELOCITY);

        motor.setInverted(angleAdjustmentMotorPolarity);
        outputMotorVelocity = velocity * FastMath.pow(1 - Math.abs(dPos), 4);
    }

    /**
     * Sets the module to a 2-D target velocity. Input array should have magnitude less than 1
     */
    public void setVectorVelocity(double[] normalizedV, double speedLimit) {
        // set the velocity to the magnitude of vector v scaled to the maximum desired speed
        setMotorVelocity(speedLimit * FastMath.hypot(normalizedV[0], normalizedV[1])
                * SwerveConstants.MAX_MOTOR_SPEED);
        // set rotor to the theta of vector v if the magnitude of the vector is not too small
        if (Math.abs(outputMotorVelocity) < 0.1) {
            return;
        }
        setRotorPos(FastMath.toDegrees(FastMath.atan2(normalizedV[1], normalizedV[0])));
    }

    /**
     * Resets the sensor(s) to zero.
     */
    @Override
    public void zero() {
        outputMotorProfile = SetValueMotionProfile.Disable.value;
        outputRotorProfile = SetValueMotionProfile.Disable.value;
        outputRotorPosition = 0.0;
        zeroMotor();
        zeroRotor();
    }

    /**
     * Stops the swerve module motor while maintaining rotor position.
     */
    @Override
    public void stop() {
        setControlState(ControlState.VELOCITY);
        outputMotorVelocity = 0.0;
        outputMotorProfile = SetValueMotionProfile.Disable.value;
        outputRotorProfile = SetValueMotionProfile.Disable.value;

        var s = new MotionProfileStatus();
        motor.getMotionProfileStatus(s);
        System.out.println(
            "btmBuffer: " + s.btmBufferCnt + " topBuffer: " + s.topBufferCnt + " underrun: " + s.hasUnderrun + " isUnderrun: " + s.isUnderrun
        );
        System.out.println("motor clear: " + motor.clearMotionProfileTrajectories());
        System.out.println("rotor clear: " + rotor.clearMotionProfileTrajectories());
    }

    /**
     * Zeroes the encoder of the motor.
     */
    public void zeroMotor() {
        motor.setSelectedSensorPosition(0);
    }

    /**
     * Zeroes the internal encoder of the rotor to the absolute position from the external encoder.
     */
    public void zeroRotor() {
        double zeroDeg = (rotorExternalEncoder.getAbsolutePosition() - zeroAngle + 360) % 360;
        rotor.setSelectedSensorPosition(
                zeroDeg * (SwerveConstants.ROTOR_REVOLUTION_RATIO / 360.0),
                0, 20);

        outputRotorPosition = zeroDeg / 360.0;
    }

    /**
     * Returns the rotor position in units of revolutions.
     * 
     * @return rotor position in revolutions
     */
    public double getRotorPosition() {
        return rotor.getSelectedSensorPosition(0) / SwerveConstants.ROTOR_REVOLUTION_RATIO;
    }

    /**
     * Executes a path using a holonomic profile follower.
     */
    public void executePath(Path path) {
        if (controlState == ControlState.PATHING) {
            return;
        }
        setControlState(ControlState.PATHING);
        profileFollower.reset();
        profileFollower.start(path.getPathPoints());
    }

    /**
     * Returns whether the swerve has finished following a path.
     * 
     * @return if the drive is finished pathing
     */
    public boolean isFinishedWithPath() {
        if (profileFollower == null || controlState != ControlState.PATHING) {
            return false;
        }
        System.out.println("Q" + quadrant + " finished? " + profileFollower.isFinished());
        return profileFollower.isFinished();
    }

}
