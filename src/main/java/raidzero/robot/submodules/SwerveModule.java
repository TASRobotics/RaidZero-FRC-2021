package raidzero.robot.submodules;

import raidzero.pathgen.PathPoint;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.dashboard.Tab;
import raidzero.robot.pathing.HolonomicProfileFollower;
import raidzero.robot.pathing.Path;
import raidzero.robot.utils.EncoderUtils;
import raidzero.robot.wrappers.LazyTalonFX;

import java.util.Map;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.CANCoder;

import org.apache.commons.math3.util.FastMath;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SwerveModule extends Submodule {

    private enum ControlState {
        POSITION, VELOCITY, PATHING
    };

    public static class TargetPolarityTuple {
        public double target;
        public boolean polarity;

        public TargetPolarityTuple(double target, boolean polarity) {
            this.target = target;
            this.polarity = polarity;
        }
    }

    public LazyTalonFX motor;
    public LazyTalonFX rotor;

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

    private double currentRotorPositionTicks = 0.0;

    private ControlState controlState = ControlState.VELOCITY;
    private HolonomicProfileFollower profileFollower;

    private NetworkTableEntry rotorAngleEntry;
    private NetworkTableEntry rotorTicksEntry;
    private NetworkTableEntry motorVelocityEntry;

    @Override
    public void onInit() {
        throw new RuntimeException("Cannot initialize SwerveModule by itself.");
    }

    /**
     * Called once when the submodule is initialized.
     */
    public void onInit(int motorId, int rotorId, double initialAngle, int quadrant) {
        this.quadrant = quadrant;

        motor = new LazyTalonFX(motorId);
        rotor = new LazyTalonFX(rotorId);
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

        int column = 0;
        int row = 0;
        if (quadrant == 1) {
            column = 3;
            row = 0;
        } else if (quadrant == 2) {
            column = 2;
            row = 0;
        } else if (quadrant == 3) {
            column = 2;
            row = 1;
        } else {
            column = 3;
            row = 1;
        }

        rotorAngleEntry = Shuffleboard.getTab(Tab.MAIN).add("Rotor" + quadrant, 0)
                .withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -0.5, "max", 0))
                .withSize(1, 1).withPosition(column, row).getEntry();
        rotorTicksEntry = Shuffleboard.getTab(Tab.MAIN).add("RotorTick" + quadrant, 0)
                .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(column + 2, row).getEntry();
        motorVelocityEntry = Shuffleboard.getTab(Tab.MAIN).add("Motor" + quadrant, 0)
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1.0, "max", 1.0))
                .withSize(1, 1).withPosition(column + 4, row).getEntry();

        stop();
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
        // motor.clearMotionProfileTrajectories();
        motor.selectProfileSlot(SwerveConstants.MOTOR_VELOCITY_SLOT,
                SwerveConstants.PID_PRIMARY_SLOT);
        // rotor.clearMotionProfileTrajectories();
    }

    /**
     * Reads cached inputs & calculate outputs.
     */
    public void update(double timestamp) {
        if (controlState == ControlState.PATHING) {
            // System.out.println("updating");
            profileFollower.update();
            outputMotorProfile = profileFollower.getMotorOutput();
            outputRotorProfile = profileFollower.getRotorOutput();
            // System.out.println("MP: " + outputMotorProfile + " | RP: " + outputRotorProfile);
        }
        currentRotorPositionTicks = rotor.getSelectedSensorPosition();
        // System.out.println("Q" + quadrant + ": pos=" + getRotorPosition() * SwerveConstants.ROTOR_REVOLUTION_RATIO + " target=" + outputRotorPosition * SwerveConstants.ROTOR_REVOLUTION_RATIO);
        rotorAngleEntry.setDouble(-((1 + (getRotorPosition() % 1)) % 0.5));
        rotorTicksEntry.setDouble(currentRotorPositionTicks);
        motorVelocityEntry.setDouble(getMotorVelocity());
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

    public void setRotorPos(double pos) {
        setRotorPos(pos, true);
    }

    /**
     * Sets the rotor to a position.
     * 
     * @param pos the position to go to in units of degrees
     */
    public void setRotorPos(double pos, boolean optimize) {
        // convert degrees to revolutions
        pos /= 360.0;

        // get the difference in angle
        double cPos = getRotorPosition();
        dPos = pos - cPos;
        // get the positive adjusted angle
        dPos = dPos % 1; // |dpos| <= 1
        dPos += (dPos < -0.25 ? 1 : 0); // dPos >= -0.25
        dPos -= (dPos >= 0.75 ? 1 : 0); // dPos < 0.75

        if (!optimize) {
            angleAdjustmentMotorPolarity = false;
            outputRotorPosition = dPos + cPos;
            return;
        }

        if (dPos > 0.25) {
            dPos -= 0.5;
            angleAdjustmentMotorPolarity = true;
            outputRotorPosition = dPos + cPos;
            return;
        }

        angleAdjustmentMotorPolarity = false;
        outputRotorPosition = dPos + cPos;
    }

    public boolean getMotorPolarity() {
        return angleAdjustmentMotorPolarity;
    }

    // Revert this
    public TargetPolarityTuple setRotorPosWithOutputs(double pos, boolean optimize) {
        setRotorPos(pos, optimize);
        return new TargetPolarityTuple(outputRotorPosition, angleAdjustmentMotorPolarity);
    }

    public void setMotorPosition(double position) {
        setControlState(ControlState.POSITION);
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
                * SwerveConstants.MAX_MOTOR_SPEED_DRIVING);
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
        outputRotorPosition = getRotorPosition();
        outputMotorProfile = SetValueMotionProfile.Disable.value;
        outputRotorProfile = SetValueMotionProfile.Disable.value;
        // System.out.println("Q" + quadrant + " Current: " + getRotorPosition() + " Target: " + outputRotorPosition);

        // var s = new MotionProfileStatus();
        // motor.getMotionProfileStatus(s);
        // System.out.println(
        //     "btmBuffer: " + s.btmBufferCnt + " topBuffer: " + s.topBufferCnt + " underrun: " + s.hasUnderrun + " isUnderrun: " + s.isUnderrun
        // );
        // System.out.println("motor clear: " + motor.clearMotionProfileTrajectories());
        // System.out.println("rotor clear: " + rotor.clearMotionProfileTrajectories());
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
        double newPosition = zeroDeg * (SwerveConstants.ROTOR_REVOLUTION_RATIO / 360.0);
        rotor.setSelectedSensorPosition(newPosition, 0, 20);
        currentRotorPositionTicks = newPosition;
        outputRotorPosition = zeroDeg / 360.0;
    }

    /**
     * Returns the motor velocity in units of fractions of max speed.
     * 
     * @return motor velocity in fractions of max speed
     */
    public double getMotorVelocity() {
        return motor.getSelectedSensorVelocity() / SwerveConstants.MAX_MOTOR_SPEED_TICKS;
    }

    /**
     * Returns the rotor position in units of revolutions.
     * 
     * @return rotor position in revolutions
     */
    public double getRotorPosition() {
        return currentRotorPositionTicks / SwerveConstants.ROTOR_REVOLUTION_RATIO;
    }

    /**
     * Executes a path using a holonomic profile follower.
     */
    public void pushPath(Path path) {
        if (controlState == ControlState.PATHING) {
            return;
        }
        stop();
        setControlState(ControlState.PATHING);
        double moduloed = currentRotorPositionTicks % SwerveConstants.ROTOR_REVOLUTION_RATIO;
        // System.out.println("From: " + currentRotorPositionTicks + " To: " + moduloed);

        rotor.setSelectedSensorPosition(moduloed);
        outputRotorPosition = (moduloed / SwerveConstants.ROTOR_REVOLUTION_RATIO + 1.0) % 1.0;
        // rotor.setSelectedSensorPosition(EncoderUtils.rotorDegreesToTicks(path.getPathPoints()[0].angle));
        // outputRotorPosition = path.getPathPoints()[0].angle / 360.0;
        // zeroRotor();

        System.out.println("Q" + quadrant + " path points:");
        PathPoint.printPathPoints(path.getPathPoints());
        System.out.println("=======================================");

        profileFollower.reset();
        profileFollower.start(path.getPathPoints());
    }

    public void enableProfile() {
        // System.out.println("Q" + quadrant + ": polarity=" + angleAdjustmentMotorPolarity + " mpc=" + motor.getMotionProfileTopLevelBufferCount());
        motor.setInverted(angleAdjustmentMotorPolarity);
        profileFollower.enable();
    }

    public boolean isDoneWaitingForFill() {
        return profileFollower.isDoneWaitingForFill();
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
        // System.out.println("Q" + quadrant + " finished? " + profileFollower.isFinished());
        return profileFollower.isFinished();
    }

}
