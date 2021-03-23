package raidzero.robot.submodules;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.wrappers.LazyCANSparkMax;
import raidzero.robot.wrappers.LazyTalonSRX;
import raidzero.robot.Constants.HoodConstants;
import raidzero.robot.Constants.HoodConstants.HoodAngle;
import raidzero.robot.dashboard.Tab;
import org.apache.commons.math3.analysis.function.Logistic;

public class AdjustableHood extends Submodule {

    private enum ControlState {
        OPEN_LOOP, POSITION
    };

    private static Logistic hoodAreaFit = new Logistic(HoodConstants.LOGISTFIT[2],
        HoodConstants.LOGISTFIT[0],HoodConstants.LOGISTFIT[1],0,0,1);
    private static AdjustableHood instance = null;

    public static AdjustableHood getInstance() {
        if (instance == null) {
            instance = new AdjustableHood();
        }
        return instance;
    }

    private AdjustableHood() {
    }

    private LazyCANSparkMax hoodMotor;
    private CANPIDController pidController;
    private CANEncoder encoder;
    private CANDigitalInput reverseLimitSwitch;
    private CANDigitalInput forwardLimitSwitch;

    private double outputOpenLoop = 0.0;
    private double outputPosition = 0.0;

    private ControlState controlState = ControlState.OPEN_LOOP;

    private NetworkTableEntry hoodPositionEntry =
            Shuffleboard.getTab(Tab.MAIN).add("Hood Position", 0).withWidget(BuiltInWidgets.kDial)
                    .withProperties(Map.of("min", 0, "max", 83)).withSize(2, 2).withPosition(0, 0)
                    .getEntry();

    @Override
    public void onInit() {
        hoodMotor = new LazyCANSparkMax(HoodConstants.MOTOR_ID, MotorType.kBrushless);
        hoodMotor.restoreFactoryDefaults();
        hoodMotor.setIdleMode(HoodConstants.IDLE_MODE);
        hoodMotor.setInverted(HoodConstants.INVERSION);

        encoder = hoodMotor.getEncoder();

        forwardLimitSwitch = hoodMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
        reverseLimitSwitch = hoodMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
        forwardLimitSwitch.enableLimitSwitch(true);
        reverseLimitSwitch.enableLimitSwitch(true);

        pidController = hoodMotor.getPIDController();
        pidController.setP(HoodConstants.K_P);
        pidController.setI(HoodConstants.K_I);
        pidController.setD(HoodConstants.K_D);
        pidController.setFF(HoodConstants.K_F);
    }

    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;

        outputOpenLoop = 0.0;
        outputPosition = 0.0;
    }

    @Override
    public void update(double timestamp) {
        if (reverseLimitSwitch.get()) {
            zero();
            // System.out.println("zeroed");
        }
        SmartDashboard.putNumber("Hood Angle", encoder.getPosition());
        hoodPositionEntry.setNumber(encoder.getPosition());
    }

    @Override
    public void run() {
        switch (controlState) {
            case OPEN_LOOP:
                pidController.setReference(outputOpenLoop, ControlType.kDutyCycle);
                break;
            case POSITION:
                pidController.setReference(outputPosition, ControlType.kPosition);
                break;
        }
    }

    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = 0.0;
        outputPosition = 0.0;
        pidController.setReference(0.0, ControlType.kDutyCycle);
    }

    @Override
    public void zero() {
        encoder.setPosition(0.0);
    }

    /**
     * Returns the position of the hood.
     * 
     * @return position in encoder ticks
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    /**
     * Adjusts the hood using open-loop control.
     * 
     * @param percentOutput the percent output in [-1, 1]
     */
    public void adjust(double percentOutput) {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = percentOutput;
    }

    /**
     * Moves the hood to a position using closed-loop control.
     * 
     * @param position position in encoder units
     */
    public void moveToTick(double position) {
        controlState = ControlState.POSITION;
        outputPosition = position;
    }

    /**
     *  Automatically adjusts the hood based on
     *  the area of the target.
     * @param targetArea  the area of the target
     */

    public void autoPosition(double targetArea) {
        outputPosition = hoodAreaFit.value(targetArea);
    }

    /**
     * Moves to hood to a specific hood angle.
     * 
     * @param angle hood angle to move to
     */
    public void moveToAngle(HoodAngle angle) {
        moveToTick(angle.ticks);
    }

    /**
     * Returns whether the hood is at the target position in the position control mode.
     * 
     * @return if the hood is at the target position
     */
    public boolean isAtPosition() {
        return controlState == ControlState.POSITION
                && Math.abs(outputPosition - encoder.getPosition()) < HoodConstants.TOLERANCE;
    }
}
