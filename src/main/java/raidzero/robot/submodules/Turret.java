package raidzero.robot.submodules;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import raidzero.robot.wrappers.LazyCANSparkMax;

import raidzero.robot.Constants.TurretConstants;

public class Turret extends Submodule {

    /**
     * 63:1
     */
    public static enum ControlState {
        OPEN_LOOP, POSITION
    };

    private static Turret instance = null;

    public static Turret getInstance() {
        if (instance == null) {
            instance = new Turret();
        }
        return instance;
    }

    private Turret() {
    }

    private LazyCANSparkMax turretMotor;
    private CANPIDController turretPidController;

    private double outputOpenLoop = 0.0;
    private double outputPosition = 0.0;
    private ControlState controlState = ControlState.OPEN_LOOP;

    @Override
    public void onInit() {
        turretMotor = new LazyCANSparkMax(TurretConstants.MOTOR_ID, MotorType.kBrushless);
        turretMotor.restoreFactoryDefaults();
        turretMotor.setIdleMode(TurretConstants.NEUTRAL_MODE);
        turretMotor.setInverted(TurretConstants.INVERSION);
        
        turretPidController = turretMotor.getPIDController();

        // TODO(jimmy): Tune PID constants
        // turretPidController.setReference(0, ControlType.kVelocity);
        turretPidController.setFF(TurretConstants.KF);
        turretPidController.setP(TurretConstants.KP);
        turretPidController.setI(TurretConstants.KI);
        turretPidController.setD(TurretConstants.KD);
        turretPidController.setIZone(TurretConstants.IZONE);
        turretPidController.setOutputRange(TurretConstants.MINOUT, TurretConstants.MAXOUT);
        turretPidController.setFeedbackDevice(turretMotor.getEncoder());
    }

    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = 0.0;
        outputPosition = 0.0;
        zero();
    }

    @Override
    public void update(double timestamp) {
        if (turretMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed).get()) {
            zero();
        }
    }

    @Override
    public void run() {
        switch (controlState) {
            case OPEN_LOOP:
                turretMotor.set(outputOpenLoop);
                break;
            case POSITION:
                turretPidController.setReference(outputPosition, ControlType.kPosition);
                break;
        }
    }

    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = 0.0;
        outputPosition = 0.0;
        turretMotor.set(0);
    }

    @Override
    public void zero() {
        turretMotor.getEncoder();   
    }

    /**
     * Rotates the turret to the specified angle using closed-loop control.
     * 
     * @param angle the angle to rotate to
     */
    public void rotateToAngle(double angle) {
        controlState = ControlState.POSITION;
        outputPosition = -angle * TurretConstants.TICKS_PER_DEGREE;
    }

    /**
     * Rotates the turret using open-loop control.
     * 
     * Note: Positive (+) is clockwise
     * 
     * @param percentOutput the percent output in [-1, 1]
     */
    public void rotateManual(double percentOutput) {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = percentOutput * TurretConstants.MANUAL_COEF;
    }

    public boolean isInOpenLoop() {
        return controlState == ControlState.OPEN_LOOP;
    }

    public boolean isAtPosition() {
        return controlState == ControlState.POSITION &&
               Math.abs(turretMotor.getEncoder().getPosition()) < TurretConstants.TOLERANCE;
    }
}
