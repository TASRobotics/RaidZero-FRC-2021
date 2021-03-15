package raidzero.robot.submodules;

import raidzero.robot.wrappers.LazyTalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import raidzero.robot.Constants.IntakeConstants;

public class Intake extends Submodule {

    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
    }

    private LazyTalonFX intakeMotor;
    private boolean motorPolarityBuffer = true;

    private double outputOpenLoop = 0.0;

    @Override
    public void onInit() {
        intakeMotor = new LazyTalonFX(IntakeConstants.TOP_MOTOR_ID);
        intakeMotor.configFactoryDefault();
        intakeMotor.setNeutralMode(IntakeConstants.NEUTRAL_MODE);
        intakeMotor.setInverted(IntakeConstants.MOTOR_INVERSION);
    }

    @Override
    public void onStart(double timestamp) {
        outputOpenLoop = 0.0;
    }

    @Override
    public void run() {
        intakeMotor.set(ControlMode.PercentOutput, outputOpenLoop * (motorPolarityBuffer ? 1 : -1));
    }

    @Override
    public void stop() {
        outputOpenLoop = 0.0;
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * Spins the intake using open-loop control
     * 
     * @param percentOutput the percent output is [-1, 1]
     */
    public void intakeBalls(double percentOutput) {
        outputOpenLoop = percentOutput;
    }

    public void setMotorDirection(boolean in) {
        motorPolarityBuffer = in;
    }
}
