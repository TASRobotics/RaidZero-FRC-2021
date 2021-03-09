package raidzero.robot.submodules;

import raidzero.robot.wrappers.LazyCANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

    private LazyCANSparkMax topIntakeMotor;
    private LazyCANSparkMax bottomIntakeMotor;
    private boolean motorPolarityBuffer = true;

    private double outputOpenLoop = 0.0;

    @Override
    public void onInit() {
        topIntakeMotor = new LazyCANSparkMax(IntakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
        topIntakeMotor.restoreFactoryDefaults();
        topIntakeMotor.setIdleMode(IntakeConstants.TOP_NEUTRAL_MODE);
        topIntakeMotor.setInverted(IntakeConstants.TOP_MOTOR_INVERSION);

        bottomIntakeMotor = new LazyCANSparkMax(IntakeConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
        bottomIntakeMotor.restoreFactoryDefaults();
        bottomIntakeMotor.setIdleMode(IntakeConstants.BOTTOM_NEUTRAL_MODE);
        bottomIntakeMotor.setInverted(IntakeConstants.BOTTOM_MOTOR_INVERSION);
    }

    @Override
    public void onStart(double timestamp) {
        outputOpenLoop = 0.0;
    }

    @Override
    public void run() {
        topIntakeMotor.set(outputOpenLoop * (motorPolarityBuffer ? 1 : -1));
        bottomIntakeMotor.set(outputOpenLoop * (motorPolarityBuffer ? 1 : -1));
    }

    @Override
    public void stop() {
        outputOpenLoop = 0.0;
        topIntakeMotor.set(0.0);
        bottomIntakeMotor.set(0.0);
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
