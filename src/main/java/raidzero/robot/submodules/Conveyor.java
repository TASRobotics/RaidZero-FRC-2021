package raidzero.robot.submodules;

import raidzero.robot.wrappers.LazyCANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import raidzero.robot.Constants.ConveyorConstants;

public class Conveyor extends Submodule {

    private static Conveyor instance = null;

    public static Conveyor getInstance() {
        if (instance == null) {
            instance = new Conveyor();
        }
        return instance;
    }

    private Conveyor() {
    }

    private LazyCANSparkMax conveyorMotor;
    private CANPIDController conveyorPidController;

    private double outputOpenLoop = 0.0;

    @Override
    public void onInit() {
        conveyorMotor = new LazyCANSparkMax(ConveyorConstants.MOTOR_ID, MotorType.kBrushless);
        conveyorMotor.restoreFactoryDefaults();
        conveyorMotor.setIdleMode(ConveyorConstants.NEUTRAL_MODE);
        conveyorMotor.setInverted(ConveyorConstants.MOTOR_INVERSION);

        conveyorPidController = conveyorMotor.getPIDController();

        // TODO(jimmy): Tune PID constants
        conveyorPidController.setReference(0, ControlType.kVelocity);
        conveyorPidController.setFF(ConveyorConstants.KF);
        conveyorPidController.setP(ConveyorConstants.KP);
        conveyorPidController.setI(ConveyorConstants.KI);
        conveyorPidController.setD(ConveyorConstants.KD);
        conveyorPidController.setIZone(ConveyorConstants.IZONE);
        conveyorPidController.setOutputRange(ConveyorConstants.MINOUT, ConveyorConstants.MAXOUT);
        conveyorPidController.setFeedbackDevice(conveyorMotor.getEncoder());
    }

    @Override
    public void onStart(double timestamp) {
        outputOpenLoop = 0.0;
    }

    @Override
    public void run() {
        conveyorPidController.setReference(outputOpenLoop * ConveyorConstants.MAXRPM, ControlType.kVelocity);
    }

    @Override
    public void stop() {
        outputOpenLoop = 0.0;
        conveyorMotor.set(0.0);
    }

    /**
     * Spins the conveyor using open-loop control
     * 
     * @param percentOutput the percent output is [-1, 1]
     */
    public void moveBalls(double output) {
        outputOpenLoop = output;
    }

    public boolean upToSpeed() {
        return Math.abs( outputOpenLoop - (conveyorMotor.getEncoder().getVelocity() / ConveyorConstants.MAXRPM) ) < 0.3 ;
    }
}
