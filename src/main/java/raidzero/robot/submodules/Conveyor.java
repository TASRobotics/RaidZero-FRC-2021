package raidzero.robot.submodules;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
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

    private CANSparkMax conveyorMotor;
    private CANPIDController conveyorPidController;

    private double outputOpenLoop = 0.0;
    private final double constantthingweshouldsetlater = 10;

    @Override
    public void onInit() {
        conveyorMotor = new CANSparkMax(ConveyorConstants.MOTOR_ID, MotorType.kBrushless);
        conveyorMotor.restoreFactoryDefaults();
        conveyorMotor.setIdleMode(ConveyorConstants.NEUTRAL_MODE);
        conveyorMotor.setInverted(ConveyorConstants.MOTOR_INVERSION);

        conveyorPidController = conveyorMotor.getPIDController();

        // TODO(jimmy): Tune PID constants
        conveyorPidController.setReference(0, ControlType.kVelocity);
        conveyorMotor.pidWrite(outputOpenLoop * constantthingweshouldsetlater);
        conveyorPidController.setP(0.14);
        conveyorPidController.setIZone(0);
        conveyorPidController.setD(0.01);
        conveyorPidController.setI(0);
        conveyorPidController.setFF(0);
        conveyorPidController.setOutputRange(-1, 1);
        conveyorPidController.setFeedbackDevice(conveyorMotor.getEncoder());
    }

    @Override
    public void onStart(double timestamp) {
        outputOpenLoop = 0.0;
    }

    @Override
    public void run() {
        // conveyorMotor.getPIDController().setReference(outputOpenLoop *
        // constantthingweshouldsetlater, ControlType.kVelocity);
        conveyorMotor.set(outputOpenLoop);
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

}
