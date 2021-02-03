package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import raidzero.robot.Constants.ConveyorConstants;
import raidzero.robot.wrappers.LazyTalonSRX;

public class Conveyor extends Submodule {

    private static Conveyor instance = null;

    public static Conveyor getInstance() {
        if (instance == null) {
            instance = new Conveyor();
        }
        return instance;
    }

    private Conveyor() {}

    private CANSparkMax conveyorMotor;

    private double outputOpenLoop = 0.0;

    @Override
    public void onInit(){
        conveyorMotor = new CANSparkMax(ConveyorConstants.MOTOR_ID, MotorType.kBrushless);
        conveyorMotor.restoreFactoryDefaults();
        conveyorMotor.setIdleMode(ConveyorConstants.NEUTRAL_MODE);
        conveyorMotor.setInverted(ConveyorConstants.MOTOR_INVERSION);
    }

    @Override
    public void onStart(double timestamp){
        outputOpenLoop = 0.0;
    }

    @Override
    public void run(){
        conveyorMotor.set(outputOpenLoop);
    }

    @Override
    public void stop(){
        outputOpenLoop = 0.0;
        conveyorMotor.set(0.0);
    }

    /**
     * Spins the Conveyor using open-loop control
     * @param percentOutput the percent output is [-1, 1]
     */
    public void moveBalls(double percentOutput){
        outputOpenLoop = percentOutput;
    }
}