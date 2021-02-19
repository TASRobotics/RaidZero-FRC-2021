package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSensor;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
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
    private final double constantthingweshouldsetlater = 10;

    @Override
    public void onInit(){
        conveyorMotor = new CANSparkMax(ConveyorConstants.MOTOR_ID, MotorType.kBrushless);
        conveyorMotor.restoreFactoryDefaults();
        conveyorMotor.setIdleMode(ConveyorConstants.NEUTRAL_MODE);
        conveyorMotor.setInverted(ConveyorConstants.MOTOR_INVERSION);

        //set Constants later
        conveyorMotor.getPIDController().setReference(0, ControlType.kVelocity);
        conveyorMotor.pidWrite(outputOpenLoop * constantthingweshouldsetlater);
        conveyorMotor.getPIDController().setP(0.14);
        conveyorMotor.getPIDController().setIZone(0);
        conveyorMotor.getPIDController().setD(0.01);
        conveyorMotor.getPIDController().setI(0);
        conveyorMotor.getPIDController().setFF(0);
        conveyorMotor.getPIDController().setOutputRange(-1,1);
        conveyorMotor.getPIDController().setFeedbackDevice(conveyorMotor.getEncoder());
    }

    @Override
    public void onStart(double timestamp){
        outputOpenLoop = 0.0;
    }

    @Override
    public void run(){
        //conveyorMotor.getPIDController().setReference(outputOpenLoop * constantthingweshouldsetlater, ControlType.kVelocity);
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
    public void moveBalls(double output){
        outputOpenLoop = output;
    }
}