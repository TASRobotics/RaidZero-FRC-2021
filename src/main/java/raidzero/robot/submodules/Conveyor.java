package raidzero.robot.submodules;

import raidzero.robot.wrappers.LazyCANSparkMax;
import raidzero.robot.wrappers.LazyTalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
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

    private LazyTalonFX conveyorMotor;

    private double outputOpenLoop = 0.0;

    @Override
    public void onInit() {
        conveyorMotor = new LazyTalonFX(ConveyorConstants.MOTOR_ID);
        conveyorMotor.configFactoryDefault();
        conveyorMotor.setNeutralMode(ConveyorConstants.NEUTRAL_MODE);
        conveyorMotor.setInverted(ConveyorConstants.MOTOR_INVERSION);
        conveyorMotor.setSensorPhase(ConveyorConstants.SENSOR_PHASE);

        // TODO(jimmy): Tune PID constants
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        config.slot0.kF = ConveyorConstants.KF;
        config.slot0.kP = ConveyorConstants.KP;
        config.slot0.kI = ConveyorConstants.KI;
        config.slot0.kD = ConveyorConstants.KD;
        config.slot0.integralZone = ConveyorConstants.IZONE;

        conveyorMotor.configAllSettings(config);
        conveyorMotor.selectProfileSlot(0, 0);

        SupplyCurrentLimitConfiguration currentConfig = new SupplyCurrentLimitConfiguration(true, 35, 35, 0);
        conveyorMotor.configSupplyCurrentLimit(currentConfig);

    }

    @Override
    public void onStart(double timestamp) {
        outputOpenLoop = 0.0;
    }

    @Override
    public void run() {
        conveyorMotor.set(ControlMode.PercentOutput,  outputOpenLoop);//*ConveyorConstants.MAXSPEED);
    }

    @Override
    public void stop() {
        outputOpenLoop = 0.0;
        conveyorMotor.set(ControlMode.PercentOutput, 0.0);
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
