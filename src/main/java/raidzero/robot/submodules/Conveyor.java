package raidzero.robot.submodules;

import raidzero.robot.wrappers.LazyCANSparkMax;
import raidzero.robot.wrappers.LazyTalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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

        // TODO(jimmy): Tune PID constants
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        config.slot0.kF = ConveyorConstants.KF;
        config.slot0.kP = ConveyorConstants.KP;
        config.slot0.kI = ConveyorConstants.KI;
        config.slot0.kD = ConveyorConstants.KD;
        config.slot0.integralZone = ConveyorConstants.IZONE;

        conveyorMotor.configAllSettings(config);

    }

    @Override
    public void onStart(double timestamp) {
        outputOpenLoop = 0.0;
    }

    @Override
    public void run() {
        conveyorMotor.set(ControlMode.Velocity,  outputOpenLoop*ConveyorConstants.MAXSPEED);
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

    public boolean upToSpeed() {
        return Math.abs( outputOpenLoop - (conveyorMotor.getSelectedSensorVelocity(0) / ConveyorConstants.MAXSPEED)) < 0.3 ;
    }
}
