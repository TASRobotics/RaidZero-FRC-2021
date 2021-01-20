package raidzero.robot.submodules;

import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.submodules.Submodule;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class SwerveModule extends Submodule {
    private static SwerveModule instance = null;

    public static SwerveModule getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }

    private SwerveModule {
    }

    private static enum MotorMode {
        POSITION, VELOCITY
    };

    private MotorMode runMode = MotorMode.VELOCITY;

    private TalonFX rotor;
    private TalonFX motor;

    private double motorPos = 0;
    private double motorVel = 0;
    private double rotorTargPos = 0;

    /**
     * Called at the start of autonomous or teleop.
     * 
     * @param timestamp
     */
    public void onStart(double timestamp) {}

    /**
     * Called once when the submodule is initialized.
     */
    public void onInit(int ids[]) {
        motor = new TalonFX(ids[0]);
        rotor = new TalonFX(ids[1]);
        
        motor.configFactoryDefault();
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        motor.selectProfileSlot(1,0);
        motor.config_kP(0, 0.3);
        motor.config_kD(0, 0.05);
        motor.config_kP(1, 0.05);
        motor.config_kD(1, 0.0);
        motor.configMotionAcceleration(100000);
        motor.configMotionCruiseVelocity(80000);

        rotor.configFactoryDefault();
        rotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rotor.selectProfileSlot(0,0);
        rotor.config_kP(0, 0.8);
        rotor.config_kD(0, 1.3);
        rotor.configMotionAcceleration(100000);
        rotor.configMotionCruiseVelocity(80000);
        rotor.setSelectedSensorPosition(0);
    }

    /**
     * Reads cached inputs & calculate outputs.
     */
    public void update(double timestamp) {}
    
    /**
     * Runs components in the submodule that have continuously changing 
     * inputs.
     */
    public void run() {}

    /**
     * Stops the submodule.
     */
    public abstract void stop();

    /**
     * Resets the sensor(s) to zero.
     */
    public void zero() {}


}