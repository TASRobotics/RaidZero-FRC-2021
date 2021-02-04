package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import raidzero.robot.Constants.SpindexerConstants;
import raidzero.robot.wrappers.LazyTalonSRX;

public class Spindexer extends Submodule {

    private static Spindexer instance = null;

    public static Spindexer getInstance() {
        if (instance == null) {
            instance = new Spindexer();
        }
        return instance;
    }

    private Spindexer() {}

    private LazyTalonSRX spindexerMotor;
    private PWM rampServo;

    private double outputOpenLoop = 0.0;
    private double outputServoPosition = 0.15;

    @Override
    public void onInit(){
        rampServo = new PWM(9);
        rampServo.setPosition(0.15);
        rampServo.setBounds(2.0, 1.6, 1.5, 1.4, 1.0);
        spindexerMotor = new LazyTalonSRX(SpindexerConstants.MOTOR_ID);
        spindexerMotor.configFactoryDefault();
        spindexerMotor.setNeutralMode(SpindexerConstants.NEUTRAL_MODE);
        spindexerMotor.setInverted(SpindexerConstants.MOTOR_INVERSION);
    }

    @Override
    public void onStart(double timestamp){
        outputOpenLoop = 0.0;
    }

    @Override
    public void run(){
        rampServo.setPosition(outputServoPosition);
        spindexerMotor.set(ControlMode.PercentOutput, outputOpenLoop);
    }

    @Override
    public void stop(){
        outputOpenLoop = 0.0;
        spindexerMotor.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * Spins the Spindexer using open-loop control
     * @param percentOutput the percent output is [-1, 1]
     */
    public void rotate(double percentOutput){
        outputOpenLoop = percentOutput;
    }

    public void rampUp() {
        outputServoPosition = 1.0;
    }

    public void rampDown() {
        outputServoPosition = 0.15;
    }
}