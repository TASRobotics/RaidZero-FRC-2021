package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.PWM;
import raidzero.robot.Constants.SpindexerConstants;
import raidzero.robot.wrappers.LazyTalonSRX;

public class Spindexer extends Submodule {

    private enum ControlState {
        OPEN_LOOP, VELOCITY
    };

    private static Spindexer instance = null;

    public static Spindexer getInstance() {
        if (instance == null) {
            instance = new Spindexer();
        }
        return instance;
    }

    private Spindexer() {
    }

    private LazyTalonSRX spindexerMotor;
    private PWM rampServo;

    private double outputOpenLoop = 0.0;
    private double outputVelocity = 0.0;
    private double outputServoPosition = SpindexerConstants.SERVO_DOWN;

    private ControlState controlState = ControlState.OPEN_LOOP;

    @Override
    public void onInit() {
        rampServo = new PWM(0);
        rampServo.setPosition(SpindexerConstants.SERVO_DOWN);
        //rampServo.setBounds(2.0, 1.6, 1.5, 1.4, 1.0);

        spindexerMotor = new LazyTalonSRX(SpindexerConstants.MOTOR_ID);
        spindexerMotor.configFactoryDefault();
        spindexerMotor.setNeutralMode(SpindexerConstants.NEUTRAL_MODE);
        spindexerMotor.setInverted(SpindexerConstants.MOTOR_INVERSION);
    }

    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;

        outputOpenLoop = SpindexerConstants.SERVO_DOWN;
        outputVelocity = 0.0;
    }

    @Override
    public void run() {
        rampServo.setPosition(outputServoPosition);
        switch (controlState) {
            case OPEN_LOOP:
                spindexerMotor.set(ControlMode.PercentOutput, outputOpenLoop);
                break;
            case VELOCITY:
                spindexerMotor.set(ControlMode.Velocity, outputVelocity);
                break;
        }
    }

    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = SpindexerConstants.SERVO_DOWN;
        outputVelocity = 0.0;
        spindexerMotor.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * Spins the spindexer using open-loop control
     * 
     * @param percentOutput the percent output is [-1, 1]
     */
    public void rotate(double percentOutput) {
        outputOpenLoop = percentOutput;
    }

    /**
     * Spins the spindexer using closed-loop velocity control
     * 
     * @param velocity the velocity in encoder ticks / 100ms
     */
    public void rotateAtVelocity(double velocity) {
        outputVelocity = velocity;
    }

    /** 
     * Moves the ramp up.
     */
    public void rampUp() {
        outputServoPosition = SpindexerConstants.SERVO_UP;
    }

    /**
     * Moves the ramp down.
     */
    public void rampDown() {
        outputServoPosition = SpindexerConstants.SERVO_DOWN;
    }

    /**
     * shoot ballz lols
     */
    public void shoot() {
        rotate(SpindexerConstants.SHOOT_SPEED);
    }
}
