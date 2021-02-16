package raidzero.robot.submodules;

import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.pathing.HolonomicProfileFollower;
import raidzero.robot.pathing.Path;
import raidzero.robot.pathing.ProfileFollower;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;


public class SwerveModule extends Submodule {

    private static enum MotorMode {
        POSITION, VELOCITY
    };

    private double zeroAngle;
    private MotorMode runMode = MotorMode.VELOCITY;

    private TalonFX rotor;
    private TalonFX motor;
    private HolonomicProfileFollower moduleProfile;

    private CANCoder angle;
    private int quadrant;

    private double motorPos = 0;
    private double motorVel = 0;
    private double rotorTargPos = 0;
    // whether or not to reverse the motor becuase of constant
    // angle adjustments
    private boolean angleAdjustmentMotorPolarity = false;

    private Path path;

    /**
     * Called at the start of autonomous or teleop.
     * 
     * @param timestamp
     */
    public void onStart(double timestamp) {}

    /**
     * Called once when the submodule is initialized.
     */
    public void onInit(int ids[],double initAngle,int quadrant) {
        motor = new TalonFX(ids[0]);
        rotor = new TalonFX(ids[1]);
        angle = new CANCoder(quadrant);
        moduleProfile = new HolonomicProfileFollower(motor, rotor);
        zeroAngle = initAngle;
        this.quadrant = quadrant;


        motor.configFactoryDefault();
        rotor.setInverted(SwerveConstants.DEFAULT_MOTOR_INVERSION);
        motor.configSelectedFeedbackSensor(SwerveConstants.FEEDBACKDEVICE);
        motor.selectProfileSlot(SwerveConstants.MOTOR_VELOCITY_SLOT,SwerveConstants.PID_PRIMARY_SLOT);
        motor.config_kP(SwerveConstants.MOTOR_POSITION_SLOT, SwerveConstants.MOTOR_POSI_KP);
        motor.config_kD(SwerveConstants.MOTOR_POSITION_SLOT, SwerveConstants.MOTOR_POSI_KD);
        //motor.config_kF(SwerveConstants.MOTOR_VELOCITY_SLOT, SwerveConstants.MOTOR_VELO_KF);
        motor.config_kP(SwerveConstants.MOTOR_VELOCITY_SLOT, SwerveConstants.MOTOR_VELO_KP);
        motor.config_kD(SwerveConstants.MOTOR_VELOCITY_SLOT, SwerveConstants.MOTOR_VELO_KD);
        motor.configMotionAcceleration(SwerveConstants.DEFAULT_TARG_ACCEL);
        motor.configMotionCruiseVelocity(SwerveConstants.DEFAULT_TARG_VELO);

        rotor.configFactoryDefault();
        rotor.setInverted(SwerveConstants.ROTOR_INVERSION);
        rotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rotor.selectProfileSlot(SwerveConstants.ROTOR_PID_SLOT,SwerveConstants.PID_PRIMARY_SLOT);
        rotor.config_kP(SwerveConstants.ROTOR_PID_SLOT, SwerveConstants.ROTOR_KP);
        rotor.config_kD(SwerveConstants.ROTOR_PID_SLOT, SwerveConstants.ROTOR_KD);
        rotor.configMotionAcceleration(SwerveConstants.ROTOR_TARG_ACCEL);
        rotor.configMotionCruiseVelocity(SwerveConstants.ROTOR_TARG_VELO);

        zero();
    }

    public void changeMotorMode(MotorMode mode){
        runMode = mode;
        switch(runMode){
            case POSITION:        
                motor.selectProfileSlot(SwerveConstants.MOTOR_POSITION_SLOT,SwerveConstants.PID_PRIMARY_SLOT);
            break;
            case VELOCITY:            
                motor.selectProfileSlot(SwerveConstants.MOTOR_VELOCITY_SLOT,SwerveConstants.PID_PRIMARY_SLOT);
            break;
        }
    }

/**
 * sets the rotor to PID to a position
 * @param pos the position to go to in units of full rotations
 */
    public void setRotorPos(double pos) {
        // convert to revolutions
        pos = pos / SwerveConstants.DEGREES_IN_REV;
        // get the difference in angle
        double dPos = pos - getRotorPosition();
        // get the positive adjusted angle
        dPos = dPos % 1;
        dPos += (dPos < -0.25 ? 1 : 0);
        dPos -= (dPos > 0.75 ? 1 : 0);
        
        if (dPos > 0.25) {
            dPos -= 0.5;
            angleAdjustmentMotorPolarity = true;
            rotorTargPos = dPos + getRotorPosition();
            return;
        }
        angleAdjustmentMotorPolarity = false;
        rotorTargPos = dPos + getRotorPosition();
    }

    public void setMotorVelocity(double speed) {
        motor.setInverted(angleAdjustmentMotorPolarity);
        motorVel = speed;
    }

    public void setMotorPosition(double position) {
        // will have to sort out reversing later
        motorPos = position;
    }

    /**
     * Reads cached inputs & calculate outputs.
     */
    public void update(double timestamp) {
        
    }
    
    /**
     * Sets the module to a 2-D target velocity.  Input array should have magnitude less than 1
     */
    public void setVectorVelocity(double[] v){
        // set the velocity to the magnitude of vector v scaled to the maximum desired speed
        setMotorVelocity(Math.sqrt(Math.pow(v[0],2)+Math.pow(v[1],2))*SwerveConstants.MAX_MOTOR_SPEED);
        // set rotor to the theta of cartesian vector v if the magnitude of the vector is not too small
        if (motorVel < 0.01){
            return;
        }
        setRotorPos(SwerveConstants.DEGREES_IN_REV/(2*Math.PI)*Math.atan2(v[1], v[0]));
    }


    /**
     * Runs components in the submodule that have continuously changing 
     * inputs.
     */
    public void run() {
        rotor.set(ControlMode.MotionMagic, rotorTargPos * SwerveConstants.ROTOR_REVOLUTION_RATIO);
        switch(runMode){
            case POSITION:        
                motor.set(ControlMode.MotionMagic, motorPos);
            break;
            case VELOCITY:            
                motor.set(ControlMode.Velocity, motorVel);
            break;
        }
    }

    /**
     * Resets the sensor(s) to zero.
     */
    public void zero() {
        rotorTargPos = 0;
        zeroMotor();
        zeroRotor();
    }

    public void stop() {

    }
    
    public void zeroRotor() {
        rotor.setSelectedSensorPosition((int)((angle.getAbsolutePosition()-zeroAngle)*(SwerveConstants.ROTOR_REVOLUTION_RATIO/SwerveConstants.DEGREES_IN_REV)), 0, 10);
    }
    
    public void zeroMotor() {
        motor.setSelectedSensorPosition(0);
    }
    

    public double getRotorPosition() {
        return rotor.getSelectedSensorPosition(0) / SwerveConstants.ROTOR_REVOLUTION_RATIO;
    }


    public void runPath(Path path){
        moduleProfile.start(path.getPathPoints());
    }

}