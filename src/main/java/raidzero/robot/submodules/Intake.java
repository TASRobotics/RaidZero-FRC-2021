package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import raidzero.robot.wrappers.LazyTalonSRX;
import raidzero.robot.wrappers.InactiveDoubleSolenoid;
import edu.wpi.first.networkTables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import raidzero.robot.Constants.IntakeConstants;
import raidzero.robot.dashboard.Tab;

public class Intake extends Submodule {
    private static enum Position {
        DOWN, UP
    }

    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {}

    private LazyTalonSRX intakeMotor;
    private InactiveDoubleSolenoid solenoid;

    private double outputOpenLoop = 0.0;

    //private NetworkTableEntry intakePositionEntry = Shuffleboard.getTab(Tab.MAIN)
        //.add("Intake Position", position.toString())
        //.withPosition(4, 2)
        //.withSize(1,1)
        //.getEntry();

    @Override
    public void onInit(){
        intakeMotor = new LazyTalonSRX(IntakeConstants.MOTOR_ID);
        intakeMotor.configFactoryDefault();
        intakeMotor.setNeutralMode(IntakeConstants.NEUTRAL_MODE);
        intakeMotor.setInverted(IntakeConstants.INVERSION);

        solenoid = new InactiveDoubleSolenoid(IntakeConstants.INTAKE_FORWARD_ID, IntakeConstants.INTAKE_REVERSE_ID);
        setPosition(position);
    }

    @Override
    public void onStart(double timestamp){
        outputOpenLoop = 0.0;
    }

    @Override
    public void run(){
        intakeMotor.set(ControlMode.PercentOutput, outputOpenLoop);
    }

    @Override
    public void stop(){
        outputOpenLoop = 0.0;
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Spins the intake using open-loop control
     * @param percentOutput the percent output is [-1, 1]
     */

     public void intakeBalls(double percentOutput){
         outputOpenLoop = percentOutput;
     }

     public void setPosition(Position pos){
         position = pos;
         //intakePositionEntry.setString(position.toString());
         if (position == Position.DOWN){
             solenoid.set(Value.kForward);
         } else {
             solenoid.set(Value.kReverse);
         }
     }

     /**
      * Moves intake out or in depending on what state it's in
      */
      
      public void flipIntake(){
          invertPos();
          setPosition(position);
      }

      private void invertPos(){
          if (position == Position.DOWN){
              position = Position.UP;
              return;
          }
          position = Position.DOWN;
      }
}