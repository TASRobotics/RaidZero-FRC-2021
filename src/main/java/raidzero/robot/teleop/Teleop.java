package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import raidzero.robot.submodules.Superstructure;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.Constants.IntakeConstants;
import raidzero.robot.submodules.Conveyor;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Spindexer;
import raidzero.robot.utils.JoystickUtils;

public class Teleop {

    private static Teleop instance = null;
    private static XboxController p1 = new XboxController(0);
    private static XboxController p2 = new XboxController(1);

    private static final Swerve swerve = Swerve.getInstance();
    private static final Intake intake = Intake.getInstance();
    private static final Conveyor conveyor = Conveyor.getInstance();
    private static final Spindexer spindexer = Spindexer.getInstance();

    public static Teleop getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }

    public void onStart() {

    }

    /**
     * Stops the submodule.
     */
    public void stop() {

    }

    /**
     * Resets the sensor(s) to zero.
     */
    public void zero() {}
    /**
     * Continuously loops in teleop.
     */
    public void onLoop() {
        /**
         * shared controls
         */

        /**
         * p1 controls
         */
        p1Loop();
        /**
         * p2 controls
         */
        p2Loop();
    }

    private void p1Loop() {
        swerve.Drive(p1.getX(Hand.kLeft), p1.getX(Hand.kLeft), p1.getX(Hand.kRight));
        
        int intakeDirection = 1;
        if (p1.getBumper(Hand.kRight)){
            intakeDirection = -1;
        }
        intake.intakeBalls(JoystickUtils.deadband(
            IntakeConstants.CONTROL_SCALING_FACTOR * (intakeDirection * p1.getTriggerAxis(Hand.kRight))
        ));

        int spindexerDirection = 1;
        if (p1.getBumper(Hand.kLeft)){
            spindexerDirection = -1;
        }
        spindexer.rotate(JoystickUtils.deadband(
            spindexerDirection * p1.getTriggerAxis(Hand.kLeft)
        ));

        if (p1.getStartButton()) {
            spindexer.rampUp();
        } else if (p1.getBackButton()) {
            spindexer.rampDown();
        }

        if (p1.getBButton()) {
            conveyor.moveBalls(1.0);
        } else if (p1.getXButton()) {
            conveyor.moveBalls(-1.0);
        } else {
            conveyor.moveBalls(0.0);
        }

        // if(p1.getBumper(Hand.kLeft)) {
        //     swerve.test(p1);
        //     return;
        // }
        swerve.FieldOrientedDrive(
            JoystickUtils.deadband(p1.getX(Hand.kLeft)),
            JoystickUtils.deadband(-p1.getY(Hand.kLeft)), 
            JoystickUtils.deadband(p1.getX(Hand.kRight)), 
            JoystickUtils.deadband(-p1.getY(Hand.kRight))
        );
        if(p1.getAButton()) swerve.zero();
    }

    private void p2Loop() {
    }
}
