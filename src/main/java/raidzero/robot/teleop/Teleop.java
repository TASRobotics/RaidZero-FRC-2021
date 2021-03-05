package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import raidzero.robot.submodules.Superstructure;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.Constants.HoodConstants.HoodAngle;
import raidzero.robot.Constants.IntakeConstants;
import raidzero.robot.submodules.Conveyor;
import raidzero.robot.submodules.AdjustableHood;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.submodules.Spindexer;
import raidzero.robot.submodules.Turret;
import raidzero.robot.utils.JoystickUtils;

public class Teleop {

    private static Teleop instance = null;
    private static XboxController p1 = new XboxController(0);
    private static XboxController p2 = new XboxController(1);

    private static final Swerve swerve = Swerve.getInstance();
    private static final Intake intake = Intake.getInstance();
    private static final Conveyor conveyor = Conveyor.getInstance();
    private static final Spindexer spindexer = Spindexer.getInstance();
    private static final Superstructure superstructure = Superstructure.getInstance();
    private static final AdjustableHood hood = AdjustableHood.getInstance();
    private static final Shooter shooter = Shooter.getInstance();
    private static final Turret turret = Turret.getInstance();

    private static boolean shift1 = false;
    private static boolean shift2 = false;

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
    public void zero() {
    }

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
        shift1 = p1.getBumper(Hand.kRight);

    /**
     * Drive
    */
        swerve.fieldOrientedDrive(JoystickUtils.deadband(p1.getX(Hand.kLeft)),
            JoystickUtils.deadband(-p1.getY(Hand.kLeft)),
            JoystickUtils.deadband(p1.getX(Hand.kRight)),
            JoystickUtils.deadband(-p1.getY(Hand.kRight)));
        /**
         * DO NOT CONTINUOUSLY CALL THE ZERO FUNCTION its not that bad but the absolute encoders are
         * not good to PID off of so a quick setting of the relative encoder is better
         */
        if (p1.getBackButtonPressed()) {
            swerve.zero();
            return;
        }
        
        
    /**
     * Intake
    */
        // intakeOut is used to passively shuffle the spindexer
        double intakeOut = p1.getTriggerAxis(Hand.kRight);
      
        intake.intakeBalls(JoystickUtils.deadband(IntakeConstants.CONTROL_SCALING_FACTOR * intakeOut));
        intake.setMotorDirection(shift1);
    /**
     * Spindexer
     */
        // shifting reverses it, and intaking moves it slowly forward to shuffle
        spindexer.rotate(JoystickUtils.deadband( ((shift1 ? -1 : 1) * p1.getTriggerAxis(Hand.kLeft)) +
            (shift1 ? 0 : intakeOut/5)));

    /**
     * Conveyor
     */
        if (p1.getBButton()) {
            conveyor.moveBalls(1.0);
        } else if (p1.getXButton()) {
            conveyor.moveBalls(-1.0);
        } else {
            conveyor.moveBalls(0.0);
        }
        
    }

    private void p2Loop() {
        shift2 = p2.getBumper(Hand.kRight);

        if (p2.getBumper(Hand.kLeft)) {
            shooter.shoot(JoystickUtils.deadband(p2.getTriggerAxis(Hand.kRight)), false);

            if (p2.getAButtonPressed()) {
                // TODO: PID turret 90 degrees
                superstructure.setTurretPIDing(true);
            } else if (p2.getAButtonReleased()) {
                superstructure.setTurretPIDing(false);
            }
            return;
        }

        if (p2.getAButtonPressed()) {
            superstructure.setAiming(true);
        } else if (p2.getAButtonReleased()) {
            // In case the override button is released while PIDing
            if (superstructure.isTurretPIDing()) {
                superstructure.setTurretPIDing(false);
            }
            superstructure.setAiming(false);
        }
        // Turn turret using right joystick
        if (!superstructure.isUsingTurret()) {
            turret.rotateManual(JoystickUtils.deadband(p2.getX(Hand.kRight)));
        }

        /**
         * Shooter
         */
        if (p2.getBumperPressed(Hand.kRight)) {
            shooter.shoot(1.0, false);
        } else if (p2.getBumperReleased(Hand.kRight)) {
            shooter.shoot(0.0, false);
        }

        /**
         * Hood
         */
        if (p2.getStickButton(Hand.kRight)) {
            superstructure.setAimingAndHood(true);
        } else {
            superstructure.setAimingAndHood(false);
        }

        /**
         * Adjustable hood
         */
        int p2Pov = p2.getPOV();
        if (p2Pov == 0) {
            hood.moveToAngle(HoodAngle.RETRACTED);
        } else if (p2Pov == 90) {
            hood.moveToAngle(HoodAngle.HIGH);
        } else if (p2Pov == 180) {
            hood.moveToAngle(HoodAngle.MEDIUM);
        } else if (p2Pov == 270) {
            hood.moveToAngle(HoodAngle.LOW);
        } else {
            if (p2.getXButton()) {
                hood.adjust(-0.5);
            } else if (p2.getBButton()) {
                hood.adjust(0.5);
            } else {
                hood.stop();
            }
        }
    }
}
