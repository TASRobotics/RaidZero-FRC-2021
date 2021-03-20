package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import raidzero.robot.submodules.Superstructure;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.Constants.HoodConstants.HoodAngle;
import raidzero.robot.Constants.IntakeConstants;
import raidzero.robot.Constants.SpindexerConstants;
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
        swerve.zero();
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
        p1Loop(p1);
        /**
         * p2 controls
         */
        p2Loop(p2);
    }

    private void p1Loop(XboxController p) {
        /**
         * Drive
        */
        swerve.fieldOrientedDrive(
            JoystickUtils.deadband(0.5*p.getX(Hand.kLeft)),
            JoystickUtils.deadband(-0.5*p.getY(Hand.kLeft)),
            //JoystickUtils.deadband(p.getX(Hand.kRight)));
            JoystickUtils.deadband(0.25*p.getRawAxis(2)));
        /**
         * DO NOT CONTINUOUSLY CALL THE ZERO FUNCTION its not that bad but the absolute encoders are
         * not good to PID off of so a quick setting of the relative encoder is better
         */
        if (p.getBackButtonPressed()) {
            swerve.zero();
            return;
        }
        
        
        /**
         * Intake
        */
        shift1 = p.getBumper(Hand.kRight);
        // intakeOut is used to passively shuffle the spindexer
        double intakeOut = p.getTriggerAxis(Hand.kRight);
      
        intake.intakeBalls(JoystickUtils.deadband(IntakeConstants.CONTROL_SCALING_FACTOR * intakeOut));
        intake.setMotorDirection(shift1);
        /**
        * Spindexer
         */
        // shifting reverses it, and intaking moves it slowly forward to shuffle
        spindexer.rotate(JoystickUtils.deadband( ((shift1 ? -1 : 1) * p.getTriggerAxis(Hand.kLeft)) +
            (shift1 ? 0 : intakeOut/7.5)));
        // OVERWRITTEN WHEN THE CONVEYOR MOVES
        
    }

    private void p2Loop(XboxController p) {
        shift2 = p.getBumper(Hand.kLeft);

        /**
         * if (p.getBumper(Hand.kLeft)) {
         *     shooter.shoot(JoystickUtils.deadband(p.getTriggerAxis(Hand.kRight)), false);
         * 
         *     if (p.getAButtonPressed()) {
         *         // TODO: PID turret 90 degrees
         *         superstructure.setTurretPIDing(true);
         *     } else if (p.getAButtonReleased()) {
         *         superstructure.setTurretPIDing(false);
         *     }
         *     return;
         * }
         */

        /**
         * autoAim
         */
        /** if (p.getAButtonPressed()) {
            superstructure.setAiming(true);
        } else if (p.getAButtonReleased()) {
            // In case the override button is released while PIDing
            if (superstructure.isTurretPIDing()) {
                superstructure.setTurretPIDing(false);
            }
            superstructure.setAiming(false);
        }
        */

        /**
         * Turret
         */
        // Turn turret using right joystick
        if (!superstructure.isUsingTurret()) {
            turret.rotateManual(JoystickUtils.deadband(p.getX(Hand.kRight)));
        }

        /**
         * Shooter
         */
        if (p.getBumperPressed(Hand.kRight)) {
            shooter.shoot(1.0, false);
        } else if (p.getBumperReleased(Hand.kRight)) {
            shooter.shoot(0.0, false);
        }

        /**
         * Spindexer
         */
        spindexer.rotate(p.getTriggerAxis(Hand.kRight) 
            * SpindexerConstants.SHOOT_SPEED);
        if(p.getStartButton()) {
            spindexer.rampUp();
        } else {
            spindexer.rampDown();
        }

        /**
         * Conveyor
         */
        if (p.getYButton()) {
            conveyor.moveBalls(1.0);
            spindexer.shoot();
        } else {
            conveyor.moveBalls(-JoystickUtils.deadband(p.getY(Hand.kLeft)));
        }
        

        /**
         * Hood
         */
        if (p.getStickButton(Hand.kRight)) {
            superstructure.setAimingAndHood(true);
        } else {
            superstructure.setAimingAndHood(false);
        }

        /**
         * Adjustable hood
         */
        hood.adjust(p.getTriggerAxis(Hand.kLeft) * (shift2 ? 1 : -1));

        int pPov = p.getPOV();
        if (pPov == 0) {
            hood.moveToAngle(HoodAngle.RETRACTED);
        } else if (pPov == 90) {
            hood.moveToAngle(HoodAngle.HIGH);
        } else if (pPov == 180) {
            hood.moveToAngle(HoodAngle.MEDIUM);
        } else if (pPov == 270) {
            hood.moveToAngle(HoodAngle.LOW);
        }
    }
}
