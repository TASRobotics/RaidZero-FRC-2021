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
import raidzero.robot.submodules.Limelight;
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
    private static final Limelight limelight = Limelight.getInstance();

    private static boolean shift1 = false;
    private static boolean shift2 = false;
    private static double intakeOut = 0;
    private boolean autoDisabled = false;

    public static Teleop getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }

    public void onStart() {
        swerve.zero();
        if(!autoDisabled) hood.goToZero();
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
         * Disable auto
         */

        if(p.getRawButton(3))autoDisabled = true;
        if(p.getRawButton(4))autoDisabled = false;

        /**
         * Drive
        */
        boolean turning = p.getRawButton(12);
        swerve
        .fieldOrientedDrive(
            JoystickUtils.deadband(p.getX(Hand.kLeft) * (p.getRawButton(1) ? 1 : 0.5)),
            JoystickUtils.deadband(p.getY(Hand.kLeft) * (p.getRawButton(1) ? -1 : -0.5)),
            //JoystickUtils.deadband(p.getX(Hand.kRight)));
            (turning) ? JoystickUtils.deadband(p.getRawAxis(2)) * (p.getRawButton(1) ? 0.5 : 0.25) : 0);
        /**
         * DO NOT CONTINUOUSLY CALL THE ZERO FUNCTION its not that bad but the absolute encoders are
         * not good to PID off of so a quick setting of the relative encoder is better
         */
        if (p.getRawButton(2)) {
            swerve.zero();
            return;
        }
        
        
        /**
         * Intake
        */
        shift1 = p.getRawButton(8);
        // intakeOut is used to passively shuffle the spindexer
        intakeOut = ((p.getRawButton(7) || shift1) ? 1 : 0) * ((-p.getRawAxis(3))+1) / 2;
      
        intake.intakeBalls((IntakeConstants.CONTROL_SCALING_FACTOR * intakeOut));
        intake.setMotorDirection(shift1);
        
    }

    private void p2Loop(XboxController p) {
        shift2 = p.getBumper(Hand.kLeft);     

        /**
         * Turret
         */
        // Turn turret using right joystick
        if (JoystickUtils.deadband(p.getX(Hand.kRight)) != 0 || autoDisabled) {
            superstructure.setAiming(false);
            turret.rotateManual(JoystickUtils.deadband(p.getX(Hand.kRight)));
        } else {
            superstructure.setAiming(true);
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
        spindexer.rotate(JoystickUtils.deadband( ((shift2 ? -1 : 1) * p.getTriggerAxis(Hand.kRight)) +
        ((intakeOut > 0) ? 0.13 : 0)));
        if(p.getStartButton()) {
            autoDisabled = true;
            spindexer.rampUp();
        } else {
            spindexer.rampDown();
            autoDisabled = true;
        }
        if(p.getBButton()) {
            spindexer.shoot();
        } else {
        }

        /**
         * Conveyor
         */
        if (p.getYButton()) {
            conveyor.moveBalls(1.0);
        } else {
            conveyor.moveBalls(-JoystickUtils.deadband(p.getY(Hand.kLeft)));
        }


        /**
         * Adjustable hood
         */
        if(p.getBackButtonPressed()) hood.goToZero();
        if(!autoDisabled)hood.autoPosition(limelight.getTa());
        else hood.adjust(p.getTriggerAxis(Hand.kLeft) * (shift2 ? 1 : -1));

        int pPov = p.getPOV();
        if (pPov == 0) {
            hood.moveToTick(HoodAngle.RETRACTED.ticks);
        } else if (pPov == 90) {
            hood.moveToTick(HoodAngle.HIGH.ticks);
        } else if (pPov == 180) {
            hood.moveToTick(HoodAngle.MEDIUM.ticks);
        } else if (pPov == 270) {
            hood.moveToTick(HoodAngle.LOW.ticks);
        }
    }
}
