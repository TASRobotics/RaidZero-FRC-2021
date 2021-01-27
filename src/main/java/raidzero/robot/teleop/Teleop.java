package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import raidzero.robot.submodules.Superstructure;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.utils.JoystickUtils;

public class Teleop {

    private static Teleop instance = null;
    private static XboxController p1 = new XboxController(0);
    private static XboxController p2 = new XboxController(1);

    private static Swerve swerve = Swerve.getInstance();

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
        if(p1.getBumper(Hand.kLeft)) {
            swerve.test(p1);
            return;
        }
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
