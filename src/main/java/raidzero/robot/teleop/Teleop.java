package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;

import raidzero.robot.submodules.Superstructure;

public class Teleop {

    private static Teleop instance = null;

    public static Teleop getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }

    private Teleop() {
    }

    private static Superstructure superstructure = Superstructure.getInstance();

    private XboxController p1 = new XboxController(0);
    private XboxController p2 = new XboxController(1);

    /**
     * Runs at the start of teleop.
     */
    public void onStart() {
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
    }

    private void p2Loop() {
    }
}
