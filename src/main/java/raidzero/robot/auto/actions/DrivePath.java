package raidzero.robot.auto.actions;

import raidzero.robot.pathing.HolonomicPath;
import raidzero.robot.submodules.Swerve;

/**
 * Action for following a path.
 */
public class DrivePath implements Action {

    private static final Swerve swerve = Swerve.getInstance();

    private HolonomicPath path;

    public DrivePath(HolonomicPath path) {
        this.path = path;
    }

    @Override
    public boolean isFinished() {
        if (swerve.isFinishedWithPath()) {
            System.out.println("[Auto] Path finished!");
            return true;
        }
        return false;
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        swerve.pushHolonomicPath(path);
    }

    @Override
    public void update() {
        if (swerve.isDoneWaitingForFill()) {
            // System.out.println("Enabling profile...");
            swerve.enableProfile();
        }
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        swerve.stop();
    }
}
