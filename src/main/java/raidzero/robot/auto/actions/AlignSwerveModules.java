package raidzero.robot.auto.actions;

import raidzero.robot.pathing.HolonomicPath;
import raidzero.robot.submodules.Swerve;

/**
 * Action for aligning swerve modules to an angle.
 */
public class AlignSwerveModules implements Action {

    private static final Swerve swerve = Swerve.getInstance();

    private double angle;

    public AlignSwerveModules(double angle) {
        this.angle = angle;
    }

    @Override
    public boolean isFinished() {
        for (int i = 0; i < 4; ++i) {
            System.out.println("M" + i + ": rn=" + swerve.getModuleRotorPosition(i) + ", target=" + angle);
            if (Math.abs(swerve.getModuleRotorPosition(i) - angle) > 1.0) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        swerve.setRotorPositions(angle);
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        swerve.stop();
    }
}
