package raidzero.robot.auto.actions;

import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.submodules.Swerve;

/**
 * Action for aligning swerve modules to an angle.
 */
public class AlignSwerveModules implements Action {

    private static final Swerve swerve = Swerve.getInstance();

    private double angle;
    private double[] targetAngles;

    public AlignSwerveModules(double angle) {
        this.angle = angle;
    }

    @Override
    public boolean isFinished() {
        for (int i = 0; i < 4; ++i) {
            System.out.println("M" + i + ": rn=" + swerve.getModuleRotorPosition(i) + ", target=" + targetAngles[i]);
            if (Math.abs(swerve.getModuleRotorPosition(i) - targetAngles[i]) / SwerveConstants.ROTOR_REVOLUTION_RATIO > 1.0) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        targetAngles = swerve.setRotorPositions(angle);
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
