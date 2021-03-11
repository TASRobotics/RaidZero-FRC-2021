package raidzero.robot.auto.actions;

import raidzero.robot.submodules.Swerve;
import raidzero.robot.submodules.SwerveModule.TargetPolarityTuple;

/**
 * Action for aligning swerve modules to an angle.
 */
public class AlignSwerveModules implements Action {

    private static final Swerve swerve = Swerve.getInstance();

    private double angle;
    private TargetPolarityTuple[] targetAndPolarities;
    private double[] lastError = new double[4];

    public AlignSwerveModules(double angle) {
        this.angle = angle;
    }

    @Override
    public boolean isFinished() {
        for (int i = 0; i < 4; ++i) {
            double error = swerve.getModuleRotorPosition(i) - targetAndPolarities[i].target;
            System.out.println("M" + i + ": rn=" + swerve.getModuleRotorPosition(i) + ", target=" + targetAndPolarities[i].target + ", err=" + error + ", derror=" + (error - lastError[i]));
            if (Math.abs(error) > 0.01 && Math.abs(error - lastError[i]) > 0.01) {
                return false;
            }
            lastError[i] = error;
        }
        return true;
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        swerve.zeroRotors();
        targetAndPolarities = swerve.setRotorPositions(angle, false);
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
