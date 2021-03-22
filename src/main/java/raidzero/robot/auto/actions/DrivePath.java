package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.trajectory.Trajectory;

import raidzero.robot.pathing.Path;
import raidzero.robot.submodules.Swerve;

/**
 * Action for following a path.
 */
public class DrivePath implements Action {

    private static final Swerve swerve = Swerve.getInstance();

    private Trajectory trajectory;

    public DrivePath(Path path) {
        this(path.getTrajectory());
    }

    public DrivePath(Trajectory trajectory) {
        this.trajectory = trajectory;
    }

    @Override
    public boolean isFinished() {
        if (swerve.isFinishedPathing()) {
            System.out.println("[Auto] Path finished!");
            return true;
        }
        return false;
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        swerve.followPath(trajectory);
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
