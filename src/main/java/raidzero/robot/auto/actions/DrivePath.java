package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

import raidzero.robot.pathing.Path;
import raidzero.robot.submodules.Swerve;

/**
 * Action for following a path.
 */
public class DrivePath implements Action {

    private static final Swerve swerve = Swerve.getInstance();

    private Trajectory trajectory;
    private Rotation2d targetAngle;

    public DrivePath(Path path) {
        this(path, new Rotation2d(0.0));
    }

    public DrivePath(Path path, Rotation2d targetAngle) {
        this(path.getTrajectory(), targetAngle);
    }

    public DrivePath(Trajectory trajectory, Rotation2d targetAngle) {
        this.trajectory = trajectory;
        this.targetAngle = targetAngle;
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
        swerve.followPath(trajectory, targetAngle);
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
