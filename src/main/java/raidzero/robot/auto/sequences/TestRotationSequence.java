package raidzero.robot.auto.sequences;

import java.util.Arrays;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;

import raidzero.robot.auto.actions.*;
import raidzero.robot.pathing.Path;
import raidzero.robot.submodules.*;

public class TestRotationSequence extends AutoSequence {

    private static final Path PATH = Path.fromWaypoints(
        Arrays.asList(
            new Pose2d(
                Units.inchesToMeters(0), Units.inchesToMeters(0),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(100), Units.inchesToMeters(0),
                Rotation2d.fromDegrees(0)
            )
        ),
        false, 2.5, 2.5
    );
    private static final Swerve swerve = Swerve.getInstance();

    public TestRotationSequence() {
    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(Arrays.asList(
            new LambdaAction(() -> {
                swerve.zero();
                swerve.setPose(
                    new Pose2d()
                );
            }),
            new DrivePath(PATH, Rotation2d.fromDegrees(180.0))
        )));
        System.out.println("Added actions.");
    }

    @Override
    public void onEnded() {
        System.out.println("TestRotationSequence ended!");
    }

    @Override
    public String getName() {
        return "Test Rotation Sequence";
    }
}