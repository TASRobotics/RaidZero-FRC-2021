package raidzero.robot.auto.sequences;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;

import raidzero.robot.auto.actions.*;
import raidzero.robot.pathing.Path;
import raidzero.robot.submodules.*;

public class SearchPathASequence extends AutoSequence {

    private static final Path PATH_RED = Path.fromWaypoints(
        Arrays.asList(
            new Pose2d(
                Units.inchesToMeters(0), Units.inchesToMeters(0),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(60), Units.inchesToMeters(45),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(120), Units.inchesToMeters(-15),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(150), Units.inchesToMeters(90),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(300), Units.inchesToMeters(90),
                Rotation2d.fromDegrees(0)
            )
        ),
        false, 3.5, 3.5
    );
    private static final Path PATH_BLUE = Path.fromWaypoints(
        Arrays.asList(
            new Pose2d(
                Units.inchesToMeters(0), Units.inchesToMeters(0),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(150), Units.inchesToMeters(-35),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(180), Units.inchesToMeters(70),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(240), Units.inchesToMeters(30),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(300), Units.inchesToMeters(30),
                Rotation2d.fromDegrees(0)
            )
        ),
        false, 3.5, 3.5
    );
    private static final Swerve swerve = Swerve.getInstance();

    public SearchPathASequence() {
    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(Arrays.asList(
            new LambdaAction(() -> {
                swerve.zero();
                swerve.setPose(
                    new Pose2d(
                        Units.inchesToMeters(-12.0),
                        Units.inchesToMeters(0.0), 
                        new Rotation2d()
                    )
                );
            }),
            new DrivePath(PATH_BLUE)
        )));
        System.out.println("Added actions.");
    }

    @Override
    public void onEnded() {
        System.out.println("SearchPathASequence ended!");
    }

    @Override
    public String getName() {
        return "Search Path A Sequence";
    }
}