package raidzero.robot.auto.sequences;

import java.util.Arrays;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;

import raidzero.robot.auto.actions.*;
import raidzero.robot.pathing.Path;
import raidzero.robot.submodules.*;

public class SlalomPathSequence extends AutoSequence {

    private static final Path PATH = Path.fromWaypoints(
        Arrays.asList(
            new Pose2d(
                Units.inchesToMeters(0), Units.inchesToMeters(0),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(60), Units.inchesToMeters(30),
                Rotation2d.fromDegrees(90)
            ),
            new Pose2d(
                Units.inchesToMeters(120), Units.inchesToMeters(60),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(180), Units.inchesToMeters(60),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(240), Units.inchesToMeters(30),
                Rotation2d.fromDegrees(-90)
            ),
            new Pose2d(
                Units.inchesToMeters(270), Units.inchesToMeters(-15),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(320), Units.inchesToMeters(30),
                Rotation2d.fromDegrees(90)
            ),
            new Pose2d(
                Units.inchesToMeters(270), Units.inchesToMeters(60),
                Rotation2d.fromDegrees(180)
            ),
            new Pose2d(
                Units.inchesToMeters(250), Units.inchesToMeters(30),
                Rotation2d.fromDegrees(235)
            ),
            new Pose2d(
                Units.inchesToMeters(180), Units.inchesToMeters(-5),
                Rotation2d.fromDegrees(180)
            ),
            new Pose2d(
                Units.inchesToMeters(90), Units.inchesToMeters(-5),
                Rotation2d.fromDegrees(180)
            ),
            new Pose2d(
                Units.inchesToMeters(60), Units.inchesToMeters(52),
                Rotation2d.fromDegrees(120)
            ),
            new Pose2d(
                Units.inchesToMeters(20), Units.inchesToMeters(60),
                Rotation2d.fromDegrees(180)
            )
        ),
        false, 4.0, 4.2
    );
    private static final Swerve swerve = Swerve.getInstance();

    public SlalomPathSequence() {
    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(Arrays.asList(
            new LambdaAction(() -> {
                swerve.zero();
                swerve.setPose(
                    new Pose2d(
                        Units.inchesToMeters(12.0),
                        Units.inchesToMeters(-10.0), 
                        new Rotation2d()
                    )
                );
            }),
            new DrivePath(PATH)
        )));
        System.out.println("Added actions.");
    }

    @Override
    public void onEnded() {
        System.out.println("SlalomPathSequence ended!");
    }

    @Override
    public String getName() {
        return "Slalom Pathing Sequence";
    }
}