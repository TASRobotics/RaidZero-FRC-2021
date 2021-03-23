package raidzero.robot.auto.sequences;

import org.junit.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import raidzero.robot.pathing.Path;
import static org.junit.Assert.*;
import java.util.Arrays;

public class TestSequence {

    private static final Path PATH = Path.fromWaypoints(
        Arrays.asList(
            new Pose2d(
                Units.inchesToMeters(0), Units.inchesToMeters(0),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(120), Units.inchesToMeters(0),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(150), Units.inchesToMeters(-30),
                Rotation2d.fromDegrees(-90)
            ),
            new Pose2d(
                Units.inchesToMeters(120), Units.inchesToMeters(-60),
                Rotation2d.fromDegrees(-180)
            ),
            new Pose2d(
                Units.inchesToMeters(70), Units.inchesToMeters(-30),
                Rotation2d.fromDegrees(-270)
            ),
            new Pose2d(
                Units.inchesToMeters(180), Units.inchesToMeters(0),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(250), Units.inchesToMeters(30),
                Rotation2d.fromDegrees(90)
            ),
            new Pose2d(
                Units.inchesToMeters(210), Units.inchesToMeters(80),
                Rotation2d.fromDegrees(180)
            ),
            new Pose2d(
                Units.inchesToMeters(170), Units.inchesToMeters(0),
                Rotation2d.fromDegrees(270)
            ),
            new Pose2d(
                Units.inchesToMeters(240), Units.inchesToMeters(-50),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                Units.inchesToMeters(320), Units.inchesToMeters(-30),
                Rotation2d.fromDegrees(90)
            ),
            new Pose2d(
                Units.inchesToMeters(300), Units.inchesToMeters(30),
                Rotation2d.fromDegrees(180)
            ),
            new Pose2d(
                Units.inchesToMeters(0), Units.inchesToMeters(10),
                Rotation2d.fromDegrees(180)
            )
        ),
        false, 3.0, 3.0
    );

    @Test
    public void isRunningFine() {
        var traj = PATH.getTrajectory();
        double time = traj.getTotalTimeSeconds();
        System.out.println("Total time: " + time);
        for (double t = 0.0; t < 14.0; t += 0.1) {
            var p = traj.sample(t);
            // System.out.println(p.toString());
            double angle = p.poseMeters.getRotation().getDegrees();
            while(angle > 180)angle-=360;
            while(angle < -180)angle+=360;
            System.out.println(angle);
        }
    }
}