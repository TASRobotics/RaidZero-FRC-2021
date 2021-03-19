package raidzero.robot.auto.sequences;

import java.util.Arrays;

import raidzero.pathgen.Point;
import raidzero.robot.auto.actions.*;
import raidzero.robot.pathing.HolonomicPath;
import raidzero.robot.submodules.Swerve;

public class TestPathingSequence extends AutoSequence {

    private static final Point[] POINTS = new Point[]{
        new Point(15, 15, 90),
        // new Point(15, 35, 90),
        new Point(-30, 65, 170),
        new Point(-70, 120, 90),
        new Point(-70, 180, 90),
        new Point(-30, 245, 10),
        new Point(15, 275, 90),
    };

    // private static final Point[] POINTS2 = new Point[]{
    //     new Point(0, 100, 0),
    //     new Point(100, 100, 0)
    // };

    // private static final Point[] POINTS3 = new Point[]{
    //     new Point(100, 100, 90),
    //     new Point(0, 0, 0),
    // };

    private static final HolonomicPath PATH = new HolonomicPath(
        POINTS, 6.0, 6.0, new double[]{0, 0}, 100);

    // private static final HolonomicPath PATH2 = new HolonomicPath(
    //     POINTS2, 12.0, 15.0, new double[]{0, 0}, 100);

    // private static final HolonomicPath PATH3 = new HolonomicPath(
    //     POINTS3, 12.0, 15.0, new double[]{0, 0}, 100);
    
    private static final Swerve swerve = Swerve.getInstance();

    public TestPathingSequence() {
    }
    

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new LambdaAction(() -> swerve.zero()),
                new AlignSwerveModules(PATH.getFirstPoint().angle.getAsDouble()),
                new DrivePath(PATH)
                // new AlignSwerveModules(PATH2.getFirstPoint().angle.getAsDouble()),
                // new DrivePath(PATH2),
                // new AlignSwerveModules(PATH3.getFirstPoint().angle.getAsDouble()),
                // new DrivePath(PATH3)
            )
        ));
        System.out.println("Added actions.");
    }

    @Override
    public void onEnded() {
        System.out.println("TestingPathSequence ended!");
    }

    @Override
    public String getName() {
        return "Testing Pathing";
    }
}