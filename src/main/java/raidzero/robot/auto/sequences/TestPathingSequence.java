package raidzero.robot.auto.sequences;

import java.util.Arrays;

import raidzero.pathgen.Point;
import raidzero.robot.auto.actions.*;
import raidzero.robot.pathing.HolonomicPath;
import raidzero.robot.submodules.Swerve;

public class TestPathingSequence extends AutoSequence {

    private static final Point[] POINTS = new Point[]{
        new Point(0, 0, 90),
        // new Point(0, 25, 90),
        new Point(0, 70, 90),
    };

    private static final Point[] POINTS2 = new Point[]{
        new Point(0, 50, 0),
        new Point(25, 50, 0),
        new Point(50, 50, 0),
    };

    private static final Point[] POINTS3 = new Point[]{
        new Point(50, 50, -90),
        new Point(0, 0, 180),
    };

    private static final HolonomicPath PATH = new HolonomicPath(
        POINTS, 10.0, 10.0, new double[]{0, 90}, 100);

    // private static final HolonomicPath PATH2 = new HolonomicPath(
    //     POINTS2, 10.0, 10.0, new double[]{0, 0}, 100);

    // private static final HolonomicPath PATH3 = new HolonomicPath(
    //     POINTS3, 10.0, 10.0, new double[]{0, 0}, 100);
    
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