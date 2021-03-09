package raidzero.robot.auto.sequences;

import java.util.Arrays;

import raidzero.pathgen.Point;
import raidzero.robot.auto.actions.*;
import raidzero.robot.pathing.HolonomicPath;
import raidzero.robot.submodules.Swerve;

public class TestPathingSequence extends AutoSequence {

    private static final Point[] POINTS = new Point[]{
        new Point(0, 0, 90),
        new Point(0, 35, 90),
        new Point(70, 70, 0),
    };

    private static final HolonomicPath PATH = new HolonomicPath(
        POINTS, 6.0, 6.0, new double[]{0, 0}, 100);
    
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