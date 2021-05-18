package raidzero.robot.pathing;

import org.junit.*;
import raidzero.pathgen.HolonomicPathGenerator;
import raidzero.pathgen.HolonomicPathPoint;
import raidzero.pathgen.PathPoint;
import raidzero.pathgen.Point;
import static org.junit.Assert.*;

public class HolonomicPathTester {

    public static final Point[] POINTS = new Point[]{
        new Point(0, 0, 90),
        new Point(0, 70, 90)
    };

    @Test
    public void isCorrectlyGenerating() {
        HolonomicPath path = new HolonomicPath(
            POINTS, 10.0, 10.0, new double[]{0, 45}, 100);
        HolonomicPathPoint.printPathPoints(path.getPathPoints());
        var modulePath = HolonomicPathGenerator.generateModulePath(path.getPathPoints(), Math.PI / 4, 23.0);
        System.out.println("=============================");        
        PathPoint.printPathPoints(modulePath);
    }
}