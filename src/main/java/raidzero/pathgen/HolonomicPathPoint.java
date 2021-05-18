package raidzero.pathgen;

public class HolonomicPathPoint extends PathPoint {

    /**
     * Cartesian coordinates of the robot in inches using the same convention as the
     * rest of pathgen (x is lengthwise of the field and y is widthwise). This is
     * important for holonomic pathing to keep track of the location of the robot on
     * the field. Can also be used for differential drivetrains in the future for
     * more advanced odometry techniques.
     */
    public double x, y;

    /**
     * Orientation of the robot in degrees. This is only different from angle for
     * holonomic robots.
     */
    public double orientation;

    public static void printPathPoints(HolonomicPathPoint[] pathPoints) {
        for (var pathPoint : pathPoints) {
            System.out.println(pathPoint);
        }
    }

    @Override
    public String toString() {
        return String.format("%.2fs > (%.2f, %.2f) | %.2f in/100ms | %.2f deg", (timeFromStart / 10.0), x, y, velocity, orientation);
    }
}
