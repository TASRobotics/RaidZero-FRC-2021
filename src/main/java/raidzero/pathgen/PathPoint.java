package raidzero.pathgen;

/**
 * Point along a generated robot path.
 *
 * Since this class has no explicit constructor, each field needs to be set
 * individually.
 */
public class PathPoint {

    /**
     * Position of the robot in inches.
     *
     * Note: position means the distance the robot has traveled by the time it
     * reaches this point, not the 2-dimensional x and y coordinates of the robot.
     */
    public double position;

    /**
     * Velocity of the robot in inches/100ms.
     */
    public double velocity;

    /**
     * Time it takes for the robot to go from the previous point to the current
     * point in units of 100ms.
     *
     * This is 0 for the first point in the path.
     */
    public double time;

    /**
     * Time from the start in units of 100ms.
     */
    public double timeFromStart;

    /**
     * Angle heading of the robot in degrees.
     */
    public double angle;

    public static void printPathPoints(PathPoint[] pathPoints) {
        for (var pathPoint : pathPoints) {
            System.out.println(pathPoint);
        }
    }

    @Override
    public String toString() {
        return String.format("%.2fs > %.2f in | %.2f in/100ms | %.2f deg", (timeFromStart / 10.0), position, velocity, angle);
    }
}
