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

    @Override
    public String toString() {
        return (timeFromStart / 10.0) + "s - (" + x + ", " + y + ") | " + velocity + " in/100ms " + orientation + " deg";
    }
}
