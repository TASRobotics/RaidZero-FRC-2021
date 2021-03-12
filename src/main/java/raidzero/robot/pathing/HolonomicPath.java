package raidzero.robot.pathing;

// import edu.wpi.first.wpilibj.Timer;

import raidzero.pathgen.HolonomicPathGenerator;
import raidzero.pathgen.HolonomicPathPoint;
import raidzero.pathgen.Point;

public class HolonomicPath {

    private HolonomicPathPoint[] pathPoints;
    private Point[] points;
    private double cruiseVel;
    private double targetAccel;

    public HolonomicPath(Point[] points, double cruiseVel, double targetAccel, double[] endPointAngles,
            double targetAngularAccel) {
        this.points = points;
        this.cruiseVel = cruiseVel;
        this.targetAccel = targetAccel;

        // double startTime = Timer.getFPGATimestamp();
        pathPoints = HolonomicPathGenerator.generateHolonomicPath(points, cruiseVel, targetAccel, endPointAngles,
                targetAngularAccel);
        // System.out.println("Holonomic path:");
        // for (var pp : pathPoints) {
        //     System.out.println(
        //         (pp.time / 10.0) + "s - (" + pp.x + ", " + pp.y + ") | " + pp.velocity + " in/100ms " + pp.orientation + " deg"
        //     );
        // }
        // System.out.println("PathGenerator: " + (Timer.getFPGATimestamp() - startTime) + "s to generate a path!");
    }

    /**
     * Returns the waypoints of the path.
     * 
     * @return array of waypoints
     */
    public Point[] getPoints() {
        return points;
    }

    /**
     * Returns the first waypoint of the path.
     * 
     * @return first point
     */
    public Point getFirstPoint() {
        return points[0];
    }

    /**
     * Returns the last waypoint of the path.
     * 
     * @return last point
     */
    public Point getLastPoint() {
        return points[points.length - 1];
    }

    /**
     * Returns the path points for the motion profile.
     * 
     * @return array of path points
     */
    public HolonomicPathPoint[] getPathPoints() {
        return pathPoints;
    }

    /**
     * Returns the target cruise velocity of the robot.
     * 
     * @return cruise velocity in in/100ms
     */
    public double getCruiseVelocity() {
        return cruiseVel;
    }

    /**
     * Returns the target acceleration of the robot.
     * 
     * @return target acceleration in in/100ms/s
     */
    public double getTargetAcceleration() {
        return targetAccel;
    }
}
