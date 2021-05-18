
package raidzero.pathgen;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.util.FastMath;

public class HolonomicPathGenerator extends PathGenerator {

    public static HolonomicPathPoint[] generateHolonomicPath(Point[] waypoints,
            double cruiseVelocity, double targetAcceleration, double[] angleEndpoints,
            double targetAngularAcceleration) {
        generatePath(waypoints, cruiseVelocity, targetAcceleration);
        var queryData = getQueryData(waypoints);
        var splines = calculateSplines(waypoints, queryData);

        var path = new HolonomicPathPoint[queryData.queryCount];
        for (var i = 0; i < path.length; i++) {
            path[i] = new HolonomicPathPoint();
        }

        calculatePathPoints(path, cruiseVelocity, targetAcceleration, splines, queryData);

        var xQueries = query(splines.x.getPolynomials()[0]::value, queryData);
        var yQueries = query(splines.y.getPolynomials()[0]::value, queryData);

        // TODO(louis): Incorperate robot orientation with trapezoidal profile
        var interp = (new LinearInterpolator()).interpolate(
            new double[] {0, path[path.length - 1].timeFromStart},
            angleEndpoints
        );
        // Fill in the x y coordinates of the points along with the linearly
        // interpolated robot orientations
        for (var i = 0; i < path.length; i++) {
            path[i].x = xQueries[i];
            path[i].y = yQueries[i];
            path[i].orientation = interp.value(path[i].timeFromStart);
        }
        return path;
    }

    public static PathPoint[] generateModulePath(HolonomicPathPoint[] pathPoints, double moduleAngle, double moduleRadius) {
        PathPoint[] shiftedPathPoints = new PathPoint[pathPoints.length];
        for (int i = 0; i < pathPoints.length; ++i) {
            shiftedPathPoints[i] = new PathPoint();
        }

        double[] dx = new double[pathPoints.length];
        double[] dy = new double[pathPoints.length];

        // TODO(louis): Verify if this is fine for most cases
        dx[0] = 1.0;
        dy[0] = Math.tan(Math.toRadians(pathPoints[0].angle));
        shiftedPathPoints[0].velocity = 0;
        
        // double cosNow = 0.0, cosPrev = 0.0;
        // double sinNow = 0.0, sinPrev = 0.0;
        double currentAngle = 0.0;
        double dtheta = 0.0; // change in orientation
        for (int i = 1; i < pathPoints.length; i++) {
            // cosNow = Math.cos(angleOffset - Math.toRadians(pathPoints[i].orientation));
            // cosPrev = Math.cos(angleOffset - Math.toRadians(pathPoints[i - 1].orientation));
            // sinNow = Math.sin(angleOffset - Math.toRadians(pathPoints[i].orientation));
            // sinPrev = Math.sin(angleOffset - Math.toRadians(pathPoints[i - 1].orientation));
            // dx[i] = (pathPoints[i].x + radius * cosNow) - (pathPoints[i - 1].x + radius * cosPrev);
            // dy[i] = (pathPoints[i].y + radius * sinNow) - (pathPoints[i - 1].y + radius * sinPrev);
            currentAngle = Math.toRadians(pathPoints[i].orientation);
            dtheta = currentAngle - Math.toRadians(pathPoints[i - 1].orientation);
            double vX = pathPoints[i].x - pathPoints[i - 1].x;
            double vY = pathPoints[i].y - pathPoints[i - 1].y;
            /**
             * double newX = vX * cos + vY * sin;
             * double newY = -vX * sin + vY * cos;
             */
            double cos = Math.cos(currentAngle);
            double sin = Math.sin(currentAngle);
            dx[i] = vX * cos + vY * sin - dtheta * moduleRadius * Math.sin(moduleAngle);
            dy[i] = -vX * sin + vY * cos + dtheta * moduleRadius * Math.cos(moduleAngle);
            //dx[i] = (dtheta) * moduleRadius * Math.sin(moduleAngle);// - currentAngle);
            //dy[i] = (dtheta) * moduleRadius * Math.cos(moduleAngle);// - currentAngle);
            shiftedPathPoints[i].time = pathPoints[i].time;
            shiftedPathPoints[i].timeFromStart = pathPoints[i].timeFromStart;
            shiftedPathPoints[i].velocity = FastMath.hypot(dx[i], dy[i]) / shiftedPathPoints[i].time;
            // System.out.println("Shifted vel: " + shiftedPathPoints[i].velocity + " dx: " + dx[i] + " dy: " + dy[i]);
        }

        calculateAngles(dx, dy, shiftedPathPoints);
        cumulativeDistances(dx, dy, shiftedPathPoints);

        // shiftedPathPoints[0].angle = pathPoints[0].angle;

        // for (var pp : shiftedPathPoints) {
        //     System.out.println(
        //         (pp.time / 10.0) + "s " + pp.position + " in " + pp.velocity + " in/100ms " + pp.angle + " deg"
        //     );
        // }
        return shiftedPathPoints;
    }
}
