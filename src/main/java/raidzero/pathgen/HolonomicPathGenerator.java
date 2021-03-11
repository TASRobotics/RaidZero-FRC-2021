
package raidzero.pathgen;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;

public class HolonomicPathGenerator extends PathGenerator {

    public static HolonomicPathPoint[] generateHolonomicPath(Point[] waypoints, double cruiseVelocity,
            double targetAcceleration, double[] angleEndpoints, double targetAngularAcceleration) {
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

        double time = 0.0;
        for (var i = 0; i < path.length; i++) {
            path[i].x = xQueries[i];
            path[i].y = yQueries[i];
            time += path[i].time;
        }

        // TODO(louis): Incorperate robot orientation with trapezoidal profile
        LinearInterpolator interpolator = new LinearInterpolator();
        var poly = interpolator.interpolate(new double[] {0, time}, angleEndpoints);
        for (var pathPoint : path) {
            System.out.println("Target orientation: " + poly.value(pathPoint.time));
            pathPoint.orientation = poly.value(pathPoint.time);
        }
        return path;
    }
}
