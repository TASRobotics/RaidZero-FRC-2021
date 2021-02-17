
package raidzero.pathgen;

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

        for (var i = 0; i < path.length; i++) {
            path[i].x = xQueries[i];
            path[i].y = yQueries[i];
        }

        TrapezoidalProfiler orientationProfile = new TrapezoidalProfiler(angleEndpoints, targetAngularAcceleration,
                path);
        orientationProfile.fillOrientations(path);

        return path;
    }
}
