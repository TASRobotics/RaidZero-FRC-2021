package raidzero.pathgen;

public class TrapezoidalProfiler{
    /**
     * The Initial and final position to trapezoid between (in appropriate units)
     */
    private double[] endpoints;

    /**
     * The target acceleration in units /100ms^2
     */
    private double targetAcceleration;

    /**
     * The total time in units of 100ms
     */
    private double[] pointTimes;

    private double[] positionPoints;

    /**
     * Constructs the trapezoidal profile based on the given data cruise velocity is not needed since we construct the profile given a limited time.
     * Therefore, the maximum velocity will be based on the other given data.
     * @param endpoints the endpoints of the trapezoidal motion to calculate
     * @param targetAcceleration  the target acceleration to use
     */
    TrapezoidalProfiler(double[] endpoints, double targetAcceleration, double[] pointTimes){
        this.endpoints = endpoints;
        this.targetAcceleration = targetAcceleration;
        this.pointTimes = pointTimes;

        CreateProfile();

    }

    TrapezoidalProfiler(double[] endpoints, double targetAcceleration, HolonomicPathPoint[] pathPoints){
        this.endpoints = endpoints;
        this.targetAcceleration = targetAcceleration;
        this.pointTimes = new double[pathPoints.length];

        for(int i=0;i<pathPoints.length;i++){
            this.pointTimes[i] = pathPoints[i].time;
        }

        CreateProfile();
        FillAngles(pathPoints);

    }

    public void FillAngles(HolonomicPathPoint[] pathPoints){
        for (int i=0;i<pathPoints.length;i++){
            pathPoints[i].orientation = positionPoints[i];
        }
    }

    private void CreateProfile(){
        positionPoints = new double [pointTimes.length];
        double totalDistance = endpoints[1]-endpoints[0];
        double totalTime = pointTimes[pointTimes.length-1];
        double  deltat = totalTime/2-Math.sqrt(Math.pow(totalTime,2)/4-totalDistance/targetAcceleration);
        for (int i=0;i<positionPoints.length;i++){
            if(pointTimes[i]<deltat){ //Ramping up distance calculation
                positionPoints[i] = 1/2*targetAcceleration*Math.pow(pointTimes[i],2);
            } else if (pointTimes[i] > totalTime-deltat){ //Ramping down distance calculation
                positionPoints[i] = totalDistance - 1/2*targetAcceleration*Math.pow(totalTime-pointTimes[i],2);
            } else{  //Cruise Distance Calculations
                positionPoints[i] = 1/2*targetAcceleration*Math.pow(deltat,2)+targetAcceleration*deltat*(pointTimes[i]-deltat);
            }
        }
    }
}