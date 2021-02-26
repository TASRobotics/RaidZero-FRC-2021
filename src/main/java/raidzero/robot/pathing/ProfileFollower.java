package raidzero.robot.pathing;

import java.util.function.DoubleFunction;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import raidzero.pathgen.PathGenerator;
import raidzero.pathgen.Point;
import raidzero.pathgen.PathPoint;
import raidzero.robot.Constants.PathConstants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.utils.EncoderUtils;
import edu.wpi.first.wpilibj.Notifier;

public class ProfileFollower {

    /**
     * The states of the motion profile.
     */
    private enum State {
        FillPoints, WaitPoints, Run;
    };

    private BaseTalon leaderTalon;

    private boolean reversed;

    private boolean initRun;
    private State state;
    private MotionProfileStatus status;
    private SetValueMotionProfile setValue;

    private DoubleFunction<Double> positionUnitConverter;

    /**
     * Runs periodically to push the trajectory points into the controller.
     */
    Notifier notifier = new Notifier(() -> {
        leaderTalon.processMotionProfileBuffer();
    });

    /**
     * Creates the profile follower.
     */
    public ProfileFollower(BaseTalon leader, DoubleFunction<Double> positionUnitConverter) {
        leaderTalon = leader;
        this.positionUnitConverter = positionUnitConverter;

        setValue = SetValueMotionProfile.Disable;
        status = new MotionProfileStatus();
        notifier.startPeriodic(0.001 * PathConstants.TRANSMIT_PERIOD_MS);
        state = State.FillPoints;

        reversed = false;
    }

    /**
     * Starts the motion profile by generating & filling the points.
     *
     * @param points
     *                      the points to put in the path generator
     * @param cruiseVel
     *                      the cruise velocity desired in in/100ms
     * @param tarAccel
     *                      the target acceleration desired in in/100ms/s
     */
    public void start(Point[] points, double cruiseVel, double tarAccel) {
        start(PathGenerator.generatePath(points, cruiseVel, tarAccel));
    }

    /**
     * Starts the motion profile by filling the points.
     *
     * @param points
     *                      the points to put in the path generator
     * @param cruiseVel
     *                      the cruise velocity desired in in/100ms
     * @param tarAccel
     *                      the target acceleration desired in in/100ms/s
     */
    public void start(PathPoint[] pathPoints, boolean useAux) {
        startFilling(pathPoints, useAux);
        initRun = true;
    }

    public void start(PathPoint[] pathPoints) {
        start(pathPoints, true);
    }

    /**
     * Updates the state of the motion profile from the motor controller.
     * 
     * Note: Should be called periodically.
     */
    public void update() {
        System.out.println("bottom buffer: " + status.btmBufferCnt + " valid: " + status.activePointValid);
        switch (state) {
        case FillPoints:
            if (initRun) {
                initRun = false;
                setValue = SetValueMotionProfile.Disable;
                state = State.WaitPoints;
            }
            break;
        case WaitPoints:
            leaderTalon.getMotionProfileStatus(status);
            if (status.btmBufferCnt > PathConstants.MIN_POINTS_IN_TALON) {
                setValue = SetValueMotionProfile.Enable;
                state = State.Run;
                System.out.println("--> Executing profile...");
            }
            break;
        case Run:
            leaderTalon.getMotionProfileStatus(status);
            if (status.activePointValid && status.isLast) {
                System.out.println("--> Profile ended...");
                setValue = SetValueMotionProfile.Hold;
                state = State.FillPoints;
            }
            break;
        }
    }

    /**
     * Returns the output of the current set value. Should be passed to the set
     * method of the ultimate leader Talon.
     * 
     * @see SetValueMotionProfile
     * 
     * @return int
     */
    public int getOutput() {
        return setValue.value;
    }

    /**
     * Returns whether the current motion profile has finished executing.
     * 
     * @return if the profile is finished
     */
    public boolean isFinished() {
        return setValue == SetValueMotionProfile.Hold;
    }

    /**
     * Changes whether the path should be reversed or not. Must be called before
     * {@link #start(PathPoint[])}.
     *
     * @param reversed
     *                     if the robot should move backwards
     */
    public void setReverse(boolean reversed) {
        this.reversed = reversed;
    }

    /**
     * Clears the Motion profile buffer and resets state info.
     */
    public void reset() {
        leaderTalon.clearMotionProfileTrajectories();
        setValue = SetValueMotionProfile.Disable;
        state = State.FillPoints;
        initRun = false;
    }

    private void startFilling(PathPoint[] waypoints) {
        startFilling(waypoints, true);
    }

    /**
     * Starts filling the buffer with trajectory points.
     *
     * @param waypoints
     *                      the array of points created by the path generator
     */
    private void startFilling(PathPoint[] waypoints, boolean useAux) {
        //System.out.println("Started filling");
        int reverse = reversed ? -1 : 1;
        // Clear under run error
        if (status.hasUnderrun) {
            leaderTalon.clearMotionProfileHasUnderrun();
        }

        // Clear the buffer just in case the robot is still running some points
        leaderTalon.clearMotionProfileTrajectories();

        double totalTime = 0.0;
        for (int i = 0; i < waypoints.length; i++) {
            TrajectoryPoint tp = new TrajectoryPoint();
            tp.position = reverse * positionUnitConverter.apply(waypoints[i].position);
            tp.velocity = reverse * positionUnitConverter.apply(waypoints[i].velocity);
            totalTime += waypoints[i].time;
            // timeDur takes ms, but Pathpoint::time is in 100 ms
            tp.timeDur = (int) (waypoints[i].time * 100);
            // auxiliaryPos takes in units of 3600 ticks, but angle is in 360 degress
            tp.auxiliaryPos = waypoints[i].angle * 10;
            tp.useAuxPID = useAux;
            tp.profileSlotSelect0 = SwerveConstants.PID_PRIMARY_SLOT;
            tp.profileSlotSelect1 = SwerveConstants.PID_AUX_SLOT;
            tp.zeroPos = false;

            if (i == 0) {
                tp.zeroPos = true;
            }

            if (i == waypoints.length - 1) {
                tp.isLastPoint = true;
            }

            System.out.println(
                "TP: " + tp.position + "u, " + waypoints[i].velocity + "in/100ms (" + tp.velocity + "u/100ms), " + tp.timeDur + " ms, zero=" + tp.zeroPos + ", last=" + tp.isLastPoint
            );

            leaderTalon.pushMotionProfileTrajectory(tp);
        }
        System.out.println("--> Total time: " + (totalTime / 10.0) + " s");

    }
}
