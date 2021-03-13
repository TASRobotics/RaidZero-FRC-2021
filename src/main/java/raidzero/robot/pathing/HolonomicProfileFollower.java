package raidzero.robot.pathing;

import java.util.function.DoubleFunction;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import raidzero.pathgen.PathPoint;
import raidzero.robot.utils.EncoderUtils;

public class HolonomicProfileFollower extends ProfileFollower {

    BaseTalon rotor;
    ProfileFollower rotorProfile;

    public HolonomicProfileFollower(BaseTalon motor, BaseTalon rotor, 
        DoubleFunction<Double> motorUnitConverter,
        DoubleFunction<Double> rotorUnitConverter
    ) {
        super(motor, motorUnitConverter);
        rotorProfile = new ProfileFollower(rotor, rotorUnitConverter);
    }

    @Override
    public void start(PathPoint[] path) {
        // for (var pp : path) {
        //     System.out.println(
        //         (pp.time / 10.0) + "s " + pp.position + " in " + pp.velocity + " in/100ms " + pp.angle + " deg"
        //     );
        // }
        super.start(path, false, true);

        // Create profile for rotor angle
        PathPoint[] rotorPath = new PathPoint[path.length];
        for (int i = 0; i < path.length; ++i) {
            rotorPath[i] = new PathPoint();
        }
        rotorPath[0].position = path[0].angle;
        rotorPath[0].velocity = 0.0;
        rotorPath[0].time = 0.0;
        rotorPath[0].timeFromStart = 0.0;
        rotorPath[path.length - 1].position = path[path.length - 1].angle;
        rotorPath[path.length - 1].velocity = 0;
        for (int i = 1; i < path.length - 1; i++) {
            rotorPath[i].position = path[i].angle;
            rotorPath[i].velocity = (path[i + 1].angle - path[i - 1].angle) / (path[i + 1].time + path[i].time);
            rotorPath[i].time = path[i].time;
            rotorPath[i].timeFromStart = path[i].timeFromStart;
        }
        rotorPath[path.length - 1].time = path[path.length - 1].time;
        rotorPath[path.length - 1].timeFromStart = path[path.length - 1].timeFromStart;
        rotorProfile.start(rotorPath, false, false);
    }

    @Override
    public boolean isDoneWaitingForFill() {
        return super.isDoneWaitingForFill() && rotorProfile.isDoneWaitingForFill();
    }

    @Override
    public void enable() {
        super.enable();
        rotorProfile.enable();
    }

    @Override
    public void update() {
        super.update();
        rotorProfile.update();
    }

    @Override
    public void reset() {
        super.reset();
        rotorProfile.reset();
    }

    /**
     * Returns whether the motor & rotor motion profile haves finished executing.
     * 
     * @return if the profiles are finished
     */
    @Override
    public boolean isFinished() {
        return super.isFinished() && rotorProfile.isFinished();
    }

    /**
     * Returns the motor profile follower output.
     * 
     * @return motor motion profile output
     */
    public int getMotorOutput() {
        return getOutput();
    }

    /**
     * Returns the rotor profile follower output.
     * 
     * @return rotor motion profile output
     */
    public int getRotorOutput() {
        return rotorProfile.getOutput();//SetValueMotionProfile.Disable.value;
    }
}
