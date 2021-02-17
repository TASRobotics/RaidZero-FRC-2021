package raidzero.robot.pathing;

import java.util.function.DoubleFunction;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import raidzero.pathgen.PathPoint;
import raidzero.robot.utils.EncoderUtils;

public class HolonomicProfileFollower extends ProfileFollower {

    BaseTalon rotor;
    ProfileFollower rotorProfile;

    public HolonomicProfileFollower(BaseTalon motor, BaseTalon rotor, DoubleFunction<Double> positionUnitConverter) {
        super(motor, positionUnitConverter);
        rotorProfile = new ProfileFollower(rotor, EncoderUtils::rotorDegreesToTicks);
    }

    public void start(PathPoint[] path) {
        super.start(path, false);

        // Create profile for rotor angle
        PathPoint[] rotorPath = new PathPoint[path.length];
        rotorPath[0].position = path[0].angle;
        rotorPath[0].velocity = 0;
        rotorPath[path.length - 1].position = path[path.length - 1].angle;
        rotorPath[path.length - 1].velocity = 0;
        for (int i = 1; i < path.length - 1; i++) {
            rotorPath[i].position = path[i].angle;
            rotorPath[i].velocity = (path[i + 1].angle - path[i - 1].angle) / (path[i + 1].time - path[i - 1].time);
            rotorPath[i].time = path[i].time;
        }
        rotorProfile.start(rotorPath, false);
    }

    public void update() {
        super.update();
        rotorProfile.update();
    }

}
