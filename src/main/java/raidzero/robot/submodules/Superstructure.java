package raidzero.robot.submodules;

import java.util.Arrays;

import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.ReusableSeriesAction;
import raidzero.robot.auto.actions.TurnToGoal;
import raidzero.robot.auto.actions.TurnTurretToAngle;
import raidzero.robot.auto.actions.VisionAssistedTargeting;
import raidzero.pathgen.Point;
import raidzero.robot.pathing.Path;

public class Superstructure extends Submodule {

    private static Superstructure instance = null;

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    private Superstructure() {
    }

    /**
     * define any actions to use during teleop here, as well as 
     *spline paths and waypoints
     *
     * private boolean is[ActionName] = false;
     * private Action [ActionName]Action;
     */


    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void stop() {
    }
}