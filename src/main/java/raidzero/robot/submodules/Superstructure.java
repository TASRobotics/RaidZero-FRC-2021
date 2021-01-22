package raidzero.robot.submodules;

import java.util.Arrays;


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