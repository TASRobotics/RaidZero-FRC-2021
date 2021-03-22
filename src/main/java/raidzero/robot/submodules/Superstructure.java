package raidzero.robot.submodules;

import raidzero.robot.auto.actions.TurnTurretToAngle;
import raidzero.robot.auto.actions.TurnToGoal;
import raidzero.robot.auto.actions.ReusableSeriesAction;

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

    private boolean isAiming = false;
    private TurnToGoal aimAction;

    private boolean isAimingAndHood = false;
    private ReusableSeriesAction aimAndHoodAction;

    private boolean isTurretPIDing = false;
    private TurnTurretToAngle turretPIDAction;

    private boolean isCloseAligning = false;
    // private DrivePath closeAlignAction; 
    // ^ didn't copy over the DrivePath file because we were supposed to leave Drive stuff alone



    /**
     * define any actions to use during teleop here, as well as 
     *spline paths and waypoints
     *
     * private boolean is[ActionName] = false;
     * private Action [ActionName]Action;
     */


    @Override
    public void onStart(double timestamp) {
        aimAction = new TurnToGoal();
    }

    @Override
    public void update(double timestamp) {
        if(isAiming) aimAction.update();
    }

    @Override
    public void stop() {
        setAiming(false);
        setAimingAndHood(false);
        setTurretPIDing(false);
    }

    public boolean isAiming() {
        return isAiming;
    }

    public void setAiming(boolean status) {
        if (status == isAiming) {
            return;
        }
        isAiming = status;
        if (status) {
            // Don't aim if the robot is aiming and shooting already
            if (isAimingAndHood) {
                isAiming = false;
                return;
            }
            aimAction.start();
        } else {
            aimAction.done();
        }
    }

    public boolean isAimingAndHood() {
        return isAimingAndHood;
    }

    /**
     * Sets the aiming & shooting status.
     * 
     * @param status the status
     */
    public void setAimingAndHood(boolean status) {
        if (status == isAimingAndHood) {
            return;
        }
        isAimingAndHood = status;
        if (status) {
            aimAndHoodAction.start();
        } else {
            aimAndHoodAction.done();
        }
    }

    public boolean isTurretPIDing() {
        return isTurretPIDing;
    }

    public void setTurretPIDing(boolean status) {
        if (status == isTurretPIDing) {
            return;
        }
        isTurretPIDing = status;
        if (status) {
            if (isAiming || isAimingAndHood) {
                isTurretPIDing = false;
                return;
            }
            turretPIDAction.start();
        } else {
            turretPIDAction.done();
        }   
    }

    public boolean isUsingTurret() {
        return isAiming || isAimingAndHood || isTurretPIDing || isCloseAligning;
    }
}