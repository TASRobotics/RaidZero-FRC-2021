package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

public class WaitAction implements Action {

    private Timer timer = new Timer();
    private double duration;

    public WaitAction(double duration) {
        this.duration = duration;
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void start() {
        timer.reset();
        timer.start();
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        timer.stop();
    }
}