package raidzero.robot.submodules;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class Led extends Submodule {

    private static Led instance = null;

    public static Led getInstance() {
        if (instance == null) {
            instance = new Led();
        }
        return instance;
    }

    private Led() {
    }

    private static I2C leds;
    private static byte[] data = new byte[1];

    /**
     * Called at the start of autonomous or teleop.
     * 
     * @param timestamp
     */
    public void onStart(double timestamp) {
        glowPattern(0);
    }

    /**
     * Called once when the submodule is initialized.
     */
    public void onInit() {
        leds = new I2C(Port.kOnboard, 4);
        glowPattern(0);
    }

    public void glowPattern(int patternId) {
        data[0] = (byte) patternId;
        if (patternId < 256) leds.transaction(data, 1, new byte[1], 0);
        else return;
    }

    /**
     * Reads cached inputs & calculate outputs.
     */
    public void update(double timestamp) {}
    
    /**
     * Runs components in the submodule that have continuously changing 
     * inputs.
     */
    public void run() {}

    public void stop() {
        glowPattern(1);
    }

    /**
     * Resets the sensor(s) to zero.
     */
    public void zero() {}
}