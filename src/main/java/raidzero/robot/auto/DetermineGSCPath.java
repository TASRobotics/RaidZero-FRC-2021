package raidzero.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.Limelight;

public class DetermineGSCPath {
    
    public enum GSCPath {
        PATH_A_RED, PATH_A_BLUE, PATH_B_RED, PATH_B_BLUE, UNKNOWN
    }

    private static final Limelight limelight = Limelight.getInstance();

    public static GSCPath lookForPath() {
        limelight.setPipeline(0); // path A target

        double tx = limelight.getTx();
        double ta = limelight.getTa();
        if (Math.abs(tx) < 2.0 && ta > 0.0075) {
            return GSCPath.PATH_A_RED;
        }
        if (Math.abs(tx) < 2.0 && ta > 0.00040 && ta < 0.00050) {
            return GSCPath.PATH_A_BLUE;
        }

        limelight.setPipeline(1); // path B target
        Timer.delay(0.1);

        tx = limelight.getTx();
        double ty = limelight.getTy();
        if (Math.abs(tx) < 2.0 && ty < 3.5) {
            return GSCPath.PATH_B_RED;
        }
        if (Math.abs(tx) < 2.0 && ty > 4.0) {
            return GSCPath.PATH_B_BLUE;
        }

        return GSCPath.UNKNOWN;
    }

}
