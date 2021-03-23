package raidzero.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.Limelight;

public class DetermineGSCPath {
    
    public enum GSCPath {
        PATH_A_RED, PATH_A_BLUE, PATH_B_RED, PATH_B_BLUE, UNKNOWN
    }

    private static final Limelight limelight = Limelight.getInstance();

    public static GSCPath lookForPath() {
        limelight.setPipeline(1); // path A target

        double tx = limelight.getTx();
        double ta = limelight.getTa();
        System.out.println("Search: tx=" + tx + " ta=" + ta);
        if (Math.abs(tx) < 2.0 && ta > 0.05) {
            return GSCPath.PATH_A_RED;
        }
        if (Math.abs(tx) < 2.0 && ta > 0.020 && ta < 0.04) {
            return GSCPath.PATH_A_BLUE;
        }

        limelight.setPipeline(2); // path B target
        Timer.delay(0.1);

        tx = limelight.getTx();
        double ty = limelight.getTy();
        System.out.println("Search: tx=" + tx + " ty=" + ty);
        if (Math.abs(tx) > 3.0 && ty < 1.0) {
            return GSCPath.PATH_B_RED;
        }
        if (Math.abs(tx) > 3.0 && ty > 1.0) {
            return GSCPath.PATH_B_BLUE;
        }

        return GSCPath.UNKNOWN;
    }

}
