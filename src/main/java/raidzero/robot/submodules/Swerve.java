package raidzero.robot.submodules;

import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.submodules.Submodule;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Swerve extends Submodule {
    private static Swerve instance = null;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
    }

    private static enum ModuleLocation {
        I(0), II(1), III(2), IV(3);

        public int label;

        private ModuleLocation(int label) {
        this.label = label;
        }
    };

    private static int numMotors;
    private static SwerveModule d;
    private static SwerveModule[] modules = new SwerveModule[4];

    // buffer variables
    private static double[][] rotV = new double[4][2];
    private static double[] totalV = new double[2];
    private static double[] basesChangeBuffer = new double[2];

    // Makes an entire swerve drive with NUMMOTORS/2 modules

    /**
     * Called at the start of autonomous or teleop.
     * 
     * @param timestamp
     */
    public void onStart(double timestamp) {}


    public void onInit(int[] motorarray) {
        numMotors = motorarray.length;

        // init each module
        for (int i = 0; i < numMotors / 2; i++) {
            modules[i] = new Module();
            int[] motornums = new int[] { motorarray[2 * i], motorarray[2 * i + 1] };
            modules[i].onInit(motornums);
            
            /**
             * moduleAngle: the direction of the vector that is +90deg offset from each modules radius vector
             * modulePos: the components of the rotation vector
             *  positions {
             *      I: j, k
             *      II: -j, k
             *      III: -j, -k
             *      IV: j, -k
             *  }
             */
            double moduleAngle = Math.PI / 4 + SwerveConstants.QUARTER_RADIANS * i;
            rotV[i] = new double[] { Math.cos(moduleAngle), Math.sin(moduleAngle)};
        }
        d = modules[0];
    }


    /**
     * Runs components in the submodule that have continuously changing 
     * inputs.
     */
    public void run() {
        for (SwerveModeule module : modules) {
            module.run();
        }
    }

    /**
     * Stops the submodule.
     */
    public void stop() {

    }

    /**
     * Resets the sensor(s) to zero.
     */
    public void zero() {
        modules.forEach(SwerveModule (o -> o.zero()));
    }

    public void Drive(double Vx, double Vy, double omega) {
        for (int i = 0; i < modules.length; i++) {
            totalV[0] = Vx - omega * rotV[i][0];
            totalV[1] = Vy - omega * rotV[i][1];
            modules[i].setVectorVelocity(robotToModuleVector(totalV));
        }
    }

    public void Drive(double Vx, double Vy, double omega, double heading) {
        double rotangle = 2 * Math.PI / SwerveConstants.DEGREES_IN_REV * heading;
        double newVx = Vx * Math.cos(rotangle) + Vy * Math.sin(rotangle);
        double newVy = Vx * Math.sin(rotangle) + Vy * Math.cos(rotangle);
        Drive((double) newVx, (double) newVy, omega);
    }

    public double[] robotToModuleVector(double[] v) {
        basesChangeBuffer[0] = v[1];
        basesChangeBuffer[1] = -v[0];
        return basesChangeBuffer;
    }

    public double[] moduleToRobotVector(double[] v) {
        basesChangeBuffer[0] = -v[1];
        basesChangeBuffer[1] = v[0];
        return basesChangeBuffer;
    }

    public void test(XboxController c) {
        if (c.getAButtonPressed()) {
            d = modules[0];
        }
        if (c.getBButtonPressed()) {
            d = modules[1];
        }
        if (c.getXButtonPressed()) {
            d = modules[2];
        }
        if (c.getYButtonPressed()) {
            d = modules[3];
        }
        d.setMotorVelocity(c.getY(Hand.kRight) * 40000);
        d.setRotorPos(c.getY(Hand.kLeft) / 4);
        if (c.getTriggerAxis(Hand.kLeft) > 0.5) {
            d.setRotorPos(0.25);
        }
    }

}