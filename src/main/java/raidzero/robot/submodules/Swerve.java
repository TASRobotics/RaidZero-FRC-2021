package raidzero.robot.submodules;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import raidzero.robot.Constants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.utils.JoystickUtils;
import com.ctre.phoenix.sensors.PigeonIMU;

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

    private static int numMotors;
    private static SwerveModule d;
    private static SwerveModule[] modules = new SwerveModule[4];
    private static PigeonIMU pigey = new PigeonIMU(0);
    private static double[] ypr = new double[3];

    // buffer variables
    private static double[][] rotV = new double[4][2];
    private static double[] totalV = new double[] {0,0};
    private static double[] basesChangeBuffer = new double[2];

    // Makes an entire swerve drive with NUMMOTORS/2 modules

    /**
     * Called at the start of autonomous or teleop.
     * 
     * @param timestamp
     */
    public void onStart(double timestamp) {}


    public void onInit() {
        numMotors = SwerveConstants.SWERVE_IDS.length;

        // init each module
        for (int i = 0; i < numMotors / 2; i++) {
            modules[i] = new SwerveModule();
            int[] motornums = new int[] { SwerveConstants.SWERVE_IDS[2 * i], SwerveConstants.SWERVE_IDS[2 * i + 1] };
            modules[i].onInit(motornums,SwerveConstants.INIT_MODULES_DEGREES[i],i+1);
            
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
            double moduleAngle = Math.PI / 4 + (Math.PI / 2) * i;
            rotV[i] = new double[] { -Math.sin(moduleAngle), Math.cos(moduleAngle)};
        }
        d = modules[0];
        zero();
    }


    /**
     * Runs components in the submodule that have continuously changing 
     * inputs.
     */
    public void run() {
        setHeading();
        for (SwerveModule module : modules) {
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
        zeroPigeon();
        for(SwerveModule mod : modules) {
            mod.zero();
        }
    }

    public void zeroPigeon() {
        pigey.setYaw(0);
    }

    public void Drive(double vX, double vY, double omega) {
        double mag = Math.sqrt(Math.pow(vX + (Constants.SQRTTWO * omega),2)
            + Math.pow(vY + (Constants.SQRTTWO * omega), 2));
        if(mag > 1) {
            mag = 1/(mag * Constants.SQRTTWO);
        } else {
            mag = 1;
        }
        for (int i = 0; i < modules.length; i++) {
            totalV[0] =  mag * (vX - omega * rotV[i][0]);
            totalV[1] =  mag * (vY - omega * rotV[i][1]);
            modules[i].setVectorVelocity(totalV);
        }
    }

    public void FieldOrientedDrive(double vX, double vY, double omega) {
        double rotangle = 2 * Math.PI / SwerveConstants.DEGREES_IN_REV * ypr[0];
        double newX = vX * Math.cos(rotangle) + vY * Math.sin(rotangle);
        double newY = -vX * Math.sin(rotangle) + vY * Math.cos(rotangle);
        Drive(newX, newY, omega);
    }

    private static void setHeading() {
        pigey.getYawPitchRoll(ypr);
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
        d.setVectorVelocity(
            new double[] {JoystickUtils.deadband(-c.getY(Hand.kLeft)), JoystickUtils.deadband(c.getX(Hand.kLeft))}
        );
        //d.setMotorVelocity(JoystickUtils.deadband(c.getY(Hand.kRight))* 40000);
        //d.setRotorPos(JoystickUtils.deadband(c.getY(Hand.kLeft)) * 360 / (4));
        if (c.getTriggerAxis(Hand.kLeft) > 0.5) {
            d.setRotorPos(90);
        }
    }

}