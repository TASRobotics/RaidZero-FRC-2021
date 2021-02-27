package raidzero.robot.submodules;

import edu.wpi.first.wpilibj.controller.PIDController;
import raidzero.pathgen.PathGenerator;
import raidzero.robot.Constants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.pathing.HolonomicPath;
import raidzero.robot.pathing.Path;
import com.ctre.phoenix.sensors.PigeonIMU;
import org.apache.commons.math3.util.FastMath;

public class Swerve extends Submodule {

    private enum ControlState {
        OPEN_LOOP, PATHING
    };

    private static Swerve instance = null;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
    }

    private SwerveModule[] modules = new SwerveModule[4];
    private PigeonIMU pigey = new PigeonIMU(0);

    private double[] ypr = new double[3];

    private double omega = 0.0;
    // private double targetAngle = 0.0;
    // private double headingError = 0.0;
    private PIDController headingPID;

    // private SwerveModule d;

    // buffer variables
    private double[][] rotV = new double[4][2];
    private double[] totalV = new double[] {0, 0};

    private ControlState controlState = ControlState.OPEN_LOOP;

    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;
        headingPID.reset();
        headingPID.setSetpoint(0.0);
    }

    public void onInit() {
        // Total number of modules in the swerve
        int motorCount = SwerveConstants.SWERVE_IDS.length;

        // Create and initialize each module
        for (int i = 0; i < motorCount / 2; i++) {
            modules[i] = new SwerveModule();
            modules[i].onInit(SwerveConstants.SWERVE_IDS[2 * i],
                    SwerveConstants.SWERVE_IDS[2 * i + 1], SwerveConstants.INIT_MODULES_DEGREES[i],
                    i + 1);

            /**
             * moduleAngle: the direction of the vector that is +90deg offset from each modules
             * radius vector modulePos: the components of the rotation vector positions { I: j, k
             * II: -j, k III: -j, -k IV: j, -k }
             */
            double moduleAngle = Math.PI / 4 + (Math.PI / 2) * i;
            rotV[i] = new double[] {-Math.sin(moduleAngle), Math.cos(moduleAngle)};
        }
        // d = modules[0];

        headingPID = new PIDController(SwerveConstants.HEADING_KP, SwerveConstants.HEADING_KI,
                SwerveConstants.HEADING_KD);
        headingPID.setTolerance(5);

        zero();
    }

    @Override
    public void update(double timestamp) {
        // Retrive the pigeon's gyro values
        pigey.getYawPitchRoll(ypr);
        for (SwerveModule module : modules) {
            module.update(timestamp);
        }
    }

    /**
     * Runs components in the submodule that have continuously changing inputs.
     */
    @Override
    public void run() {
        for (SwerveModule module : modules) {
            module.run();
        }
    }

    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;
        totalV = new double[] {0.1, 0};

        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    /**
     * Resets the sensor(s) to zero.
     */
    public void zero() {
        // targetAngle = 0;
        totalV = new double[] {0.1, 0};

        zeroHeading();
        for (SwerveModule mod : modules) {
            mod.zero();
        }
    }

    /**
     * Zeroes the heading of the swerve.
     */
    public void zeroHeading() {
        pigey.setYaw(0);
    }

    /**
     * Drives the swerve.
     * 
     * @param vX     velocity in the x direction
     * @param vY     velocity in the y direction
     * @param omegaR angular velocity of the swerve
     */
    public void drive(double vX, double vY, double omegaR) {
        // backslash fix note: problem is above this method
        double mag = FastMath.hypot(vX + (Constants.SQRTTWO * omegaR / 2),
                vY + (Constants.SQRTTWO * omegaR / 2));
        double coef = 1;
        if (mag > 1) {
            coef = 1 / mag;
        }
        for (int i = 0; i < modules.length; i++) {
            totalV[0] = (vX - omegaR * rotV[i][0]);
            totalV[1] = (vY - omegaR * rotV[i][1]);
            modules[i].setVectorVelocity(totalV, coef);
        }
    }

    public void fieldOrientedDrive(double vX, double vY, double rX, double rY) {
        // translational adjustment to move w/ respect to the field
        double heading = Math.toRadians(ypr[0]);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double newX = vX * cos + vY * sin;
        double newY = -vX * sin + vY * cos;

        // TODO(jimmy): Absolute turning instead of relative?
        // rotational adjustment to PID to the directed heading
        // if (Math.abs(rX + rY) > 0.01) {
        // targetAngle = Math.toDegrees(Math.atan2(-rX, rY));
        // }
        // headingError = targetAngle - ypr[0];
        // omega = headingPID.calculate(headingError);
        // if (omega > 1)
        // omega = 1;
        // if (omega < -1)
        // omega = -1;

        // for relative control
        omega = rX;

        // send new directions to drive
        drive(newX, newY, omega);
    }

    /**
     * Sets the positions of all the rotors on the swerve modules.
     * 
     * @param angle angle in degree
     */
    public void setRotorPositions(double angle) {
        for (SwerveModule module : modules) {
            module.setRotorPos(angle);
        }
    }

    /**
     * Returns the position of the rotor on the specified module.
     * 
     * @param moduleId ID of the swerve module
     * @return rotor position in degrees [0, 360)
     */
    public double getModuleRotorPosition(int moduleId) {
        return (modules[moduleId].getRotorPosition() * 360.0) % 360.0;
    }

    /**
     * Executes a holonomic path.
     * 
     * @param path the holonomic path to execute
     */
    public void executeHolonomicPath(HolonomicPath path) {
        if (controlState == ControlState.PATHING) {
            return;
        }
        controlState = ControlState.PATHING;
        // Paths for the each modules are generated based on the holonomic path.
        for (int i = 0; i < modules.length; i++) {
            modules[i].executePath(new Path(PathGenerator.generatePath(path.getPathPoints(),
                    Constants.SwerveConstants.MODULE_ANGLES[i],
                    Constants.SwerveConstants.ROBOT_RADIUS)));
        }
    }

    /**
     * Returns whether the swerve has finished following a path.
     * 
     * @return if the swerve is finished pathing
     */
    public boolean isFinishedWithPath() {
        if (controlState != ControlState.PATHING) {
            return false;
        }
        for (SwerveModule module : modules) {
            if (!module.isFinishedWithPath()) {
                return false;
            }
        }
        return true;
    }

    // public void test(XboxController c) {
    // if (c.getAButtonPressed()) {
    // d = modules[0];
    // }
    // if (c.getBButtonPressed()) {
    // d = modules[1];
    // }
    // if (c.getXButtonPressed()) {
    // d = modules[2];
    // }
    // if (c.getYButtonPressed()) {
    // d = modules[3];
    // }
    // d.setVectorVelocity(new double[] {JoystickUtils.deadband(-c.getY(Hand.kLeft)),
    // JoystickUtils.deadband(c.getX(Hand.kLeft))}, 1);
    // // d.setMotorVelocity(JoystickUtils.deadband(c.getY(Hand.kRight))* 40000);
    // // d.setRotorPos(JoystickUtils.deadband(c.getY(Hand.kLeft)) * 360 / (4));
    // if (c.getTriggerAxis(Hand.kLeft) > 0.5) {
    // d.setRotorPos(90);
    // }
    // }
}
