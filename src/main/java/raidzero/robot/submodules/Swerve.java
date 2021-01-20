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

    private Swerve {
    }

    private static enum ModuleLocation {
        I(0), II(1), III(2), IV(3);

        public final int label;

        private ModuleLocation(int label) {
        this.label = label;
        }
    };

    private int numMotors;
    private static Module d;
    private Module[] modules = new Module[4];
    private double[] moduleAngles = new double[2];
    private double[][] rotV = new double[4][2];
    private static final double DEGREES_PER_ROT = 360;

    // Makes an entire swerve drive with NUMMOTORS/2 modules

    public void onInit(int[] motorarray, double[] modAngles) {
        this.numMotors = motorarray.length;
        this.moduleAngles = modAngles;
        for (int i = 0; i < this.numMotors / 2; i++) {
        this.modules[i] = new Module();
        int[] motornums = new int[] { motorarray[2 * i], motorarray[2 * i + 1] };
        this.modules[i].onInit(motornums);

        
        double moduleAngle = Math.PI / 4 + Math.PI / 2 * i;
        double[] modulePos = { Math.cos(moduleAngle), Math.sin(moduleAngle)};
        double rotX =  modulePos[0]*Math.cos(Math.PI/2)-modulePos[1]*Math.sin(Math.PI/2);
        double rotY =  modulePos[0]*Math.sin(Math.PI/2)+modulePos[1]*Math.cos(Math.PI/2);
        this.rotV[i] = new double[] {rotX, rotY};
        }
        d = modules[0];
    }

    public void Drive(double Vx, double Vy, double omega) {
        for (int i = 0; i < modules.length; i++) {
        double[] totalV = new double[] { Vx - omega * rotV[i][0], Vy - omega * rotV[i][1] };
        modules[i].setVectorVelocity(totalV);
        modules[i].run();
        }
    }

    public void Drive(double Vx, double Vy, double omega, double heading) {
        double rotangle = 2 * Math.PI / DEGREES_PER_ROT * heading;
        double newVx = Vx * Math.cos(rotangle) - Vy * Math.sin(rotangle);
        double newVy = Vx * Math.sin(rotangle) + Vy * Math.cos(rotangle);
        Drive((double) newVx, (double) newVy, omega);
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