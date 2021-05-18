package raidzero.robot.wrappers;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class SendablePigeon extends PigeonIMU implements Sendable {

    protected double[] ypr = new double[3];

    public SendablePigeon(int deviceNumber) {
        super(deviceNumber);
    }
    
    public double getHeading() {
        getYawPitchRoll(ypr);
        return -ypr[0];
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", this::getHeading, null);
    }
}
