package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class Imu extends PigeonIMU implements Sendable  {
    public Imu(int port) {
        super(port);
    }
    public double yaw(){
        double[] ypr = {0,0,0};
        getYawPitchRoll(ypr);
        return ypr[0] % 360;
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", this::yaw, null);
    }
   
}