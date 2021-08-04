package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private Spark elevatorMotor = new Spark(ElevatorConstants.kMotorPort);
    private Encoder encoder = new Encoder(//
            ElevatorConstants.kEncoderChannelA, ElevatorConstants.kEncoderChannelB);

    public ElevatorSubsystem() {
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator encoder value", getEncoderMeters());
    }

    public void setMotor(double speed) {
        elevatorMotor.set(speed);
    }

    public double getEncoderMeters() {
        return encoder.get() * ElevatorConstants.kEncoderTick2Meter;
    }
}
