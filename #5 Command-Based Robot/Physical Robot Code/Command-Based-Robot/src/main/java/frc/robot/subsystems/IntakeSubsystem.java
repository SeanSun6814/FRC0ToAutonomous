package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private Spark intakeLeftMotor = new Spark(IntakeConstants.kLeftMotorPort);
    private Spark intakeRightMotor = new Spark(IntakeConstants.kRightMotorPort);

    public IntakeSubsystem() {
    }

    @Override
    public void periodic() {
    }

    public void setPosition(boolean open) {
        if (open) {
            intakeLeftMotor.set(IntakeConstants.kOpenSpeed);
            intakeRightMotor.set(IntakeConstants.kOpenSpeed);
        } else {
            intakeLeftMotor.set(IntakeConstants.kCloseSpeed);
            intakeRightMotor.set(IntakeConstants.kCloseSpeed);
        }
    }
}
