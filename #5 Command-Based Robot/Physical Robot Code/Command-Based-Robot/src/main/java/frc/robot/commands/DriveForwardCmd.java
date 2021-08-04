
package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveForwardCmd extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final double distance;

    public DriveForwardCmd(DriveSubsystem driveSubsystem, double distance) {
        this.driveSubsystem = driveSubsystem;
        this.distance = driveSubsystem.getEncoderMeters() + distance;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("DriveForwardCmd started!");
    }

    @Override
    public void execute() {
        driveSubsystem.setMotors(DriveConstants.kAutoDriveForwardSpeed, DriveConstants.kAutoDriveForwardSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setMotors(0, 0);
        System.out.println("DriveForwardCmd ended!");
    }

    @Override
    public boolean isFinished() {
        if (driveSubsystem.getEncoderMeters() > distance)
            return true;
        else
            return false;
    }
}
