// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.Current;
import frc.robot.subsystems.DriveTrain;

public class DriveCurrentMoniter extends CommandBase {
  private PowerDistribution powerDistributionPanel = new PowerDistribution();

  private DriveTrain driveTrainSubsystem;

  // set this to the max current at full throtle
  private final double maxCurrent = 30.0;

  private double leftCurrentCounter = 0;
  private double rightCurrentCounter = 0;

  // the state of the DriveTrain
  private boolean isStalled;

  /** Creates a new DriveCurrentMoniter. */
  public DriveCurrentMoniter(DriveTrain driveTrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.driveTrainSubsystem = driveTrainSubsystem;

    // Do not addRequirements(driveTrainSubsystem)
    // Do not need to add requirements because this is only reading values

    isStalled = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double frontRightCurrent = powerDistributionPanel.getCurrent(Current.FRONT_RIGHT_DRIVE);
    double backRightCurrent = powerDistributionPanel.getCurrent(Current.BACK_RIGHT_DRIVE);
    double frontLeftCurrent = powerDistributionPanel.getCurrent(Current.FRONT_LEFT_DRIVE);
    double backLeftCurrent = powerDistributionPanel.getCurrent(Current.BACK_LEFT_DRIVE);

    double rightCurrent = Math.max(frontRightCurrent, backRightCurrent);
    double leftCurrent = Math.max(frontLeftCurrent, backLeftCurrent);

    // dampening for current spikes
    rightCurrentCounter = rightCurrentCounter * 0.9 + rightCurrent * 0.1;
    leftCurrentCounter = leftCurrentCounter * 0.9 + leftCurrent * 0.1;

    rightCurrent = rightCurrentCounter;
    leftCurrent = leftCurrentCounter;

    boolean isRightStalled = rightCurrent / (driveTrainSubsystem.getRightPercent() + 0.1) > maxCurrent;
    boolean isLeftStalled = leftCurrent / (driveTrainSubsystem.getLeftPercent() + 0.1) > maxCurrent;

    if (isRightStalled || isLeftStalled)
    {
      isStalled = true;
    }
    else
    {
      isStalled = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isStalled() {
    return isStalled;
  }

  public PowerDistribution getPowerDistributionPanel() {
    return powerDistributionPanel;
  }
}
