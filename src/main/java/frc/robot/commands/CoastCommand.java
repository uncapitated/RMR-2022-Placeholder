// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class CoastCommand extends CommandBase {
  public DriveTrainSubsystem driveTrain;

  /** Creates a new CoastCommand. */
  public CoastCommand(DriveTrainSubsystem driveTrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    driveTrain = driveTrainSubsystem;
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setCoast();
    driveTrain.setPercent(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.setPercent(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setPercent(0, 0);
    driveTrain.setCoast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
