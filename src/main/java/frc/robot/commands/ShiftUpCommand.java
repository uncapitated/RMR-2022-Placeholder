// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ShiftUpCommand extends CommandBase {
  private DriveTrainSubsystem driveTrain;

  private double startTime;

  /** Creates a new ShiftUpCommand. 
   * This command runs for 0.1 seconds in coast mode, shifts the gear, and waits 0.1 seconds in coast mode
  */
  public ShiftUpCommand(DriveTrainSubsystem driveTrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = driveTrainSubsystem;
    startTime = Timer.getFPGATimestamp();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - startTime > 0.1)
    {
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > 0.2;
  }
}
