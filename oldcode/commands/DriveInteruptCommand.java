// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controller;
import frc.robot.subsystems.DriveTrain;

public class DriveInteruptCommand extends CommandBase {
  private DriveTrain driveTrainSubsystem;

  /** Creates a new InteruptDriveCommand. */
  public DriveInteruptCommand(DriveTrain driveTrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrainSubsystem = driveTrainSubsystem;
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // frees the motors and does not try to hold them
    driveTrainSubsystem.setCoast();
    Controller.Drive.setRumble(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< Updated upstream:oldcode/commands/DriveInteruptCommand.java
    driveTrainSubsystem.set(0, 0);
=======
    if (levitation.top.get() || levitation.bottom.get()) {
      levitation.set(Controller.Drive.get_secondary_vertical_stick() * 0.6);
    }
>>>>>>> Stashed changes:src/main/java/frc/robot/commands/WingardiumLeviosa.java
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // motors will attempt to hold their position
    driveTrainSubsystem.setBreak();
    Controller.Drive.setRumble(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
