// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveStraightCommand extends CommandBase {
  private DriveTrainSubsystem driveTrainSubsystem;
  private int time;
  private int speed;
  private Instant startTime;

  /** Creates a new DriveStraightCommand. */
  public DriveStraightCommand(DriveTrainSubsystem driveTrainSubsystem, int time, int speed) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    startTime = Instant.now();
    System.out.println("Earlier test");
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrainSubsystem.set(new ChassisSpeeds(speed, speed, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    System.out.println("Test");
    return Instant.now().getEpochSecond() - startTime.getEpochSecond() > time;
  }
}
