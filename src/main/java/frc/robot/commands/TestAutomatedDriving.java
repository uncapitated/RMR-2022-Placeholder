// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gyroscope;

public class TestAutomatedDriving extends CommandBase {
  private Gyroscope gyroscope;

  /** Creates a new TestAutomatedDriving. */
  public TestAutomatedDriving(Gyroscope gyroscope) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gyroscope = gyroscope;
    addRequirements(gyroscope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.print(gyroscope.getGyro().getGyroAngleX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
