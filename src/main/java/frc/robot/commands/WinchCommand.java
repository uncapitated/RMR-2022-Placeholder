// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controller;
import frc.robot.subsystems.ClimberSubsystem;

public class WinchCommand extends CommandBase {
  private ClimberSubsystem climberSubsystem;
  /** Creates a new WinchCommand. */
  public WinchCommand(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.climberSubsystem.set(Controller.Manipulator.getWinchButton().get() ? 0.01 : 0);
    System.out.println(Controller.Manipulator.getWinchButton());
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
