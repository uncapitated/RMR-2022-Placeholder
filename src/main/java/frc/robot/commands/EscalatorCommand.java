// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CardinalShuffleboard;
import frc.robot.Controller;
import frc.robot.subsystems.Escalator;

public class EscalatorCommand extends CommandBase {
  private Escalator Escalator;

  /** Creates a new Escalator. */
  public EscalatorCommand(Escalator Escalator) {
    this.Escalator = Escalator;
    addRequirements(Escalator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Escalator.set(Controller.Drive.getYButton()*.3);
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
