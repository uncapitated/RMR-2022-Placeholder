// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controller;
import frc.robot.subsystems.SpinnyThing;

public class SpinTheSpinner extends CommandBase {
  /** Creates a new SpinTheSpinner. */
  private final SpinnyThing spinnyThing;
  public SpinTheSpinner(SpinnyThing spinnyThing) {
    this.spinnyThing = spinnyThing;
    addRequirements(spinnyThing);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spinnyThing.set(Controller.Drive.get_right_stick_horizontal() * 0.1);
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
