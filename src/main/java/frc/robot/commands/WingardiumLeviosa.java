// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controller;
import frc.robot.subsystems.Levitation;

public class WingardiumLeviosa extends CommandBase {
  private Levitation levitation;
  /** Creates a new WingardiumLeviosa. */
  public WingardiumLeviosa(Levitation levitation) {
    this.levitation = levitation;
    addRequirements(levitation);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!levitation.top.get() || !levitation.bottom.get()) {
      levitation.set(Controller.Drive.get_secondary_vertical_stick() * 0.6);
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
}
