// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class LimelightCommand extends CommandBase {
  private Limelight limelight;
  private DriveTrain driveTrain;

  /** Creates a new LimelightCommand. */
  public LimelightCommand(Limelight Limelight, DriveTrain DriveTrain) {
    // do not add the requirement of the limelight because multiple things can use it at the same time
    this.limelight = Limelight;

    this.driveTrain = DriveTrain;
    addRequirements(DriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set the light to be on
    limelight.getLightValue().setNumber(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.getHorizontalOffsetAngle().getDouble(0) > 3)
    {
      driveTrain.set(0, -0.7);
    }
    else if(limelight.getHorizontalOffsetAngle().getDouble(0) < -3)
    {
      driveTrain.set(0, 0.7);
    }
    else
    {
      //smoothly land in the middle
      driveTrain.set(0, -0.233 * limelight.getHorizontalOffsetAngle().getDouble(0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // set the light to be off
    limelight.getLightValue().setNumber(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (limelight.getHasValidTargets().getBoolean(false)) {
      return true;
    }

    if (Math.abs(limelight.getHorizontalOffsetAngle().getDouble(0)) < 1.5)
    {
      return true;
    }

    return false;
  }
}
