// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.Controller;
import frc.robot.Controller.Drive;

public class LimelightCommand extends CommandBase {
  private Limelight limelight;
  private DriveTrain driveTrain;

  /** Creates a new LimelightCommand. */
  public LimelightCommand(Limelight Limelight, DriveTrain DriveTrain) {
    this.limelight = Limelight;
    addRequirements(Limelight);

    this.driveTrain = DriveTrain;
    addRequirements(DriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.getLightValue().setNumber(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println(limelight.getHorizontalOffsetAngle().getDouble(0));
    if(Controller.Drive.get_x_button())
    {
      if(limelight.getHorizontalOffsetAngle().getDouble(0) > 0.5)
      {
        driveTrain.set(0, 0.1);
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.getLightValue().setNumber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
