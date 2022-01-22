// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CardinalShuffleboard;
import frc.robot.Controller;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

/**
 * Link to WPILib Command Based Programming
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html
 */

public class DriveCommand extends CommandBase {
  private DriveTrain driveTrainSubsystem;
  private Limelight limelight;

  private double maxForward = Math.sqrt(0.7); 
  private double maxTurn = Math.sqrt(0.5);

  /** Creates a new Drive. */
  public DriveCommand(DriveTrain in_driveTrainSubsystem, Limelight Limelight) {
    this.limelight = Limelight;
    driveTrainSubsystem = in_driveTrainSubsystem;
    
    addRequirements(limelight);
    // always add requirements for subsystems which are controlled
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.getLightValue().setNumber(3);
    // stop the robot from moving
    driveTrainSubsystem.set(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // grab entries from the shuffleboard
    maxForward = CardinalShuffleboard.getMaxForwardPowerEntry();
    maxTurn = CardinalShuffleboard.getMaxTurnPowerEntry();

    // smoothing of the forward and turn power is handled in controller
    double targetForwardPower = Controller.Drive.get_forward();
    double targetTurnPower = Controller.Drive.get_turn();
    boolean xButton = Controller.Drive.getXButton();

    // command subsystem
    if(xButton)
    {
      if(limelight.getHorizontalOffsetAngle().getDouble(0) > 3)
      {
        driveTrainSubsystem.set(0, -0.7);
      }
      if(limelight.getHorizontalOffsetAngle().getDouble(0) < -3)
      {
        driveTrainSubsystem.set(0, 0.7);
      }
    }
    else
    {
      //command subsystem
      driveTrainSubsystem.set(targetForwardPower * maxForward, targetTurnPower * maxTurn);
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

  public double getMaxForward() {
    return maxForward;
  }

  public double getMaxTurn() {
    return maxTurn;
  }
}