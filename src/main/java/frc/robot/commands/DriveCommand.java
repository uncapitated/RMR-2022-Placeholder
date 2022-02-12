// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Controller;
import frc.robot.subsystems.DriveTrain;

/**
 * Link to WPILib Command Based Programming
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html
 */

public class DriveCommand extends CommandBase {
  private DriveTrain driveTrainSubsystem;

  // in m/s
  private double maxForward = 0.7;
  // in rad/s
  private double maxTurn = 0.1;

  /** Creates a new Drive. */
  public DriveCommand(DriveTrain in_driveTrainSubsystem) {
    driveTrainSubsystem = in_driveTrainSubsystem;
    // always add requirements for subsystems which are controlled
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // stop the robot from moving
    driveTrainSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // smoothing of the forward and turn power is handled in controller
    double targetForwardPower = Controller.Drive.get_forward();
    double targetTurnPower = Controller.Drive.get_turn();

    //command subsystem
    ChassisSpeeds robotSpeeds = new ChassisSpeeds(targetForwardPower * maxForward, 0, targetTurnPower * maxTurn);
    driveTrainSubsystem.set(Constants.Drive.KINEMATICS.toWheelSpeeds(robotSpeeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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