// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Controller;
import frc.robot.Controller.Drive;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * Link to WPILib Command Based Programming
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html
 */

public class DriveCommand extends CommandBase {
  private DriveTrainSubsystem driveTrainSubsystem;

  // in m/s
  private double maxForward = 3.0;
  // in rad/s
  private double maxTurn = 2.8;


  private SlewRateLimiter forwardLimiter = new SlewRateLimiter(Constants.Drive.DRIVE_MAX_ACCEL);
  private SlewRateLimiter turnLimiter = new SlewRateLimiter(Constants.Drive.DRIVE_MAX_ANGLE_ACCEL);

  /** Creates a new Drive. */
  public DriveCommand(DriveTrainSubsystem in_driveTrainSubsystem) {
    driveTrainSubsystem = in_driveTrainSubsystem;
    // always add requirements for subsystems which are controlled
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // stop the robot from moving
    driveTrainSubsystem.stop();
    driveTrainSubsystem.setCoast();

    forwardLimiter.reset(0);
    turnLimiter.reset(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // smoothing of the forward and turn power is handled in controller
    double targetForwardPower = Controller.Drive.get_forward();
    double targetTurnPower = Controller.Drive.get_turn();

    //command subsystem
    driveTrainSubsystem.set(new ChassisSpeeds(forwardLimiter.calculate(targetForwardPower * maxForward), 0, turnLimiter.calculate(targetTurnPower * maxTurn)));

    if(Controller.Drive.getTriggerLeft().get()){
      driveTrainSubsystem.setShifter(DriveTrainSubsystem.SHIFTER_POSITION.HIGH);
    } else if (Controller.Drive.getTriggerRight().get()){
      driveTrainSubsystem.setShifter(DriveTrainSubsystem.SHIFTER_POSITION.LOW);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();
    driveTrainSubsystem.setBreak();
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