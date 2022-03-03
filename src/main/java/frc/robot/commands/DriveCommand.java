// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Controller;
import frc.robot.Controller.Drive;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.SHIFTER_POSITION;

/**
 * Link to WPILib Command Based Programming
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html
 */

public class DriveCommand extends CommandBase {
  private DriveTrainSubsystem driveTrainSubsystem;

  // in m/s
  private double maxForward = 3.0;
  // in rad/s
  private double maxTurn = 3.5;


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
    driveTrainSubsystem.setCoast();

    forwardLimiter.reset(Constants.Drive.KINEMATICS.toChassisSpeeds(new DifferentialDriveWheelSpeeds(driveTrainSubsystem.getLeftSpeed(), driveTrainSubsystem.getRightSpeed())).vxMetersPerSecond);
    turnLimiter.reset(Constants.Drive.KINEMATICS.toChassisSpeeds(new DifferentialDriveWheelSpeeds(driveTrainSubsystem.getLeftSpeed(), driveTrainSubsystem.getRightSpeed())).omegaRadiansPerSecond);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // smoothing of the forward and turn power is handled in controller
    double targetForwardPower = Controller.Drive.get_forward();
    double targetTurnPower = Controller.Drive.get_turn();

    double forwardVelocity = targetForwardPower * maxForward;
    double angularVelocity = targetTurnPower * maxTurn;

    if (Controller.Drive.getSlowButton()) {
      forwardVelocity *= 0.5;
      angularVelocity *= 0.3;

      (new SequentialCommandGroup(new ShiftDownCommand(driveTrainSubsystem), this)).schedule();
    } else {
      (new SequentialCommandGroup(new ShiftDownCommand(driveTrainSubsystem), this)).schedule();
    }

    //command subsystem
    driveTrainSubsystem.set(new ChassisSpeeds(forwardLimiter.calculate(forwardVelocity), 0, turnLimiter.calculate(angularVelocity)));

    /*
    if(Controller.Drive.getTriggerLeft().get()){
      driveTrainSubsystem.setShifter(DriveTrainSubsystem.SHIFTER_POSITION.HIGH);
    } else if (Controller.Drive.getTriggerRight().get()){
      driveTrainSubsystem.setShifter(DriveTrainSubsystem.SHIFTER_POSITION.LOW);
    }/**/
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