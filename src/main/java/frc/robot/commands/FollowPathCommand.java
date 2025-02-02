// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.Constants;

public class FollowPathCommand extends CommandBase {
  private DriveTrainSubsystem driveTrainSubsystem;

  private Trajectory currentTrajectory;
  private RamseteController controller;

  private Pose2d robotPosition;
  /** Use to align a trajectory if robot position is off the starting position */
  private Pose2d robotOffset;
  private double startTime;

  /** Creates a new TestAutomatedDriving. */
  public FollowPathCommand(DriveTrainSubsystem driveTrainSubsystem, Trajectory toFollow) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrainSubsystem = driveTrainSubsystem;

    addRequirements(driveTrainSubsystem);

    controller = new RamseteController();

    currentTrajectory = toFollow;

    // calculates offset that aligns the current robot pos to the path
    robotOffset = driveTrainSubsystem.getCalculatedRobotPose().relativeTo(currentTrajectory.getInitialPose());
    robotPosition = currentTrajectory.getInitialPose();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // calculates offset that aligns the current robot pos to the path
    robotOffset = driveTrainSubsystem.getCalculatedRobotPose().relativeTo(currentTrajectory.getInitialPose());
    robotPosition = currentTrajectory.getInitialPose();
    startTime = Timer.getFPGATimestamp();

    driveTrainSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // this calculation ignores sensors
    //robotPosition = currentTrajectory.sample(Timer.getFPGATimestamp() - startTime).poseMeters;

    // this calculation uses sensors (uses the offset to align the robot pos to the path)
    robotPosition = driveTrainSubsystem.getCalculatedRobotPose().relativeTo(robotOffset);

    ChassisSpeeds chassisSpeeds = controller.calculate(robotPosition, currentTrajectory.sample(Timer.getFPGATimestamp() - startTime));

    DifferentialDriveWheelSpeeds wheelSpeeds = Constants.Drive.KINEMATICS.toWheelSpeeds(chassisSpeeds);

    driveTrainSubsystem.set(wheelSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime) > currentTrajectory.getTotalTimeSeconds();
  }
}
