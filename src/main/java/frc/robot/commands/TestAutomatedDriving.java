// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;
import frc.robot.Constants.Autonomous;

public class TestAutomatedDriving extends CommandBase {
  private DriveTrain driveTrainSubsystem;

  private Trajectory currentTrajectory;
  private RamseteController controller;

  private Pose2d robotPosition;
  private double startTime;

  /** Creates a new TestAutomatedDriving. */
  public TestAutomatedDriving(DriveTrain driveTrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrainSubsystem = driveTrainSubsystem;

    addRequirements(driveTrainSubsystem);

    controller = new RamseteController();

    // setup trajectory
    TrajectoryConfig config = new TrajectoryConfig(Autonomous.MAX_SPEED_METERS_PER_SECOND, 
      Autonomous.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED).setKinematics(Constants.Drive.KINEMATICS);

    currentTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(
          new Translation2d(1, 1),
          new Translation2d(2, -1)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      // Pass config
      config
    );

    robotPosition = currentTrajectory.getInitialPose();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotPosition = currentTrajectory.getInitialPose();
    startTime = Timer.getFPGATimestamp();

    driveTrainSubsystem.setLeftAndRight(0, 0);
    driveTrainSubsystem.setBreak();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds = controller.calculate(robotPosition, currentTrajectory.sample(Timer.getFPGATimestamp() - startTime));
    robotPosition = currentTrajectory.sample(Timer.getFPGATimestamp() - startTime).poseMeters;

    DifferentialDriveWheelSpeeds wheelSpeeds = Constants.Drive.KINEMATICS.toWheelSpeeds(chassisSpeeds);

    driveTrainSubsystem.setLeftAndRight(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.setLeftAndRight(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime) > currentTrajectory.getTotalTimeSeconds();
  }
}
