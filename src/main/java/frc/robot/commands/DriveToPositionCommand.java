// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * automatically generates a path to the position when the command is initialized and then creates a follow path command and runs while that is running
 */
public class DriveToPositionCommand extends CommandBase {
  // trajectory constraints
  private TrajectoryConfig config;

  // thing to control
  DriveTrainSubsystem driveTrain;

  // command that this class creates to drive to the target pos
  FollowPathCommand autoDriveCommand;

  // position
  Pose2d targetPose;

  /** Creates a new DriveToPositionCommand. */
  public DriveToPositionCommand(DriveTrainSubsystem driveTrainSubsystem, Pose2d position) {
    this(driveTrainSubsystem, position, false);
  }

  /** Creates a new DriveToPositionCommand. */
  public DriveToPositionCommand(DriveTrainSubsystem driveTrainSubsystem, Pose2d position, boolean isReversed) {
    // setup
    config = new TrajectoryConfig(Constants.Autonomous.MAX_SPEED_METERS_PER_SECOND, 
      Constants.Autonomous.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED).setKinematics(Constants.Drive.KINEMATICS);
    
    config.setReversed(isReversed);

    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = driveTrainSubsystem;
    // Don't add this subsystem as a requirement because the command that it calls will require the subsystem to be free
    //addRequirements(driveTrainSubsystem);

    targetPose = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Trajectory path = TrajectoryGenerator.generateTrajectory(List.of(driveTrain.getCalculatedRobotPose(), targetPose), config);

    // create command
    autoDriveCommand = new FollowPathCommand(driveTrain, path);

    autoDriveCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return autoDriveCommand.isFinished();
  }
}
