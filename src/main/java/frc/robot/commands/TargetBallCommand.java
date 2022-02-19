// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivePID;
import frc.robot.Controller.Drive;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TargetBallCommand extends CommandBase {
  NetworkTable table, ck;

  private DriveTrainSubsystem driveTrain;

  private PIDController tpid;

  double xmin, ymin, xmax, ymax, avX, degrees;

  private double turnSpeed;
  private double forwardSpeed;
  /** Creates a new TargetBallCommand. */
  public TargetBallCommand(DriveTrainSubsystem DriveTrain) {

    this.driveTrain = DriveTrain;

    tpid = new PIDController(DrivePID.akP,DrivePID.akI,DrivePID.akD);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    table = NetworkTableInstance.getDefault().getTable("ML/detections");

    //Note: this is for use to check if the coral is attached, first must check how this is sent to NetworkTables. 
    ck = NetworkTableInstance.getDefault().getTable("ML/coral");

    driveTrain.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xmin = table.getEntry("xmin").getDouble(0);
    ymin = table.getEntry("ymin").getDouble(0);
    xmax = table.getEntry("xmax").getDouble(0);
    ymax = table.getEntry("ymax").getDouble(0);

    avX = (xmin+xmax)/2;

    degrees = (DrivePID.maxX/2-avX)/(DrivePID.maxX/2)*180;

    turnSpeed = -1*tpid.calculate(degrees, 0);

    driveTrain.set(new ChassisSpeeds(0, 0, turnSpeed));
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
