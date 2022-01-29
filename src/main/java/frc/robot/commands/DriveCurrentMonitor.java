// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.Current;

public class DriveCurrentMonitor extends CommandBase {
  /**
   * Link to WPI for using power distribution class
   * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html
   * Link to API
   * https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/PowerDistribution.html
   * Object for controlling the power distribution panel
   */
  private PowerDistribution powerDistribution = new PowerDistribution();

  // set this to the max current at full throttle
  private final double maxCurrent = 20.0;

  /**
   * Link to WPILib for using Median Filters
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/median-filter.html
   * Link to API
   * https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/filter/MedianFilter.html
   * uses the median of the last 15 samples to filter the inputs
   */
  
  private MedianFilter leftCurrentFilter = new MedianFilter(15);
  private MedianFilter rightCurrentFilter = new MedianFilter(15);

  // the state of the DriveTrain
  private boolean isStalled;

  /** Creates a new DriveCurrentMonitor. */
  public DriveCurrentMonitor() {
    // Use addRequirements() here to declare subsystem dependencies.

    // Do not addRequirements(driveTrainSubsystem)
    // Do not need to add requirements because this is only reading values

    isStalled = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double frontRightCurrent = powerDistribution.getCurrent(Current.FRONT_RIGHT_DRIVE); // Current.FRONT_RIGHT_DRIVE is frome the constants file
    double backRightCurrent = powerDistribution.getCurrent(Current.BACK_RIGHT_DRIVE);
    double frontLeftCurrent = powerDistribution.getCurrent(Current.FRONT_LEFT_DRIVE);
    double backLeftCurrent = powerDistribution.getCurrent(Current.BACK_LEFT_DRIVE);

    // get averages of left and right currents
    double rightCurrent = (frontRightCurrent + backRightCurrent) / 2;
    double leftCurrent = (frontLeftCurrent + backLeftCurrent) / 2;

    // dampening for current spikes
    rightCurrent = rightCurrentFilter.calculate(rightCurrent);
    leftCurrent = leftCurrentFilter.calculate(leftCurrent);

    boolean isRightStalled = rightCurrent > maxCurrent;
    boolean isLeftStalled = leftCurrent > maxCurrent;

    if (isRightStalled || isLeftStalled)
    {
      isStalled = true;
    }
    else
    {
      isStalled = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isStalled() {
    return isStalled;
  }

  public PowerDistribution getPowerDistribution() {
    return powerDistribution;
  }
}
