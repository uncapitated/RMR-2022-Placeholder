// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

public class Limelight extends SubsystemBase {


  @Getter
  private NetworkTableEntry hasValidTargets;

  @Getter
  private NetworkTableEntry horizontalOffsetAngle;
  
  @Getter
  private NetworkTableEntry verticalOffsetAngle;

  @Getter
  private NetworkTableEntry targetArea;

  @Getter
  private NetworkTableEntry lightValue;

  /** Creates a new Limelight. */
  public Limelight() {
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");


    // Get information from the limelight, and store it
    hasValidTargets = limelight.getEntry("tv");
    horizontalOffsetAngle = limelight.getEntry("tx");
    verticalOffsetAngle = limelight.getEntry("ty");
    targetArea = limelight.getEntry("ta");
    lightValue = limelight.getEntry("ledMode");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
