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

  /** Creates a new Limelight. */
  public Limelight() {
    NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");

    hasValidTargets = networkTable.getEntry("tv");
    horizontalOffsetAngle = networkTable.getEntry("tx");
    verticalOffsetAngle = networkTable.getEntry("ty");
    targetArea = networkTable.getEntry("ta");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
