// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

public class LemonLamp extends SubsystemBase {

  @Getter
  private NetworkTableEntry hasValidTargets;

  @Getter
  private NetworkTableEntry horizontalOffsetAngle;
  
  /** Creates a new LemonLamp. */
  public LemonLamp() {
    NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

    hasValidTargets = networkTableInstance.getEntry("tv");
    hasValidTargets = networkTableInstance.getEntry("tx");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
