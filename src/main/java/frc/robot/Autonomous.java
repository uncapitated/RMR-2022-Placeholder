// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;

/** Add your docs here. */
public class Autonomous {
    private static Autonomous currentAutonomous;
    
    @Getter
    private Pose2d startingPos;
    @Getter
    private Rotation2d startingRotation;

    public static Autonomous getAutonomous() {
        return currentAutonomous;
    }

    public Autonomous() {
        startingRotation = new Rotation2d(0);
        startingPos = new Pose2d(0, 0, startingRotation);
        currentAutonomous = this;
    }
}
