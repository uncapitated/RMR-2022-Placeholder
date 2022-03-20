// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/** Add your docs here. */
public class Simulation {
    private Field2d field = new Field2d();

    // Shuffleboard Simulation Tab
    private ShuffleboardTab simulationTab = Shuffleboard.getTab("Simulation");
    private ComplexWidget fieldComponent = simulationTab.add("Field", field).withWidget(BuiltInWidgets.kField)
        .withPosition(1, 0).withSize(6, 3);

    public void updateRobotPos(Pose2d robotPose)
    {
        field.setRobotPose(robotPose);
    }
}
