// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.Autonomous;

// contains path to follow

/** Add your docs here. */
public class Auto {
    // setup trajectory
    private final TrajectoryConfig config = new TrajectoryConfig(Autonomous.MAX_SPEED_METERS_PER_SECOND, 
      Autonomous.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED).setKinematics(Constants.Drive.KINEMATICS);

    private Pose2d startingPosition;
    
    private List<Pose2d> positions;

    public Auto(Pose2d startPos, Pose2d... nextPositions)
    {
        startingPosition = startPos;

        positions.add(startingPosition);
        positions.addAll(Arrays.asList(nextPositions));
    }

    public Trajectory getTragectory(int index)
    {
        // invalid index
        if (index >= positions.size() - 1)
        {
            throw(new IndexOutOfBoundsException("trajectory index out of bounds"));
        }
        return TrajectoryGenerator.generateTrajectory(List.of(positions.get(index), positions.get(index + 1)), config);
    }
}
