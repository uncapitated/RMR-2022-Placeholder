// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import lombok.Getter;

/** Add your docs here. */
public class ClimberSensorCollection {
    // limit switches
    @Getter
    LimitSwitchSensor topLimitSwitchSensor;
    @Getter
    LimitSwitchSensor bottomLimitSwitchSensor;

    // distance sensor
    @Getter
    DistanceSensor distanceSensor;

    public ClimberSensorCollection()
    {
        topLimitSwitchSensor = new LimitSwitchSensor(Constants.Climber.TOP_LIMIT_SWITCH_DIO_PORT);
        bottomLimitSwitchSensor = new LimitSwitchSensor(Constants.Climber.BOTTOM_LIMIT_SWITCH_DIO_PORT);

        if (RobotBase.isReal())
        {
            distanceSensor = new DistanceSensor();
        }
    }
}
