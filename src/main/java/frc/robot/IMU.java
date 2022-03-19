// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */

//Currently not functional; check back later
public class IMU {
    // public static final ADIS16448_IMU imu = new ADIS16448_IMU();

    //calibration constant
    private static final double kVoltsPerDegreePerSecond = 0.01;
    
    public static void updateAngles()
    {
        // SmartDashboard.putNumber("x-rotation", imu.getGyroAngleX());
        // SmartDashboard.putNumber("y-rotation", imu.getGyroAngleY());
        // SmartDashboard.putNumber("z-rotation", imu.getGyroAngleZ());
    }
}
