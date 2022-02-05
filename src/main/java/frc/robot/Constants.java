// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static final class Autonomous
    {
        public static final double MAX_SPEED_METERS_PER_SECOND = 0.5;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.5;
    }

    /**
     * Current channels on the Power distribution panel which can be used to enforce current restrictions
     */
    public static final class Current
    {
        // values not confirmed (currently doesn't matter)
        public static final int FRONT_RIGHT_DRIVE = 0;
        public static final int BACK_RIGHT_DRIVE = 1;
        public static final int FRONT_LEFT_DRIVE = 14;
        public static final int BACK_LEFT_DRIVE = 15;
    }

    /**
     *  motor CAN ID class
     */
    public static final class Drive
    {
        public static final int FRONT_RIGHT = 4;
        public static final int BACK_RIGHT = 5;
        public static final int FRONT_LEFT = 1;
        public static final int BACK_LEFT = 2;

        public static final double HIGH_GEAR_RATIO = 51/153;
        public static final double LOW_GEAR_RATIO = 51/231;
        public static final double WHEEL_RADIUS = 0.10000000001;
        
        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(0.6);
    }

    /*
    Constants for escalator stuff
    */
    public static final class Esc
    {
        public static final int deviceId = 0;
    }

    /**
     * Class with constant PCM channels for the shifter
     */
    public static final class Shifter
    {
        public static final int LOW = 4;
        public static final int HIGH = 5;
    }

    public static final class Motor
    {
        /** The steps per revolution of a TalonFX */
        public static final int DRIVE_SPR = 2048;
        /** Conversion factor to convert SPR to rotations per second */
        public static final double DRIVE_VELOCITY_FACTOR = 10 / DRIVE_SPR;
    }
}
