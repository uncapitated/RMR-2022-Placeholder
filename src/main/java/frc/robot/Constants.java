// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    }
    public static final class Grabber
    {
        public static final int LEFT = 3;
        public static final int RIGHT = 6;
        public static final int PIVOT = 7;
        public static final int ELEVATOR = 8;
    }

    //solenoid PCM (Pneumatic Control Module) ID Classes

    /**
     * Class with constant PCM channels for controlling the beak
     */
    public static final class Beak
    {
        public static final int IN = 3;
        public static final int OUT = 2;
    }

    /**
     * Class with constant PCM channels for the shifter
     */
    public static final class Shifter
    {
        public static final int LOW = 4;
        public static final int HIGH = 5;
    }
    /**
     * Class with constant PCM channels for miscellaneous solenoids
     */
    public static final class Solenoid
    {
        public static final int LIFTER = 6;
        public static final int TRACK = 0;
    }


    /**
     * Class with constant DI/O channels for limit switches
     */
    public static final class LimitSwitches
    {
        public static final int TOP = 3;
        public static final int BOTTOM = 2;
        public static final int BACK = 8;
        public static final int TOP_ARM = 1;
    }

}
