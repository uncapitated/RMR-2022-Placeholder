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
    public static final class DrivePID {
        /**
         * Which PID slot to pull gains from. Starting 2018, you can choose from
         * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
         * configuration.
         */
        public static final int kSlotIdx = 0;

        /**
         * Talon FX supports multiple (cascaded) PID loops. For
         * now we just want the primary one.
         */
        public static final int kPIDLoopIdx = 0;

        /**
         * Set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int kTimeoutMs = 30;

        /**
         * PID Gains may have to be adjusted based on the responsiveness of control loop.
         * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
         * 
         * 	                                    			  kP   	 kI    kD      kF          Iz    PeakOut */

        //Angular constants; tune for robot
        public static final double akP = .1;
        public static final double akI = .000;
        public static final int akD = 5;

        //Constants for the camera
        public static final double maxX = 100;
        public static final double maxY = 100;

         public final static Gains kGains_Velocity  = new Gains( .1, .000, 5, 1023.0/20660.0,  300,  1.00);
    }
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
        public static final int FRONT_RIGHT_DRIVE = 19;
        public static final int BACK_RIGHT_DRIVE = 18;
        public static final int FRONT_LEFT_DRIVE = 16;
        public static final int BACK_LEFT_DRIVE = 17;
    }

    /**
     *  motor CAN ID class
     */
    public static final class Drive
    {
        public static final int FRONT_RIGHT = 9;
        public static final int BACK_RIGHT = 8;
        public static final int FRONT_LEFT = 6;
        public static final int BACK_LEFT = 7;

        public static final int SHIFTER_HIGH = 0;
        public static final int SHIFTER_LOW = 0;

        public static final double HIGH_GEAR_RATIO = 51.0/153;
        public static final double LOW_GEAR_RATIO = 51.0/231;
        public static final double WHEEL_RADIUS = 0.10000000001;
        
        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(0.6);
    }


    /*
    Constants for winch stuff
    */
    public static final class Climber 
    {
        public static final int WINCH_MOTOR = 47;

        public static final int SOLENOID_IN = 2;
        public static final int SOLENOID_OUT = 3;

        // height restraints

    }

    /*
    Constants for escalator stuff
    */
    public static final class Belt
    {
        public static final int TOP_MOTOR_ID = 0;
        public static final int BOTTOM_MOTOR_ID = 0;
        
        public static final int SOLENOID_IN = 2;
        public static final int SOLENOID_OUT = 3;
    }

    public static final class Motor
    {
        /** The steps per revolution of a TalonFX */
        public static final int DRIVE_SPR = 2048;
        /** Conversion factor to convert SPR to rotations per second */
        public static final double DRIVE_VELOCITY_FACTOR = 10.0 / DRIVE_SPR;
    }
}
