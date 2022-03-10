// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    /**
     * Contains Autonomous Planning constants
     */
    public static final class Autonomous {
        public static final double MAX_SPEED_METERS_PER_SECOND = 0.5;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.5;

        public static final Auto[] AUTONOMOUS = 
        {
            // Auto 1
            /** Simple Auto */
            new Auto(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0.3, 0, new Rotation2d(0)), new Pose2d(-1.5, 0, new Rotation2d(0)))
        };

    }

    public static final class StatusSwitch {
        public static final int FIRST_INPUT = 0;
        public static final int SECOND_INPUT = 1;
        public static final int THIRD_INPUT = 2;
        public static final int FOURTH_INPUT = 3;
    }

    /**
     *  motor CAN ID class
     */
    public static final class Drive {
        // drive max accel meters per second
        public static final double DRIVE_MAX_ACCEL = 5.0;
        public static final double DRIVE_MAX_ANGLE_ACCEL = 15.0;

        /** CAN ID */
        public static final int FRONT_RIGHT = 9;
        /** CAN ID */
        public static final int BACK_RIGHT = 8;
        /** CAN ID */
        public static final int FRONT_LEFT = 6;
        /** CAN ID */
        public static final int BACK_LEFT = 7;

        /** solenoid Pneumatic Hub port */
        public static final int SHIFTER_HIGH = 5;
        /** solenoid Pneumatic Hub port */
        public static final int SHIFTER_LOW = 4;

        public static final double HIGH_GEAR_RATIO = 12.0/42.0 * 24.0/50.0;
        public static final double LOW_GEAR_RATIO = 12.0/42.0 * 14.0/60.0;
        public static final double WHEEL_RADIUS = 0.122237;
        
        /** converts chassis speeds to wheel speeds */
        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(0.6);

        /** The steps per revolution of a TalonFX */
        public static final int DRIVE_SPR = 2048;
        /** Conversion factor to convert SPR to rotations per second */
        public static final double DRIVE_VELOCITY_FACTOR = 10.0 / DRIVE_SPR;

    }
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
         * 	                                    			   kP    kI  kD        kF         Iz    PeakOut */
        public final static Gains kGains_Velocity  = new Gains(.1, .001, 5,  1023.0/20660.0,  300,  1.00);
    }


    /**
     *Constants for winch stuff
     */
    public static final class Climber {
        /** CAN ID */
        public static final int WINCH_MOTOR = 12;

        /** solenoid Pneumatic Hub port */
        public static final int SOLENOID_IN = 3;
        /** solenoid Pneumatic Hub port */
        public static final int SOLENOID_OUT = 2;


        /** Winch starting position in meters */
        public static final double STARTING_POSITION = 0.10;

        /** Gear ratio between the winch and the cable (m/rev) */
        public static final double WINCH_RATIO = 1/48 * (0.027 * Math.PI);

        // height restraints 
        /** lowest the winch can go when it is angled (m) */
        public static final double MIN_ANGLED = 0.10;
        /** highest the winch can go when it is angled (m) */
        public static final double MAX_ANGLED = 0.70;
        /** lowest the winch can go when it is up (m) */
        public static final double MIN_UP = 0.00;
        /** highest the winch can go when it is up (m) */
        public static final double MAX_UP = 0.60;

        /**
         * PID Gains for the Elevator
         * 
         * PID Gains may have to be adjusted based on the responsiveness of control loop.
         * 
         * 
         * 	                                    			  kP   	kI   kD kF Iz PeakOut */
        public final static Gains kGains_Position = new Gains(0.1, 1e-4, 1, 0, 0, 1);
    }

    /**
     *Constants for escalator stuff
     */
    public static final class Belt {
        /** CAN ID */
        public static final int TOP_MOTOR_ID = 10;
        /** CAN ID */
        public static final int BOTTOM_MOTOR_ID = 11;
        
        /** solenoid Pneumatic Hub port */
        public static final int SOLENOID_IN = 1;
        /** solenoid Pneumatic Hub port */
        public static final int SOLENOID_OUT = 0;
    }

    public static final class CompressorConstants {
        /** Digital Input Port */
        public static final int COMPRESSOR_SWITCH = 4;
    }

    public static final class CameraConstants
    {
        //Constants for the camera
        public static final double maxX = 1280;
        public static final double maxY = 720;

        public static final String[] label = {"Re", "Blu"};
    }

    public static final class CameraPIDConstants{

        //Angular constants; tune for robot
        public static final double akP = .0001;
        public static final double akI = .00000;
        public static final double akD = 0.00;
    }

    public static final class Pneumatics {
        public static final int COMPRESSOR_CAN_ID = 2;
    }
}
