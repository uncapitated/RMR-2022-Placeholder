// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/** Add your docs here. */
public class Controller
{
    public static class Drive
    {
        /**
         * Drive Controller Mapping
         * Left Joystick -> controls both speed and direction
         * 
         * Either Trigger -> controls slow mode (pressing down either bumber will slow the robot)
         * Currently cuts the power output in half
         */

        private static XboxController controller = new XboxController(0);

        /**
         * @return raw forwards value between -1.0 and 1.0
         * with 1.0 being full forwards and -1.0 being full reverse
         */
        public static double get_forward()
        {
            // can be from -1.0 to 1.0
            // Calling Math.max is unnecessary here - results in same thing
            // as just calling controller.getLeftTriggerAxis()
            // double trigger = Math.max(controller.getLeftTriggerAxis(), controller.getLeftTriggerAxis());
            double trigger = controller.getLeftTriggerAxis();

            // scales the trigger value to 1 -> sqrt(0.5) and -1 -> 1
            double multiplier = 1 - (1 - Math.sqrt(0.5)) * (1 + trigger) / 2;

            // controller should be inverted forward is negative
            return -controller.getLeftY() * multiplier;
        }

        /**
         * 
         * @return raw turn value between -1.0 and 1.0
         * with 1.0 being full right turn and -1.0 being full left turn
         */
        public static double get_turn()
        {
            // can be from -1.0 to 1.0
            double trigger = Math.max(controller.getLeftTriggerAxis(), controller.getRightTriggerAxis());

            // scales the trigger value to 1 -> sqrt(0.5) and -1 -> 1
            double multiplier = 1 - (1 - Math.sqrt(0.5)) * (1 + trigger) / 2;

            return controller.getLeftX() * multiplier;
        }

        public static double get_secondary_vertical_stick() {
            return controller.getRightY();
        }

        public static boolean get_a_button() {
            return controller.getAButton();
        }

        public static boolean get_b_button() {
            return controller.getBButton();
        }

        public static double get_right_stick_horizontal(){
            return controller.getRightX();
        }

        public static void setRumble(boolean hasRumble)
        {
            if (hasRumble)
            {
                controller.setRumble(RumbleType.kLeftRumble, 1.0);
                controller.setRumble(RumbleType.kRightRumble, 1.0);
            }
            else
            {
                controller.setRumble(RumbleType.kLeftRumble, 0.0);
                controller.setRumble(RumbleType.kRightRumble, 0.0);
            }
        }
    }
}
