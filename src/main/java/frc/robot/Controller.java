// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
         * Link to WPILib for using slew rate limiters
         * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
         * Link to API
         * https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/filter/SlewRateLimiter.html
         * defines the maximum change in forward and turn (acceleration)
         */
        private static SlewRateLimiter forwardRateLimiter = new SlewRateLimiter(3.0);
        private static SlewRateLimiter turnRateLimiter = new SlewRateLimiter(3.0);

        /**
         * @return raw forwards value between -1.0 and 1.0
         * with 1.0 being full forwards and -1.0 being full reverse
         */
        public static double get_forward()
        {
            // controller should be inverted because forward is negative
            return forwardRateLimiter.calculate(-controller.getLeftY());
        }

        /**
         * 
         * @return raw turn value between -1.0 and 1.0
         * with 1.0 being full right turn and -1.0 being full left turn
         */
        public static double get_turn()
        {
            return turnRateLimiter.calculate(controller.getLeftX());
        }

        // button to activate the align command
        public static JoystickButton getXAlignButton()
        {
            return new JoystickButton(controller, XboxController.Button.kX.value);
        }

        public static double getSecondaryVerticalStick(){
            return controller.getRightY();
        }

        public static boolean getAButton() {
            return controller.getAButton();
        }

        public static boolean getBButton() {
            return controller.getBButton();
        }

        public static boolean getXButton(){
            return controller.getXButton();
        }

        public static double getRightStickHorizontal(){
            return controller.getRightX();
        }

        public static void setRumble(boolean hasRumble)
        {
            /**
             * Link to API
             */
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
