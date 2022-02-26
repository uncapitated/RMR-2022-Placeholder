// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Add your docs here. */
public class Controller
{
    /**
     * 
     * @param value the value to apply the dead band
     * @param deadBand the amount of dead band to add around the value
     * @return the value with dead band applied
     */
    private static double addDeadBand(double value, double deadBand){
        // return Math.abs(value) > deadBand ? value : 0;
        return Math.copySign(Math.max(Math.abs(value) - deadBand, 0) / (1 - deadBand), value);
    }


    public static class Drive
    {
        /**
         * Drive Controller Mapping
         * Left Joystick -> controls both speed and direction
         */

        private static XboxController controller = new XboxController(0);

        /**
         * Link to WPILib for using slew rate limiters
         * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
         * Link to API
         * https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/filter/SlewRateLimiter.html
         * defines the maximum change in forward and turn (acceleration)
         */
        //private static SlewRateLimiter forwardRateLimiter = new SlewRateLimiter(3.0);
        //private static SlewRateLimiter turnRateLimiter = new SlewRateLimiter(3.0);

        /**
         * @return raw forwards value between -1.0 and 1.0
         * with 1.0 being full forwards and -1.0 being full reverse
         */
        public static double get_forward()
        {
            // controller should be inverted because forward is negative
            return addDeadBand(-controller.getLeftY(), 0.1);
        }

        /**
         * 
         * @return raw turn value between -1.0 and 1.0
         * with 1.0 being full right turn and -1.0 being full left turn
         */
        public static double get_turn()
        {
            return addDeadBand(controller.getLeftX(), 0.1);
        }

        public static boolean getSlowButton() {
            return controller.getBButton();
        }

        public static JoystickButton getTriggerLeft()  {
            return new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
        }

        public static JoystickButton getTriggerRight()  {
            return new JoystickButton(controller, XboxController.Button.kRightBumper.value);
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

    public static class Manipulator
    {
        // manipulator controller
        private static XboxController controller = new XboxController(1);

        /** Intake button is mapped to the A Button on the second controller */
        public static JoystickButton getIntakeButton() {
            return new JoystickButton(controller, XboxController.Button.kA.value);
        }

        /** Dispense button is mapped to the B Button on the second controller */
        public static JoystickButton getDispenseButton() {
            return new JoystickButton(controller, XboxController.Button.kB.value);
        }

        public static JoystickButton getWinchDownButton() {
            return new JoystickButton(controller, XboxController.Button.kX.value);
        }

        public static JoystickButton getWinchUpButton() {
            return new JoystickButton(controller, XboxController.Button.kY.value);
        }

        /** Climber Angle button is mapped to the left bumper */
        public static JoystickButton getClimberAngleButton() {
            return new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
        }

        /** Climber UP button is mapped to the left bumper */
        public static JoystickButton getClimberUpButton() {
            return new JoystickButton(controller, XboxController.Button.kRightBumper.value);
        }

        public static JoystickButton getTargetBallButton() {
            return new JoystickButton(controller, XboxController.Button.kLeftStick.value);
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
