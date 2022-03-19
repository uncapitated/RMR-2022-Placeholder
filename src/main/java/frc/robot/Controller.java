// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

        /** B Button */
        public static boolean getSlowButton() {
            return controller.getBButton();
        }

        /** Intake button is mapped to the left bumper on the second controller */
        public static JoystickButton getIntakeButton() {
            return new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
        }

        /** Dispense button is mapped to the right bumper on the second controller */
        public static JoystickButton getDispenseButton() {
            return new JoystickButton(controller, XboxController.Button.kRightBumper.value);
        }

        /** A Button */
        public static JoystickButton getAlignButton() {
            return new JoystickButton(controller, XboxController.Button.kA.value);
        }

        /** Right trigger */
        public static double getRightTriggerSpeed() {
            return controller.getRightTriggerAxis();
        }

        /** Left trigger */
        public static double getLeftTriggerSpeed() {
            return controller.getLeftTriggerAxis();
        }

        /** X button */
        public static boolean getControlledTurnButton() {
            return controller.getXButton();
        }

        /** Y button */
        public static boolean getNormalTurnButton() {
            return controller.getYButton();
        }

        // Turn modes
        public enum TurnModes {
            NORMAL,
            CONTROLLED
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

        /** Auto Climb for the first bar is set to the Left Joystick Button */
        public static JoystickButton getFirstBarAutoClimbButton() {
            return new JoystickButton(controller, XboxController.Button.kLeftStick.value);
        }

        /** Auto Climb for the first bar is set to the Right Joystick Button */
        public static JoystickButton getSecondBarAutoClimbButton() {
            return new JoystickButton(controller, XboxController.Button.kRightStick.value);
        }

        /** Down DPad */
        public static Trigger getWinchDownButton() {
            BooleanSupplier POVUp = () -> controller.getPOV() == 270;
            return new Trigger(POVUp);
        }
        
        /** Up DPad */
        public static Trigger getWinchUpButton() {
            BooleanSupplier POVUp = () -> controller.getPOV() == 90;
            return new Trigger(POVUp);
        }

        /** A Button */
        public static JoystickButton getLowPointButton() {
            return new JoystickButton(controller, XboxController.Button.kA.value);
        }

        /** X Button */
        public static JoystickButton getTransferPointButton() {
            return new JoystickButton(controller, XboxController.Button.kRightStick.value);
        }
        /** X Button */
        public static JoystickButton getElevatorUpButton() {
            return new JoystickButton(controller, XboxController.Button.kX.value);
        }

        /** Y Button */
        public static JoystickButton getElevatorDownButton() {
            return new JoystickButton(controller, XboxController.Button.kY.value);
        }

        /** Climber UP button is mapped to the left bumper */
        public static JoystickButton getClimberUpButton() {
            return new JoystickButton(controller, XboxController.Button.kX.value);
        }
        /** Left DPad */
        public static Trigger getClimberInButton() {
            return new JoystickButton(controller, XboxController.Button.kA.value);
        }

        /** Right DPad */
        public static Trigger getClimberOutButton() {
            return new JoystickButton(controller, XboxController.Button.kB.value);
        }
        
        /** Right bumper */
        public static JoystickButton getToggleButton() {
            return new JoystickButton(controller, XboxController.Button.kRightBumper.value);
        }

        public static JoystickButton getToggleActivationButton() {
            return new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
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