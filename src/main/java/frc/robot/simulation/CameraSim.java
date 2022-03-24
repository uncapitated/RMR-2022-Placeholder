// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

/** Add your docs here. */
public class CameraSim {

    public class SimDetection
    {
        public int xmin;
        public int xmax;
        public int ymin;
        public int ymax;

        public String label;

        /**
         * creates a square detection
         * @param xPos
         * @param yPos
         * @param lengthSize
         */
        public SimDetection(int xPos, int yPos, int lengthSize){
            this(xPos, yPos, lengthSize, "default");
        }

        /**
         * creates a square detection with the center and side length
         * 
         * @param xPos the x-coordinate of the center of the detection
         * @param yPos the y-coordinate of the center of the detection
         * @param lengthSize the length/height of the detection
         * @param label the label of the detection
         */
        public SimDetection(int xPos, int yPos, int lengthSize, String label) {
            xmin = xPos - lengthSize/2;
            xmax = xPos + lengthSize/2;
            xmin = xPos - lengthSize/2;
            xmin = xPos + lengthSize/2;

            this.label = label;
        }
    }
}
