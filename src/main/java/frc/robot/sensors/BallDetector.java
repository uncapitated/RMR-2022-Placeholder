// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.util.ArrayList;
import java.util.List;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import lombok.Getter;

/** Add your docs here. */
public class BallDetector {
    /** represents a detected ball */
    public class BallDetection
    {
        @Getter
        private double lastUpdateTime;

        @Getter
        private double ballSize;

        @Getter
        private Pose2d ballPosition;

        public BallDetection(Pose2d position, double size) {
            lastUpdateTime = Timer.getFPGATimestamp();

            ballPosition = position;
            ballSize = size;
        }

        /**
         * This function can be used to manually use ball positions in calculations
         * 
         * @param robotPosition the position of the robot
         * @return the relative position of the ball.
         * positive x is forward.
         * positive y is to the left.
         * positive angle is counterclockwise.
         */
        public Pose2d getRelativeBallPosition(Pose2d robotPosition) {
            return ballPosition.relativeTo(robotPosition);
        }

        /**
         * Use this function to control drive base to target a ball
         * 
         * @param robotPosition the position of the robot
         * @return the angle (in radians) the ball should make with the front of the robot. positive is counterclockwise.
         */
        public double getRelativeAngleOffset(Pose2d robotPosition) {
            Pose2d relativePosition = getRelativeBallPosition(robotPosition);
            return Math.atan2(relativePosition.getY(), relativePosition.getX());
        }

        /**
         * Use this function to control drive base to targe a ball
         * 
         * @param robotPosition  the position of the robot
         * @return the distance to the ball (in meters)
         */
        public double getRelativeDistance(Pose2d robotPosition) {
            Pose2d relativePosition = getRelativeBallPosition(robotPosition);
            return Math.hypot(relativePosition.getX(), relativePosition.getY());
        }

        public void setBallPosition(Pose2d position) {
            ballPosition = position;

            lastUpdateTime = Timer.getFPGATimestamp();
        }

        public void setBallSize(double size) {
            ballSize = size;

            lastUpdateTime = Timer.getFPGATimestamp();
        }
    }

    private NetworkTable axonTable;
    private NetworkTableEntry detectionEntry;

    private int cameraWidth;
    private int cameraHeight;

    private DriveTrainSubsystem driveTrain;

    private List<BallDetection> detections;
    private BallDetection bestDetection;

    public BallDetector(DriveTrainSubsystem driveTrainSubsystem) {
        driveTrain = driveTrainSubsystem;

        // used to communicate with axon
        axonTable = NetworkTableInstance.getDefault().getTable("ML");
        detectionEntry = axonTable.getEntry("detections");

        detections = new ArrayList<BallDetection>();

        bestDetection = null;

        // updates the values
        detectionEntry.addListener(entry -> {
            updateDetectionsFromCamera(entry.value.getString());
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);

        cameraWidth = getCameraWidth();
        cameraHeight = getCameraHeight();
    }

    private void updateDetectionsFromCamera(String JSONString) {
        cameraWidth = getCameraWidth();
        cameraHeight = getCameraHeight();

        JSONArray detectionArray = new JSONArray(JSONString);

        List<BallDetection> newDetections = new ArrayList<BallDetection>();

        // loop through each detection
        for (int i = 0; i < detectionArray.length(); i++) {
            JSONObject detectionObject = detectionArray.getJSONObject(i);
            JSONObject detectionBoundingBox = detectionObject.getJSONObject("box");

            double ballCameraX = (detectionBoundingBox.getDouble("xmax") + detectionBoundingBox.getDouble("xmin")) / 2;
            double ballCameraY = (detectionBoundingBox.getDouble("ymax") + detectionBoundingBox.getDouble("ymin")) / 2;
            double ballCameraSize = ((detectionBoundingBox.getDouble("xmax") - detectionBoundingBox.getDouble("xmin")) + (detectionBoundingBox.getDouble("ymax") + detectionBoundingBox.getDouble("ymin"))) / 2;

            // calculates the angle offset from the middle in radians (left is positive, right is negative)
            double ballAngle = (1 / 2 - ballCameraX / CameraConstants.width) * Math.toRadians(CameraConstants.horizontalViewAngle);
            // really bad estimate of the distance the ball is from the camera
            double ballDistance = (Constants.GamePieces.BALL_DIAMETER / 2) / Math.tan(Math.toRadians(ballCameraSize * CameraConstants.degreesPerPixel / 2));

            Transform2d ballTransform = new Transform2d(new Translation2d(ballDistance, new Rotation2d(ballAngle)), new Rotation2d(0));

            // use the camera offset to adjust the ball position
            ballTransform = ballTransform.plus(CameraConstants.cameraOffset.inverse());

            Pose2d ballPosition = driveTrain.getCalculatedRobotPose().plus(ballTransform);

            // create a ball detection object and compare it to the current list of detections
            BallDetection ball = new BallDetection(ballPosition, ballCameraSize);

            newDetections.add(ball);
        }

        List<BallDetection> oldDetections = new ArrayList<BallDetection>();
        oldDetections.addAll(detections);

        // compare the list of current detections with the new detections
        for (int i = 0; i < detections.size() && newDetections.size() > 0; i++) {

            // comparing only angles
            double ballAngleToCheck = detections.get(i).getRelativeAngleOffset(driveTrain.getCalculatedRobotPose());

            BallDetection bestNewDetection = newDetections.get(0);
            double bestNewAngleDifference = Math.abs(bestNewDetection.getRelativeAngleOffset(driveTrain.getCalculatedRobotPose()) - ballAngleToCheck);
            for (BallDetection newDetection : newDetections) {
                double newBallAngle = newDetection.getRelativeAngleOffset(driveTrain.getCalculatedRobotPose());

                if (Math.abs(newBallAngle - ballAngleToCheck) < bestNewAngleDifference) {
                    bestNewDetection = newDetection;
                    bestNewAngleDifference = Math.abs(newBallAngle - ballAngleToCheck);
                }
            }

            if (bestNewAngleDifference < CameraConstants.minAngleChange) {
                detections.get(0).setBallPosition(bestNewDetection.getBallPosition());
                detections.get(0).setBallSize(bestNewDetection.getBallSize());

                newDetections.remove(bestNewDetection);
                oldDetections.remove(detections.get(i));
            }
        }

        for(BallDetection oldDetection : oldDetections) {
            // if the detection has not been updated for more than 1.5 seconds
            if (oldDetection.getLastUpdateTime() > 1.5) {
                detections.remove(oldDetection);

                // clear references to this detection
                oldDetection = null;
            }
        }

        // add all of the new detections
        detections.addAll(newDetections);

        if (detections.size() > 0) {

            // sort them to find the best
            detections.sort((a, b) -> {
                double aScore = a.getRelativeAngleOffset(driveTrain.getCalculatedRobotPose()) / a.getRelativeDistance(driveTrain.getCalculatedRobotPose());
                double bScore = b.getRelativeAngleOffset(driveTrain.getCalculatedRobotPose()) / b.getRelativeDistance(driveTrain.getCalculatedRobotPose());
                return aScore < bScore ? -1 : 1;
            });

            bestDetection = detections.get(0);
        }
    }

    private int getCameraWidth() {
        return Integer.valueOf(axonTable.getEntry("resolution").getString(CameraConstants.width + ", " + CameraConstants.height).split(", ", 2)[0]);
    }
    private int getCameraHeight() {
        return Integer.valueOf(axonTable.getEntry("resolution").getString(CameraConstants.width + ", " + CameraConstants.height).split(", ", 2)[1]);
    }

    /**
     * Interface with the ball detector and get the best detection.
     * @return The current best detection from the camera note that this value could no longer be visible from the camera and will need to be updated constantly.
     */
    public BallDetection getBestDetection() {
        return bestDetection;
    }
}
