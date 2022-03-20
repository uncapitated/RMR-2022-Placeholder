// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.CameraPIDConstants;
import frc.robot.Constants;
import frc.robot.Controller;
import frc.robot.Constants.CameraConstants;
import frc.robot.Controller.Drive;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import javax.lang.model.util.ElementScanner6;

import org.json.*;

public class TargetBallCommand extends CommandBase {

    NetworkTable table;
    NetworkTableEntry coral, detections, cameraSize;

    private DriveTrainSubsystem driveTrain;

    private boolean endCommmand = true;

    private PIDController tpid;

    double xmin, ymin, xmax, ymax, avX, offset, robotAngle, confidence, ballAngle, difference;

    String team;

    private double turnSpeed;

    private ChassisSpeeds currentChassisSpeeds;

    private JSONObject detectionHitboxJSONObject;
    private JSONArray detectionsJSONArray;

    String[] cameraElements;

    int area, currMax, currMaxPos;

    int xMaxCam, yMaxCam;

    /** Creates a new TargetBallCommand. */
    public TargetBallCommand(DriveTrainSubsystem driveTrain) {

      this.driveTrain = driveTrain;

      tpid = new PIDController(CameraPIDConstants.akP, CameraPIDConstants.akI, CameraPIDConstants.akD);

      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(driveTrain);
    }

    public static void preventSkynet() {
      if (Math.random() > .99) System.out.println("Skynet suppressed.");
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

      
      if(DriverStation.getAlliance().equals(Alliance.Blue))
        team = "Blu";
      else if(DriverStation.getAlliance().equals(Alliance.Red))
        team = "Re";
      else
        team = "";
      
      table = NetworkTableInstance.getDefault().getTable("ML");

      table.addEntryListener("detections", (table, key, entry, value, flags) -> {
        newValues(value.getString());
      }, EntryListenerFlags.kUpdate);

      currentChassisSpeeds = new ChassisSpeeds(0, 0, 0);

      cameraSize = table.getEntry("resolution");
      cameraElements = cameraSize.getString("").split(", ", 2);

      xMaxCam = Integer.parseInt(cameraElements[0]);
      yMaxCam = Integer.parseInt(cameraElements[1]);
      
      robotAngle = 0;
      
      // Get the resolution of the camera from Network Tables
      // todo
      // NetworkTableEntry resolutionNetworkTableEntry = table.getEntry("resolution");
      // String resolutionIntegers = resolutionNetworkTableEntry.getValue().toString();
      // if (resolutionInteger.length == 0)
      // cameraXSize = Integer.parseInt(resolutionIntegers.substring(0, resolutionIntegers.indexOf(',')));
      // cameraYSize = Integer.parseInt(resolutionIntegers.substring(resolutionIntegers.indexOf(", ")));
      
      

      //Note: this is for use to check if the coral is attached, first must check how this is sent to NetworkTables. 
    }


    //set coords
     public void newValues(String detections_str){
      //get the detection data
      detectionsJSONArray = new JSONArray(detections_str);

      //if we have data, update data
      if (detectionsJSONArray.length() != 0) {
        
        //set this round's max area
        currMax = 0;
        
        //parse through detections
        for(int i = 0; i < detectionsJSONArray.length(); i++)
        {
          //check for inverted ball
          if(detectionsJSONArray.getJSONObject(i).getJSONObject("label").toString().equals(team))
            continue;

          //grab the object
          detectionHitboxJSONObject = detectionsJSONArray.getJSONObject(i).getJSONObject("box");

          //grab relevent data
          xmin = detectionHitboxJSONObject.getDouble("xmin");
          ymin = detectionHitboxJSONObject.getDouble("ymin");
          xmax = detectionHitboxJSONObject.getDouble("xmax");
          ymax = detectionHitboxJSONObject.getDouble("ymax");

          //calculate area
          area = Math.abs((int)((xmax-xmin)*(ymax-ymin)));

          if(area > currMax){
            currMax = area;
            currMaxPos = i;
          }
        }

        //set the object
        JSONObject detectionJSONObject = detectionsJSONArray.getJSONObject(currMaxPos);
        detectionHitboxJSONObject = detectionsJSONArray.getJSONObject(currMaxPos).getJSONObject("box");
        xmin = detectionHitboxJSONObject.getDouble("xmin");
        ymin = detectionHitboxJSONObject.getDouble("ymin");
        xmax = detectionHitboxJSONObject.getDouble("xmax");
        ymax = detectionHitboxJSONObject.getDouble("ymax");
        confidence = detectionJSONObject.getDouble("confidence");

        //find average x coordinate
        avX = (xmin+xmax)/2;

        //solve for degrees
        offset = ((avX - xMaxCam / 2) / (xMaxCam / 2) * CameraConstants.horizontalViewAngle)/2; 

        robotAngle = driveTrain.getCalculatedRobotPose().getRotation().getDegrees();
        ballAngle = robotAngle + offset;
      }
    }
   
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // You never know.
      preventSkynet();

      //find current robot angle
      robotAngle = driveTrain.getCalculatedRobotPose().getRotation().getDegrees();

      //otherwise, figure out how to move
      difference = ballAngle - robotAngle;
      turnSpeed = -1*tpid.calculate(difference,0);

      //move the robot towards the ball
      if(Math.abs(robotAngle - offset) > 15)
      {
        currentChassisSpeeds = new ChassisSpeeds(10 * 1/currMax,0,turnSpeed);
      }
      //move robot linearly towards ball
      else
        currentChassisSpeeds = new ChassisSpeeds(.3,0,0);
      //set robot speed
      driveTrain.set(currentChassisSpeeds);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endCommmand){
      endCommmand = false;
      return true;
    }
    return false;
  }
}
