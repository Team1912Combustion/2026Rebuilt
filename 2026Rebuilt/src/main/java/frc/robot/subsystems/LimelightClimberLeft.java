// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class LimelightClimberLeft extends SubsystemBase {
  private final NetworkTable table =
	  NetworkTableInstance.getDefault().getTable(getName());
  private final NetworkTableEntry pipeline = table.getEntry("pipeline");
  private final NetworkTableEntry xOffset = table.getEntry("tx");
  private final NetworkTableEntry yOffset = table.getEntry("ty");
  private final NetworkTableEntry latency = table.getEntry("tl");
  private final NetworkTableEntry botPoseTargetSpace = table.getEntry("botpose_targetspace");
  private final NetworkTableEntry botPose = table.getEntry("botpose_wpiblue");
  private final NetworkTableEntry tagId = table.getEntry("tid");
  private final NetworkTableEntry targetArea = table.getEntry("ta");
  private final NetworkTableEntry json = table.getEntry("json");
  private double[] lastBotPose = new double[6];
  private int currentPipeline = 0;
  private int currentTagId = 0;
  /** Creates a new LimelightFrontLeft. */
  public LimelightClimberLeft() {}

  @Override
  public void periodic() {
    currentTagId = getTagId();
    lastBotPose = getBotPose();
    if(currentTagId > 0) {
     // lastPose2d = new Pose2d(lastBotPose[0],lastBotPose[1], new Rotation2d(lastBotPose[5]));
      SmartDashboard.putNumber(getPosition()+"Botpose X: ",lastBotPose[0]);
      SmartDashboard.putNumber(getPosition()+"Botpose Y: ",lastBotPose[1]);

      SmartDashboard.putNumber(getPosition()+"Botpose Yaw: ",lastBotPose[5]);

    }       
    SmartDashboard.putNumber(getPosition()+"Latency: ",(double) latency.getNumber(0));
    SmartDashboard.putNumber(getPosition()+"TagID: ",tagId.getInteger(-1));
    SmartDashboard.putBoolean(getPosition()+"HasBotPose: ",(currentTagId>0));
    //SmartDashboard.putNumber(getPosition()+"Limelight Xoffset: ",getXOffset());
    //SmartDashboard.putNumber(getPosition()+"Limelight Yoffset: ",getYOffset());

    // This method will be called once per scheduler run
  }
  // IP IS - 10.19.12.11:5801 //

  /**
   * Gets the name of the limelight.
   * @return The name of the limelight as a string
   */
  public String getName() {
    return "limelight-cl";
  }

  /**
   * Gets the position of the limelight.
   * @return The position of the limelight as a string
   */
  public String getPosition() {
    return "climberLeft";
  }
  /**
   * Toggles between pipeline 0 and 1.
   */
  public void togglePipeline() {
    if (currentPipeline == 0)
      currentPipeline = 1;
    else if (currentPipeline ==1)
      currentPipeline = 0;
  }

  /**
   * Returns the Apriltags estimated pose or null if there is no pose.
   * @return The limelight's outputted pose
   */
  public Pose2d getBotPose2d() {
    return new Pose2d(getBotPose()[0],getBotPose()[1], new Rotation2d(Math.toRadians(getBotPose()[5])));
  }
  /**
   * Returns the Apriltags estimated pose in target space or null if there is no pose.
   * @return The limleight's ouputted target space pose
   */
  public Pose2d getBotPose2dTargetSpace() {
    return new Pose2d(getBotPoseTargetSpace()[2], getBotPoseTargetSpace()[0], new Rotation2d(Math.toRadians(getBotPoseTargetSpace()[4])));
  }

  /**
   * Returns the latency of the pipeline 
   * @return
   */
  public double getLatency() {
    return latency.getDouble(0);
  }

  /**
   * Returns null if we do not see any apriltags.
   * https://docs.limelightvision.io/en/latest/networktables_api.html
   * @return
   */
  public double[] getBotPose() {
    return botPose.getDoubleArray(new double[7]);
  }
  /**
   * Gets the full array of bot pose values in target space
   * @return An array of all pose values
   */
  public double[] getBotPoseTargetSpace() {
    return botPoseTargetSpace.getDoubleArray(new double[7]);
  }
  /**
   * Gets the ID of the biggest tag in frame.
   * @return The ID of the tag
   */
  public int getTagId() {
    return (int)tagId.getInteger(0);
  }
  /**
   * Gets the number of tags that the limelight can see.
   * @return The number of tags
   */
  public int getTagCount() {
    return countSubstringOccurrences(json.getString(""), "pts");
  }
  /**
   * Gets the number of times a string appears in a larger string.
   * @param mainString The string to search
   * @param subString The string to search for
   * @return The number of times the string occurred 
   */
  public int countSubstringOccurrences(String mainString, String subString) {
    int count = 0;
    int lastIndex = 0;
    while (lastIndex != -1) {
        lastIndex = mainString.indexOf(subString, lastIndex);
        if (lastIndex != -1) {
            count++;
            lastIndex += subString.length(); // Move past the found occurrence
        }
    }
    return count;
  }
  public boolean acceptPose() {
    if (getTagId() == -1) {
      return false;
    } else if (Math.abs(getBotPose()[3]) > 1) {
      return false;
    } else if (getBotPose2d().getY() < 0) {
      return false;
    } else if (getBotPose2d().getX() < 0) {
      return false;
    } else if (getBotPose2d().getY() > VisionConstants.aprilTagLayout.getFieldWidth()) {
      return false;
    } else if (getBotPose2d().getX() > VisionConstants.aprilTagLayout.getFieldLength()) {
      return false;
    } else {
      return true;
    }
  }
  /**
   * Gets the X Offset of the tag from the center of the frame.
   * @return The X Offset
   */
  public double getXOffset() {
    if (tagId.getInteger(-1) == 7) {
      return xOffset.getDouble(0);
    } else {
      return 0;
    }
  }
  /**
   * Gets the Y Offset of the tag from the center of frame.
   * @return The Y Offset
   */
  public double getYOffset() {
    return yOffset.getDouble(0);
  }
  /**
   * Gets the area of the tag in frame
   * @return The area of the tag
   */
  public double getTargetArea() {
    return targetArea.getDouble(0);
  }
  /**
   * Turns the limelight LEDs on
   */
  public void setLedsOn() {
    NetworkTableInstance.getDefault().getTable(getName())
      .getEntry("ledMode").setNumber(3);
  }
  /**
   * Sets the limelight to a specific pipeline.
   * @param id The ID of the desired pipeline
   */
  public void setPipeline(int id) {
    pipeline.setNumber(id);
  }

  public int getPipeline() {
    return pipeline.getNumber(0).intValue();
  }

}