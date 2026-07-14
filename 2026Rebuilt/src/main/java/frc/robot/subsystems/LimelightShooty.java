// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;

public class LimelightShooty extends SubsystemBase {
  private final NetworkTable table;
  private final NetworkTableEntry tagId;
  private double[] lastBotPose = new double[6];
  private int currentPipeline = 0;
  private int currentTagId = 0;

  /** Creates a new LimelightShooter. */
  public LimelightShooty() {
    setPipeline(0);
	  table = NetworkTableInstance.getDefault().getTable(getName());
    tagId = table.getEntry("tid")
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentTagId = getTagId();
    lastBotPose = getBotPoseMT1();
    SmartDashboard.putNumber(getPosition()+"Latency: ",(double) latency.getNumber(0));
    SmartDashboard.putNumber(getPosition()+"TagID: ",tagId.getInteger(-1));
    SmartDashboard.putBoolean(getPosition()+"HasBotPose: ",(currentTagId>0));
  }
  // IP IS - 10.19.12.13:5801 //

  /**
   * Gets the name of the limelight.
   * @return The name of the limelight as a string
   */
  public String getName() {
    return "limelight-s";
  }

  /**
   * Gets the position of the limelight.
   * @return The position of the limelight as a string
   */
  public String getPosition() {
    return "shooter";
  }

  /**
   * Toggles between pipeline 0 and 1.
   */
  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(getName(), pipeline);
  }

  /**
   * Returns the Apriltags estimated pose or null if there is no pose.
   * @return The limelight's outputted pose
   */
  public Pose2d getBotPose2dMT1() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(getName()).pose;
  }

  public PoseEstimate getMT1PoseEstimate() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(getName());
  }

  public Pose2d getBotPose2dMT2() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(getName()).pose;
  }

  public PoseEstimate getMT2PoseEstimate() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(getName());
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
  public double[] getBotPoseMT1() {
    return LimelightHelpers.getBotPose_wpiBlue(getName());
  }

  /**
   * Gets the full array of bot pose values in target space
   * @return An array of all pose values
   */
  public double[] getBotPoseTargetSpace() {
    return LimelightHelpers.getBotPose_TargetSpace(getName());
  }

  /**
   * Gets the ID of the biggest tag in frame.
   * @return The ID of the tag
   */
  public int getTagId() {
    return (int) tagId.getInteger(0);
  }

  /**
   * Gets the number of tags that the limelight can see.
   * @return The number of tags
   */
  public int getTagCount() {
    return LimelightHelpers.getTargetCount(getName());
  }

  public boolean acceptPose() {
    if (!LimelightHelpers.getTV(getName())) {
      return false;
    //} else if (Math.abs(getBotPoseMT1()[3]) > 1) {
      //return false;
    } else if (getBotPose2dMT2().getY() < 0) {
      return false;
    } else if (getBotPose2dMT2().getX() < 0) {
      return false;
    } else if (getBotPose2dMT2().getY() > VisionConstants.aprilTagLayout.getFieldWidth()) {
      return false;
    } else if (getBotPose2dMT2().getX() > VisionConstants.aprilTagLayout.getFieldLength()) {
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
    return LimelightHelpers.getTX(getName());
  }

  /**
   * Gets the Y Offset of the tag from the center of frame.
   * @return The Y Offset
   */
  public double getYOffset() {
    return LimelightHelpers.getTY(getName());
  }

  /**
   * Gets the area of the tag in frame
   * @return The area of the tag
   */
  public double getTargetArea() {
    return LimelightHelpers.getTA(getName());
  }

  /**
   * Turns the limelight LEDs on
   */
  public void setLedsOn() {
    LimelightHelpers.setLEDMode_ForceOn(getName());
  }

  public int getPipeline() {
    return (int) LimelightHelpers.getCurrentPipelineIndex(getName());
  }
};
