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

public class LimelightFrontLeft extends SubsystemBase {
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
  private double[] lastBotPose = new double[6];
  private int currentPipeline = 0;
  private int currentTagId = 0;
  /** Creates a new LimelightFrontLeft. */
  public LimelightFrontLeft() {}

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
    return "limelight-fl";
  }

  /**
   * Gets the position of the limelight.
   * @return The position of the limelight as a string
   */
  public String getPosition() {
    return "frontLeft";
  }
  /**
   * Toggles between pipeline 0 and 1.
   */
  public void togglezoom() {
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

}