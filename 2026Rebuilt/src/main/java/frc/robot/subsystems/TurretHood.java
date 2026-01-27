// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.TurretConstants;

public class TurretHood extends SubsystemBase {
  Turret turret;

  ServoHub servoHub;
  ServoChannel servo;

  /** Creates a new TurretHood. */
  public TurretHood(Turret t) {
    turret = t;

    servoHub = new ServoHub(0);
    
    servo = servoHub.getServoChannel(ChannelId.kChannelId0);

    servo.setEnabled(true);

  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  /**
   * Returns the hood angle for the current distance from the shot point. There are different ranges of distances, each with a unique turret hood angle.
   * @param pose The current pose of the turret
   * @return The ideal hood angle
   */
  public int calculateHoodAngle(Pose2d pose) {
    int angle = 0;
    double distance = turret.getCurrentFieldZone().getDistanceFromShotPoint(pose);
    int index = -1;
    for (double[] range : TurretConstants.DISTANCES) {
      index += 1;
      if ((distance > range[0]) && (distance < range[1])) {
        angle = (turret.getCurrentFieldZone().getShotPointHeight() ? TurretConstants.LOW_HOOD_ANGLES[index] : TurretConstants.HIGH_HOOD_ANGLES[index]);
        break;
      }
    }

    return angle;
  }

  /**
   * Returns the hood angle for the current distance from the shot point. This is calculated as a function of distance.
   * @param pose The current pose of the turret
   * @return The ideal hood angle
   */
  public int calculateHoodAngleContinuous(Pose2d pose) {
    int angle = 0;
    double distance = turret.getCurrentFieldZone().getDistanceFromShotPoint(pose);

    // FIGURE OUT THIS FUNCTION AT SOME POINT

    return angle;
  }

  /**
   * Sets a target position for the PID controller.
   * @param target The position to go to
   */
  public void setPosition(int target) {
    servo.setPulseWidth(target);
  }

  /**
   * Gets whether or not the turret hood is aligned.
   * @return Whether or not the hood is within tolerance for the PID controller
   */
  public boolean isInPosiiton() {
    return (servo.getCurrent() > 0.04) && (servo.getCurrent() < 0.1);
  }
}
