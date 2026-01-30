// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.TurretConstants;

public class TurretHood extends SubsystemBase {
  Turret turret;

  SparkMax hood;

  PIDController pid;
  double upperLimit, lowerLimit, currentPosition, targetPosition;
  /** Creates a new TurretHood. */
  public TurretHood(Turret t) {
    turret = t;

    hood = new SparkMax(MotorIDs.TURRET_HOOD, MotorType.kBrushless);

    pid = new PIDController(0, 0, 0);
    pid.setTolerance(1);

    upperLimit = 5;
    lowerLimit = 0;
    currentPosition = hood.getEncoder().getPosition();
    targetPosition = 0;

  }

  @Override
  public void periodic() {

    currentPosition = hood.getEncoder().getPosition();

    targetPosition = Math.max(Math.min(targetPosition, upperLimit), lowerLimit);
    hood.set(pid.calculate(currentPosition, targetPosition));

    // This method will be called once per scheduler run
  }

  /**
   * Returns the hood angle for the current distance from the shot point. There are different ranges of distances, each with a unique turret hood angle.
   * @param pose The current pose of the turret
   * @return The ideal hood angle
   */
  /*public double calculateHoodAngle(Pose2d pose) {
    double angle = 0;
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
  }*/

  /**
   * Returns the hood angle for the current distance from the shot point. There are different ranges of distances, each with a unique turret hood angle.
   * @param pose The current pose of the turret
   * @return The ideal hood angle
   */
  public double calculateHoodAngle(Pose2d pose) {
    double angle = 0;
    double distance = turret.getCurrentFieldZone().getDistanceFromShotPoint(pose);
    int index = 0;

    index = (int) Math.floor(distance / 1.5);
    angle = (turret.getCurrentFieldZone().getShotPointHeight() ? TurretConstants.LOW_HOOD_ANGLES[index] : TurretConstants.HIGH_HOOD_ANGLES[index]);

    return angle;
  }

  /**
   * Returns the hood angle for the current distance from the shot point. This is calculated as a function of distance.
   * @param pose The current pose of the turret
   * @return The ideal hood angle
   */
  public double calculateHoodAngleContinuous(Pose2d pose) {
    double angle = 0;
    double distance = turret.getCurrentFieldZone().getDistanceFromShotPoint(pose);

    // FIGURE OUT THIS FUNCTION AT SOME POINT

    return angle;
  }

  /**
   * Sets a target position for the PID controller.
   * @param target The position to go to
   */
  public void setPosition(double target) {
    targetPosition = target;
  }

  /**
   * Gets whether or not the turret hood is aligned.
   * @return Whether or not the hood is within tolerance for the PID controller
   */
  public boolean isInPosiiton() {
    return pid.atSetpoint();
  }
}