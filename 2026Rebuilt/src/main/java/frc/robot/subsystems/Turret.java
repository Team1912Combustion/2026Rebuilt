// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldZoneConstants;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.SensorIDs;
import frc.robot.Constants.TurretConstants;
import frc.robot.FieldZone;

public class Turret extends SubsystemBase {
  DriveTrain driveTrain;

  TalonFX turret;

  ProfiledPIDController pid;

  double targetPosition, currentPosition, error, upperLimit, lowerLimit;

  Rotation2d turretAngle;
  Pose2d turretPose;
  Translation2d turretOffset;

  DigitalInput leftLimitSwitch, rightLimitSwitch;
  Timer limitSwitchTimer;

  FieldZone blueDepotZone;
  FieldZone blueOutpostZone;
  FieldZone redDepotZone;
  FieldZone redOutpostZone;
  FieldZone neutralTopZone;
  FieldZone neutralBottomZone;

  String targetMode;

  Integer[] hubTagsArray = {8, 10, 11, 24, 26, 27};

  List<Integer> hubTags = Arrays.asList(hubTagsArray);
  
  /** Creates a new Turret. */
  public Turret(DriveTrain dt) {
    driveTrain = dt;

    turret = new TalonFX(MotorIDs.TURRET, "1912CANivore");
    turret.setPosition(0);

    pid = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));

    targetPosition = 0;
    currentPosition = 0;
    error = 0;
    upperLimit = 18;
    lowerLimit = -18;
    turretAngle = new Rotation2d(0);
    turretPose = new Pose2d(driveTrain.getPose().getTranslation(), turretAngle);
    turretOffset = new Translation2d();

    leftLimitSwitch = new DigitalInput(SensorIDs.TURRET_LEFT_LIMIT_SWITCH);
    rightLimitSwitch = new DigitalInput(SensorIDs.TURRET_RIGHT_LIMIT_SWITCH);

    limitSwitchTimer = new Timer();

    blueDepotZone = FieldZoneConstants.BLUE_DEPOT_ZONE;
    blueOutpostZone = FieldZoneConstants.BLUE_OUTPOST_ZONE;
    redDepotZone = FieldZoneConstants.RED_DEPOT_ZONE;
    redOutpostZone = FieldZoneConstants.RED_OUTPOST_ZONE;
    neutralTopZone = FieldZoneConstants.NEUTRAL_TOP_ZONE;
    neutralBottomZone = FieldZoneConstants.NEUTRAL_BOTTOM_ZONE;

    targetMode = "pose";

  }

  @Override
  public void periodic() {
    checkLimitSwitches();

    if (DriverStation.isDisabled()) {
      setShotPoints();
    }

    turretAngle = Rotation2d.fromDegrees((currentPosition / upperLimit) * 180);
    turretAngle = Rotation2d.fromDegrees(turretAngle.getDegrees() - MathUtil.inputModulus(driveTrain.getHeading(), -180, 180));

    turretPose = new Pose2d(driveTrain.getPose().getTranslation().minus(turretOffset), turretAngle);

    currentPosition = turret.getPosition().getValueAsDouble();
    targetPosition = Math.min(Math.max(targetPosition, lowerLimit), upperLimit);
    error = currentPosition - targetPosition;

    turret.set(pid.calculate(currentPosition, targetPosition));
    // This method will be called once per scheduler run
  }

  /**
   * Sets the turret's target position to a certain field-relative angle.
   * @param angle The angle, in degrees, to set the turret to
   */
  public void setTurretAngle(double angle) {
    targetPosition = ((turretAngle.getDegrees() + MathUtil.inputModulus(driveTrain.getHeading(), -180, 180)) / 180) * upperLimit;
  }

  /**
   * Gets the turret's current angle relative to the field.
   * @return The angle, in degrees, of the turret relative to the field
   */
  public double getTurretAngle() {
    return turretAngle.getDegrees();
  }

  /**
   * Aims the turret at a tag using the x offset from center.
   * @param xOffset The x offset reported by the limelight
   */
  public void aimAtTag(double xOffset) {
    if (xOffset < TurretConstants.X_OFFSET_THRESHHOLD) {
      targetPosition += (xOffset * 0.001);
    }
  }

  /**
   * Gets the pose of the turret on the field. This is different than the robot's pose. The Rotation2d component is the turret's current angle.
   * @return The pose of the turret
   */
  public Pose2d getTurretPose() {
    return turretPose;
  }

  /**
   * Used to set the target mode to either 'pose' or 'tag'. 'pose' and 'tag' are the only valid arguments.
   */
  public void setTargetMode(String mode) {
    targetMode = mode;
  }

  /**
   * Gets the current target mode of the turret.
   * @return The target mode of the turret, either 'pose' or 'tag'
   */
  public String getTargetMode() {
    return targetMode;
  }

  /**
   * Resets the turret to -180 or 180 degrees when a certain limit switch is pressed for greater than 0.5 seconds.
   */
  public void checkLimitSwitches() {
    if (leftLimitSwitch.get() || rightLimitSwitch.get()) {
      limitSwitchTimer.start();
      if (limitSwitchTimer.get() > 0.5 && leftLimitSwitch.get()) {
        turret.setPosition(lowerLimit);
      }
      if (limitSwitchTimer.get() > 0.5 && rightLimitSwitch.get()) {
        turret.setPosition(upperLimit);
      }
    } else {
      limitSwitchTimer.stop();
      limitSwitchTimer.reset();
    }
  }

  /**
   * Gets the angle from one Translation2d to another.
   * @param origin The Translation2d to start from
   * @param goal The Translation2d to point to
   * @return The angle as a Rotation2d
   */
  public Rotation2d getDirection(Translation2d origin, Translation2d goal) {
    return Rotation2d.fromRadians(Math.atan2(
      turretPose.relativeTo(new Pose2d(blueDepotZone.getShotPoint(), new Rotation2d())).getY(), 
      turretPose.relativeTo(new Pose2d(blueDepotZone.getShotPoint(), new Rotation2d())).getX()
      ));
    
  }

  /**
   * Sets all shot points for each FieldZone based on the current alliance.
   */
  public void setShotPoints() {
    if (DriverStation.isDSAttached()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        blueDepotZone.setShotPoint(FieldZoneConstants.BLUE_HUB_SHOT_POINT);
        blueOutpostZone.setShotPoint(FieldZoneConstants.BLUE_HUB_SHOT_POINT);
        neutralTopZone.setShotPoint(FieldZoneConstants.NEUTRAL_TOP_ZONE_BLUE_SHOT_POINT);
        neutralBottomZone.setShotPoint(FieldZoneConstants.NEUTRAL_BOTTOM_ZONE_BLUE_SHOT_POINT);
        redDepotZone.setShotPoint(FieldZoneConstants.RED_DEPOT_SHOT_POINT);
        redOutpostZone.setShotPoint(FieldZoneConstants.RED_OUTPOST_SHOT_POINT);
      } else {
        blueDepotZone.setShotPoint(FieldZoneConstants.BLUE_DEPOT_SHOT_POINT);
        blueOutpostZone.setShotPoint(FieldZoneConstants.BLUE_OUTPOST_SHOT_POINT);
        neutralTopZone.setShotPoint(FieldZoneConstants.NEUTRAL_TOP_ZONE_RED_SHOT_POINT);
        neutralTopZone.setShotPoint(FieldZoneConstants.NEUTRAL_BOTTOM_ZONE_RED_SHOT_POINT);
        redDepotZone.setShotPoint(FieldZoneConstants.RED_HUB_SHOT_POINT);
        redOutpostZone.setShotPoint(FieldZoneConstants.RED_HUB_SHOT_POINT);
      }
    }
  }

  /**
   * Gets the FieldZone that the robot is currently in.
   * @return The FieldZone that the robot is in
   */
  public FieldZone getCurrentFieldZone() {
    if (blueDepotZone.isInZone(driveTrain.getPose())) {
      return blueDepotZone;
    } else if (blueOutpostZone.isInZone(driveTrain.getPose())) {
      return blueOutpostZone; 
    } else if (neutralTopZone.isInZone(driveTrain.getPose())) {
      return neutralTopZone;
    } else if (neutralBottomZone.isInZone(driveTrain.getPose())) {
      return neutralBottomZone;
    } else if (redDepotZone.isInZone(driveTrain.getPose())) {
      return redDepotZone;
    } else if (redOutpostZone.isInZone(driveTrain.getPose())) {
      return redOutpostZone;
    } else {
      return null;
    }
  }
}
