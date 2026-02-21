// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  TalonFXConfiguration turretConfig;

  double targetPosition, currentPosition, error, upperLimit, lowerLimit;

  // smaller to larger
  double[] deadZone = {-80, -10};

  Rotation2d turretAngle;
  Pose2d turretPose;

  Timer limitSwitchTimer;

  DigitalInput lowerLimitSwitch, upperLimitSwitch;

  FieldZone blueDepotZone;
  FieldZone blueOutpostZone;
  FieldZone redDepotZone;
  FieldZone redOutpostZone;
  FieldZone neutralTopZone;
  FieldZone neutralBottomZone;

  Integer[] hubTagsArray = {8, 10, 11, 24, 26, 27};

  List<Integer> hubTags = Arrays.asList(hubTagsArray);
  
  /** Creates a new Turret. */
  public Turret(DriveTrain dt) {
    driveTrain = dt;

    turret = new TalonFX(MotorIDs.TURRET, new CANBus("1912CANivore"));
    turret.setPosition(0);

    turretConfig = new TalonFXConfiguration();
    turretConfig.Slot0.kS = 0;
    turretConfig.Slot0.kV = 0;
    turretConfig.Slot0.kA = 0;
    turretConfig.Slot0.kP = 0;
    turretConfig.Slot0.kI = 0;
    turretConfig.Slot0.kD = 0;

    targetPosition = 0;
    currentPosition = 0;
    error = 0;
    upperLimit = 18;
    lowerLimit = -18;

    lowerLimitSwitch = new DigitalInput(SensorIDs.TURRET_LEFT_LIMIT_SWITCH);
    upperLimitSwitch = new DigitalInput(SensorIDs.TURRET_RIGHT_LIMIT_SWITCH);

    turretAngle = new Rotation2d(0);
    turretPose = driveTrain.getPose().plus(new Transform2d(TurretConstants.TURRET_OFFSET, turretAngle));

    limitSwitchTimer = new Timer();

    blueDepotZone = FieldZoneConstants.BLUE_DEPOT_ZONE;
    blueOutpostZone = FieldZoneConstants.BLUE_OUTPOST_ZONE;
    redDepotZone = FieldZoneConstants.RED_DEPOT_ZONE;
    redOutpostZone = FieldZoneConstants.RED_OUTPOST_ZONE;
    neutralTopZone = FieldZoneConstants.NEUTRAL_TOP_ZONE;
    neutralBottomZone = FieldZoneConstants.NEUTRAL_BOTTOM_ZONE;
  }

  @Override
  public void periodic() {
    checkLimitSwitches();

    if (DriverStation.isDisabled()) {
      setShotPoints();
    }

    turretAngle = Rotation2d.fromDegrees((currentPosition / upperLimit) * 180);

    turretPose = driveTrain.getPose().plus(new Transform2d(TurretConstants.TURRET_OFFSET, turretAngle));

    currentPosition = turret.getPosition().getValueAsDouble();
    targetPosition = Math.min(Math.max(targetPosition, lowerLimit), upperLimit);
    error = currentPosition - targetPosition;

    final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    turret.setControl(request.withPosition(targetPosition));

    SmartDashboard.putString("Current field zone", getCurrentFieldZone().getFieldZoneName());
    // This method will be called once per scheduler run
  }

  /**
   * Loops a value between the lower limit and upper limit of the motor.
   * @param value The value to loop
   * @return The looped value
   */
  public double motorModulus(double value) {
    return MathUtil.inputModulus(value, lowerLimit, upperLimit);
  }
  /**
   * Loops a value between -180 and 180.
   * @param value The value to loop
   * @return The looped value
   */
  public double angleModulus(double value) {
    return MathUtil.inputModulus(value, -180, 180);
  }
  /**
   * Sets the turret's target position to a certain field-relative angle.
   * @param angle The angle, in degrees, to set the turret to
   */
  public void setTurretAngle(double angle) {
    double robotRelativeAngle = (angle - angleModulus(driveTrain.getPose().getRotation().getDegrees()));
    double modifiedAngle = robotRelativeAngle;
    if (getCurrentFieldZone().getShotPointHeight()) {
      modifiedAngle = (isAngleInDeadZone(robotRelativeAngle) ? Math.max(deadZone[1], Math.min(deadZone[0], robotRelativeAngle)) : robotRelativeAngle);
    } else {
      modifiedAngle = robotRelativeAngle;
    }
    targetPosition = motorModulus((modifiedAngle / 180) * upperLimit);
  }

  /**
   * Returns whether or not an angle is within the turret's dead zone (an area that the turret can not shoot through, like the elevator).
   * @param angle The angle to check
   * @return Whether or not it is within the robot relative dead zone
   */
  public boolean isAngleInDeadZone(double angle) {
    return ((angle > deadZone[0]) && (angle < deadZone[1]));
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
   * Gets whether or not the turret is within the tolerance for the PID controller.
   * @return Whether or not the turret is within tolerance
   */
  public boolean isAimed() {
    return (turret.getClosedLoopError().getValueAsDouble() < TurretConstants.TURRET_ALLOWED_ERROR);
  }

  /**
   * Resets the turret to -180 or 180 degrees when a certain limit switch is pressed for greater than 0.5 seconds.
   */
  public void checkLimitSwitches() {
    if (lowerLimitSwitch.get() || upperLimitSwitch.get()) {
      limitSwitchTimer.start();
      if (limitSwitchTimer.get() > 0.5 && lowerLimitSwitch.get()) {
        turret.setPosition(lowerLimit);
      }
      if (limitSwitchTimer.get() > 0.5 && upperLimitSwitch.get()) {
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
  public Rotation2d getDirection(Pose2d origin, Pose2d goal) {
    Pose2d originPose = new Pose2d(origin.getTranslation(), new Rotation2d());
    Pose2d goalPose = new Pose2d(goal.getTranslation(), new Rotation2d());
    return Rotation2d.fromRadians(Math.atan2(
      goalPose.relativeTo(originPose).getY(), 
      goalPose.relativeTo(originPose).getX()
      )); 
  }

  /**
   * Gets the pose of the target, adjusted for the speed of the robot.
   * @return The pose of the target
   */
  public Pose2d getTarget() {
    return addVector(new Pose2d(getCurrentFieldZone().getShotPoint(), new Rotation2d()));
  }

  /**
   * Gets the distance between two poses.
   * @param origin The pose to start from
   * @param goal The pose to end at
   * @return The distance between the poses
   */
  public double getDistance(Translation2d origin, Translation2d goal) {
    return origin.getDistance(goal);
  }

  /**
   * Adds the robot's speed vector to a supplied pose
   * @param pose The pose to add to
   * @return The updated pose
   */
  public Pose2d addVector(Pose2d pose) {
    return pose.exp(driveTrain.getRobotSpeed());
  }

  /**
   * Sets all shot points for each FieldZone based on the current alliance.
   */
  public void setShotPoints() {
    if (DriverStation.isDSAttached()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        blueDepotZone.setShotPoint(FieldZoneConstants.BLUE_HUB_SHOT_POINT, false);
        blueOutpostZone.setShotPoint(FieldZoneConstants.BLUE_HUB_SHOT_POINT, false);
        neutralTopZone.setShotPoint(FieldZoneConstants.NEUTRAL_TOP_ZONE_BLUE_SHOT_POINT, true);
        neutralBottomZone.setShotPoint(FieldZoneConstants.NEUTRAL_BOTTOM_ZONE_BLUE_SHOT_POINT, true);
        redDepotZone.setShotPoint(FieldZoneConstants.RED_DEPOT_SHOT_POINT, true);
        redOutpostZone.setShotPoint(FieldZoneConstants.RED_OUTPOST_SHOT_POINT, true);
      } else {
        blueDepotZone.setShotPoint(FieldZoneConstants.BLUE_DEPOT_SHOT_POINT, true);
        blueOutpostZone.setShotPoint(FieldZoneConstants.BLUE_OUTPOST_SHOT_POINT, true);
        neutralTopZone.setShotPoint(FieldZoneConstants.NEUTRAL_TOP_ZONE_RED_SHOT_POINT, true);
        neutralBottomZone.setShotPoint(FieldZoneConstants.NEUTRAL_BOTTOM_ZONE_RED_SHOT_POINT, true);
        redDepotZone.setShotPoint(FieldZoneConstants.RED_HUB_SHOT_POINT, false);
        redOutpostZone.setShotPoint(FieldZoneConstants.RED_HUB_SHOT_POINT, false);
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
