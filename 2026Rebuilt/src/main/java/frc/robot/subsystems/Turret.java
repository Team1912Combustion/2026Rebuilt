// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldZoneConstants;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.SensorIDs;
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

  }

  @Override
  public void periodic() {
    checkLimitSwitches();

    turretAngle = new Rotation2d((currentPosition / upperLimit) / Math.PI);
    turretAngle = new Rotation2d(Math.toRadians(turretAngle.getDegrees() - MathUtil.inputModulus(driveTrain.getHeading(), -180, 180)));

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
    targetPosition = (angle / Math.PI);
  }

  /**
   * Gets the turret's current angle relative to the field.
   * @return The angle, in degrees, of the turret relative to the field
   */
  public double getTurretAngle() {
    return turretAngle.getDegrees();
  }

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
}
