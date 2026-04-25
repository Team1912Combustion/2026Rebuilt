// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldZone;
import frc.robot.Constants.FieldZoneConstants;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.TurretConstants;

public class Hood extends SubsystemBase {
  DriveTrain driveTrain;

  TalonFX hood;
  TalonFXConfiguration config;

  double upperLimit, lowerLimit, currentPosition, targetPosition;

  FieldZone blueDepotTrench, blueOutpostTrench, redDepotTrench, redOutpostTrench, noTrench;

  boolean trenchesReassigned;
  public boolean duckHood;
  /** Creates a new Hood. */
  public Hood(DriveTrain dt) {
    driveTrain = dt;

    hood = new TalonFX(MotorIDs.HOOD, new CANBus("1912CANivore"));
    
    config = new TalonFXConfiguration();
    config.Slot0.kP = 7;
    config.Slot0.kI = 0.;
    config.Slot0.kD = 0.05;
    config.Slot0.kG = 0.4;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    hood.getConfigurator().apply(config);
    hood.setPosition(0);

    upperLimit = 2.0;
    lowerLimit = 0;
    currentPosition = hood.getPosition().getValueAsDouble();
    targetPosition = 0;

    blueDepotTrench = FieldZoneConstants.BLUE_DEPOT_TRENCH_ZONE;
    blueOutpostTrench = FieldZoneConstants.BLUE_OUTPOST_TRENCH_ZONE;
    redDepotTrench = FieldZoneConstants.RED_DEPOT_TRENCH_ZONE;
    redOutpostTrench = FieldZoneConstants.RED_OUTPOST_TRENCH_ZONE;
    noTrench = new FieldZone(new Translation2d(-1, -1), new Translation2d(-0.5, -1.5), "NO TRENCH");

    trenchesReassigned = false;

    duckHood = true;
  }

  @Override
  public void periodic() {

    if (blueDepotTrench.isInZone(driveTrain.getPose()) || 
      blueOutpostTrench.isInZone(driveTrain.getPose()) ||
      redDepotTrench.isInZone(driveTrain.getPose()) ||
      redOutpostTrench.isInZone(driveTrain.getPose())
      ) {
        duckHood = true;
      }

    /*if (DriverStation.isTeleop() && DriverStation.isFMSAttached() && !trenchesReassigned) {
      blueDepotTrench = FieldZoneConstants.BLUE_DEPOT_TRENCH_ZONE_TELEOP;
      blueOutpostTrench = FieldZoneConstants.BLUE_OUTPOST_TRENCH_ZONE_TELEOP;
      redDepotTrench = FieldZoneConstants.RED_DEPOT_TRENCH_ZONE_TELEOP;
      redOutpostTrench = FieldZoneConstants.RED_OUTPOST_TRENCH_ZONE_TELEOP;

      trenchesReassigned = true;
    }*/

    currentPosition = hood.getPosition().getValueAsDouble();

    SmartDashboard.putNumber("hood position", getPosition());

    // This method will be called once per scheduler run
  }

  /**
   * Gets the ideal hood angle for the distance from the shot point.
   * @param distance The distance from the target, measured in meters
   * @return The ideal hood angle
   */
  public double calculateHoodAngle(double distance) {
    double angle = 0;
    int index = 0;

    index = (int) Math.floor(distance / TurretConstants.DELTA_DISTANCE);
    angle = (driveTrain.getCurrentFieldZone().getShotPointHeight() ? TurretConstants.LOW_HOOD_ANGLES[index] : TurretConstants.HIGH_HOOD_ANGLES[index]);

    return angle;
  }

  /**
   * Returns the hood angle for the current distance from the shot point. This is calculated as a function of distance.
   * @param pose The current pose of the turret
   * @return The ideal hood angle
   */
  public double calculateHoodAngleContinuous(double distance) {
    double angle = 0.053185 * Math.pow(2.09567, distance);
    
    return angle;
  }

  /**
   * Sets a target position for the PID controller.
   * @param target The position to go to
   */
  public void setPosition(double target) {
    double targetPosition = Math.min(upperLimit, Math.max(target, lowerLimit));
    final PositionVoltage request = new PositionVoltage(0);
    hood.setControl(request.withPosition(targetPosition).withEnableFOC(true));
  }

  /**
   * Gets the position of the hood encoder.
   * @return The position of the hood encoder in motor rotations
   */
  public double getPosition() {
    return hood.getPosition().getValueAsDouble();
  }

  /**
   * Gets the target position of the hood pid.
   * @return The target position, in motor rotations
   */
  public double getTarget() {
    return hood.getClosedLoopReference().getValueAsDouble();
  }

  /**
   * Gets whether or not the turret hood is aligned.
   * @return Whether or not the hood is within tolerance for the PID controller
   */
  public boolean isInPosiiton() {
    return (hood.getClosedLoopError().getValueAsDouble() < 0.5);
  }

  /**
   * Gets the FieldZone that the turret is currently in.
   * @return The FieldZone that the robot is in
   */
  public FieldZone getCurrentFieldZone() {
    if (blueDepotTrench.isInZone(driveTrain.getPose())) {
      return blueDepotTrench;
    } else if (blueOutpostTrench.isInZone(driveTrain.getPose())) {
      return blueOutpostTrench; 
    } else if (redDepotTrench.isInZone(driveTrain.getPose())) {
      return redDepotTrench;
    } else if (redOutpostTrench.isInZone(driveTrain.getPose())) {
      return redOutpostTrench;
    } else {
      return noTrench;
    }
  }

  /**
   * Returns whether or not the turret is near the trench and should duck.
   * @return True if the hood should be ducked, false if it should not
   */
  public boolean duckHood() {
    return duckHood;
  }
}