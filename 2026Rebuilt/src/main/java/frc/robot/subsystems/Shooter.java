// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.QuadraticSolver;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.SensorIDs;
import frc.robot.Constants.TurretConstants;

public class Shooter extends SubsystemBase {
  DriveTrain driveTrain;
  TalonFX shooter1, shooter2, shooter3, shooter4;
  TalonFX kicker;
  TalonFXConfiguration shooterConfig, kickerConfig;

  QuadraticSolver quadraticSolver;

  double upperLimit, lowerLimit;

  public double boost;

  Debouncer debouncer = new Debouncer(0.1, DebounceType.kFalling);

  /** Creates a new Shooter. */
  public Shooter(DriveTrain dt) {
    driveTrain = dt;

    shooter1 = new TalonFX(MotorIDs.SHOOTER_1, new CANBus("1912CANivore"));
    shooter2 = new TalonFX(MotorIDs.SHOOTER_2, new CANBus("1912CANivore"));
    shooter3 = new TalonFX(MotorIDs.SHOOTER_3, new CANBus("1912CANivore"));
    shooter4 = new TalonFX(MotorIDs.SHOOTER_4, new CANBus("1912CANivore"));
    kicker = new TalonFX(MotorIDs.KICKER, new CANBus("1912CANivore"));

    shooterConfig = new TalonFXConfiguration();
    shooterConfig.Slot0.kS = 0.25;
    shooterConfig.Slot0.kV = 0.12;
    shooterConfig.Slot0.kP = 0.25;
    shooterConfig.Slot0.kI = 0;
    shooterConfig.Slot0.kD = 0;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = 40;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    
    kickerConfig = new TalonFXConfiguration();
    kickerConfig.Slot0.kS = 0.4;
    kickerConfig.Slot0.kV = 0.1;
    kickerConfig.Slot0.kP = 0.28;
    kickerConfig.Slot0.kI = 0;
    kickerConfig.Slot0.kD = 0;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerConfig.CurrentLimits.SupplyCurrentLimit = 40;
    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    shooter1.getConfigurator().apply(shooterConfig);
    shooter2.getConfigurator().apply(shooterConfig);
    shooter3.getConfigurator().apply(shooterConfig);
    shooter4.getConfigurator().apply(shooterConfig);
    kicker.getConfigurator().apply(kickerConfig);

    quadraticSolver = new QuadraticSolver();

    upperLimit = 100;
    lowerLimit = -100;

    boost = -0.3;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("shooter speed", getSpeed());
    SmartDashboard.putNumber("shooter boost", boost);
    // This method will be called once per scheduler run
  }

  /**
   * Sets the speed of the shooter to a desired speed.
   * @param speed The speed to set the PID to, in rotations per second
   */
  public void setSpeed(double speed) {
    double targetSpeed = Math.min(upperLimit, Math.max(speed, lowerLimit));
    final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    shooter1.setControl(request.withVelocity(targetSpeed).withEnableFOC(true));
    final Follower followerRequest = new Follower(0, MotorAlignmentValue.Aligned);
    shooter2.setControl(followerRequest.withLeaderID(MotorIDs.SHOOTER_1).withMotorAlignment(MotorAlignmentValue.Aligned));
    shooter3.setControl(followerRequest.withLeaderID(MotorIDs.SHOOTER_1).withMotorAlignment(MotorAlignmentValue.Opposed));
    shooter4.setControl(followerRequest.withLeaderID(MotorIDs.SHOOTER_1).withMotorAlignment(MotorAlignmentValue.Opposed));
  }

  /**
   * Gets the speed of the shooter wheel.
   * @return The speed of the shooter wheel, in rotations per second
   */
  public double getSpeed() {
    return shooter1.getVelocity().getValueAsDouble();
  }

  /**
   * Gets the value that the shooter pid is set to.
   * @return The set velocity of the shooter, in rotations per second
   */
  public double getShooterTarget() {
    return shooter1.getClosedLoopReference().getValueAsDouble();
  }

  /**
   * Gets whether or not the shooter speed is within a certain error limit of its target speed.
   * @return True if the shooter is within in the limit, false if it isn't
   */
  public boolean shooterAtSpeed() {
    return debouncer.calculate((Math.abs(shooter1.getClosedLoopError().getValueAsDouble()) < 0.5));
  }

  /**
   * Runs the kicker at a specific speed.
   */
  public void kickerOn() {
    final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    kicker.setControl(request.withVelocity(-115).withEnableFOC(true));
  }

  /**
   * Runs the kicker at a specific speed.
   */
  public void kickerReverse() {
    final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    kicker.setControl(request.withVelocity(20).withEnableFOC(true));
  }

  /**
   * Gets the ideal shooter speed for the distance from the shot point. There are different ranges of distances, each with a unique shooter speed.
   * @param pose The current pose of the turret
   * @return The ideal speed
   */
  /*public double calculateSpeed(Pose2d pose) {
    double speed = 0;
    double distance = turret.getCurrentFieldZone().getDistanceFromShotPoint(pose);
    int index = -1;
    for (double[] range : TurretConstants.DISTANCES) {
      index += 1;
      if ((distance > range[0]) && (distance < range[1])) {
        speed = TurretConstants.SPEEDS[index];
        break;
      }
    }

    return speed;
  }*/
  
  /**
   * Gets the ideal shooter speed for the distance from the shot point.
   * @param distance The distance from the target, measured in meters for pose mode and measured by tag area for tag mode
   * @return The ideal speed
   */
  public double calculateSpeed(double distance) {
    double speed = 0;
    int index = 0;

    index = (int) Math.floor(distance / TurretConstants.DELTA_DISTANCE);
    speed = TurretConstants.SPEEDS[index];

    return speed;
  }

  /**
   * Gets the ideal shooter speed for the distance from the shot point. It is calculated as a function of distance.
   * @param pose The current pose of the turret
   * @return The ideal speed
   */
  public double calculateSpeedContinuous(double distance) {
    double speed = -28.61997 - (11.55667 * Math.log(distance + boost));

    return speed;
  }

  /**
   * Gets the amount of time the ball takes to reach the hub based on the initial velocity of the ball.
   * @return The time the ball takes to reach the hub, in seconds
   */
  public double calculateFuelTravelTime() {
    double rps = shooter1.getVelocity().getValueAsDouble();
    double shootSpeed = rps * TurretConstants.SHOOTER_WHEEL_CIRCUMFERENCE;

    return quadraticSolver.findZeros(-4.9, shootSpeed, -1.3);
  }

  /**
   * Turns the shooter off.
   */
  public void shooterOff() {
    shooter1.set(0);
  }

  /**
   * Turns the kicker off.
   */
  public void kickerOff() {
    kicker.set(0);
  }
}
