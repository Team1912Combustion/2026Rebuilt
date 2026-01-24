// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.SensorIDs;
import frc.robot.Constants.TurretConstants;

public class Shooter extends SubsystemBase {
  Turret turret;
  TalonFX shooter;
  TalonFX kicker;
  TalonFXConfiguration shooterConfig, kickerConfig;

  DigitalInput beambreak;

  double shotCount;
  boolean previousBeambreakState;

  double[] ranges;
  /** Creates a new Shooter. */
  public Shooter(Turret t) {
    turret = t;

    shooter = new TalonFX(MotorIDs.SHOOTER, "1912CANivore");
    kicker = new TalonFX(MotorIDs.KICKER, "1912CANivore");
    shooterConfig = new TalonFXConfiguration();
    kickerConfig = new TalonFXConfiguration();
    shooterConfig.Slot0.kS = 0.1;
    shooterConfig.Slot0.kV = 0.12;
    shooterConfig.Slot0.kP = 0;
    kickerConfig.Slot0.kS = 0.1;
    kickerConfig.Slot0.kV = 0.12;
    kickerConfig.Slot0.kP = 0;

    shooter.getConfigurator().apply(shooterConfig);
    kicker.getConfigurator().apply(kickerConfig);

    beambreak = new DigitalInput(SensorIDs.TURRET_BEAMBREAK);

    shotCount = 0;
    previousBeambreakState = true;

  }

  @Override
  public void periodic() {
    countShot();
    // This method will be called once per scheduler run
  }

  /**
   * Sets the speed of the shooter to a desired speed.
   * @param speed The speed to set the PID to, in rotations per second
   */
  public void setSpeed(double speed) {
    final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    shooter.setControl(request.withVelocity(speed));
  }

  /**
   * Gets whether or not the shooter speed is within a certain error limit of its target speed.
   * @return True if the shooter is within in the limit, false if it isn't
   */
  public boolean shooterAtSpeed() {
    return (shooter.getClosedLoopError().getValueAsDouble() < 20);
  }

  /**
   * Runs the kicker at a specific speed.
   */
  public void kickerOn() {
    final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    kicker.setControl(request.withVelocity(500));
  }

  /**
   * Gets the ideal shooter speed for the distance from the shot point. There are different ranges of distances, each with a unique shooter speed.
   * @param pose The current pose of the turret
   * @return The ideal speed
   */
  public double calculateSpeed(Pose2d pose) {
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
  }

  /**
   * Gets the ideal shooter speed for the distance from the shot point. It is calculated as a function of distance.
   * @param pose The current pose of the turret
   * @return The ideal speed
   */
  public double calculateSpeedContinuous(Pose2d pose) {
    double speed = 0;
    double distance = turret.getCurrentFieldZone().getDistanceFromShotPoint(pose);

    // FIGURE OUT THIS FUNCTION AT SOME POINT

    return speed;
  }

  /**
   * Adds a count of 1 to shotCount if a ball passes through the shooter.
   */
  public void countShot() {
    if (previousBeambreakState == false && beambreak.get() == true) {
      shotCount += 1;
    }

    previousBeambreakState = beambreak.get();
  }

  /**
   * Turns the shooter off.
   */
  public void shooterOff() {
    shooter.set(0);
  }

  /**
   * Turns the kicker off.
   */
  public void kickerOff() {
    kicker.set(0);
  }
}
