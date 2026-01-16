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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Shooter extends SubsystemBase {
  Turret turret;
  TalonFX shooter;
  TalonFXConfiguration config;

  double[] ranges;
  /** Creates a new Shooter. */
  public Shooter(Turret t) {
    turret = t;

    shooter = new TalonFX(0, "1912CANivore");
    config = new TalonFXConfiguration();
    config.Slot0.kS = 0.1;
    config.Slot0.kV = 0.12;
    config.Slot0.kP = 0;

    shooter.getConfigurator().apply(config);

  }

  @Override
  public void periodic() {
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
   * Gets the ideal shooter speed for the distance from the shot point. There are different ranges of distances, each with a unique shooter speed.
   * @param pose The current pose of the robot
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
   * Turns the shooter off.
   */
  public void off() {
    shooter.set(0);
  }
}
