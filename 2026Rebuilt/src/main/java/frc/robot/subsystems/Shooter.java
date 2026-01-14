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

public class Shooter extends SubsystemBase {
  Turret turret;
  TalonFX shooter;
  TalonFXConfiguration config;
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

  public void setSpeed(double speed) {
    final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    shooter.setControl(request.withVelocity(speed));
  }

  public double calculateSpeed(Pose2d pose) {
    return ((turret.getCurrentFieldZone().getDistanceFromShotPoint(pose))/6.13 * 93);
  }

  public void off() {
    shooter.set(0);
  }
}
