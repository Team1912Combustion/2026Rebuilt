// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class Turret extends SubsystemBase {
  TalonFX turret;

  ProfiledPIDController pid;

  double targetPosition, currentPosition, error;
  /** Creates a new Turret. */
  public Turret() {
    turret = new TalonFX(MotorIDs.TURRET, "1912CANivore");
    turret.setPosition(0);

    pid = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
    pid.enableContinuousInput(0, 36);

    targetPosition = 0;
    currentPosition = 0;
    error = 0;
  }

  @Override
  public void periodic() {
    currentPosition = turret.getPosition().getValueAsDouble();
    error = currentPosition - targetPosition;

    turret.set(pid.calculate(currentPosition, targetPosition));
    // This method will be called once per scheduler run
  }
}
