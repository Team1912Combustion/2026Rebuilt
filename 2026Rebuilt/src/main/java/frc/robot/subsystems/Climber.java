// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class Climber extends SubsystemBase {
  TalonFX elevator;

  TalonFXConfiguration config;

  double upperLimit, lowerLimit, climb;
  /** Creates a new Climber. */
  public Climber() {
    elevator = new TalonFX(MotorIDs.CLIMBER, new CANBus("1912CANivore"));

    config = new TalonFXConfiguration();
    config.Slot0.kG = 0;
    config.Slot0.kP = 0.8;
    config.Slot0.kI = 0.035;
    config.Slot0.kD = 0;
    config.MotionMagic.MotionMagicCruiseVelocity = 1600;
    config.MotionMagic.MotionMagicAcceleration = 1600;
    config.MotionMagic.MotionMagicJerk = 1600;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    elevator.getConfigurator().apply(config);

    upperLimit = 0;
    lowerLimit = -19.35;

    climb = -8;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Clamps a position between the upper and lower limits.
   * @param position The position to clamp
   * @return The clamped position
   */
  public double clampPosition(double position) {
    return Math.max(Math.min(position, upperLimit), lowerLimit);
  }

  public void elevatorUp() {
    final MotionMagicDutyCycle request = new MotionMagicDutyCycle(0).withSlot(0);
    elevator.setControl(request.withPosition(lowerLimit));
  }

  public void elevatorDown() {
    final MotionMagicDutyCycle request = new MotionMagicDutyCycle(0).withSlot(0);
    elevator.setControl(request.withPosition(0));
  }

  public void elevatorClimb() {
    final MotionMagicDutyCycle request = new MotionMagicDutyCycle(0).withSlot(0);
    elevator.setControl(request.withPosition(climb));
  }
}
