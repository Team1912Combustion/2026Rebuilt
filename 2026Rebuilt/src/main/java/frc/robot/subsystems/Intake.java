// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class Intake extends SubsystemBase {
  TalonFX rollers;
  TalonFX arm;

  TalonFXConfiguration rollerConfig;
  TalonFXConfiguration armConfig;
  /** Creates a new Intake. */
  public Intake() {
    rollers = new TalonFX(MotorIDs.INTAKE_ROLLERS,  new CANBus("1912CANivore"));
    arm = new TalonFX(MotorIDs.INTAKE_ARM, new CANBus("1912CANivore"));

    rollerConfig = new TalonFXConfiguration();
    rollerConfig.Slot0.kS = 0;
    rollerConfig.Slot0.kV = 0;
    rollerConfig.Slot0.kP = 0;
    rollerConfig.Slot0.kI = 0;
    rollerConfig.Slot0.kD = 0;

    armConfig = new TalonFXConfiguration();
    armConfig.Slot0.kG = 0;
    armConfig.Slot0.kP = 0;
    armConfig.Slot0.kI = 0;
    armConfig.Slot0.kD = 0;
    // UPPER LIMIT //
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10;
    // LOWER LIMIT //
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setRollerSpeed(double speed) {
    final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    rollers.setControl(request.withVelocity(speed));
  }

  public boolean rollersAtSpeed() {
    return (Math.abs(rollers.getClosedLoopError().getValueAsDouble()) < 20);
  }

  public void setArmPosition(double position) {
    final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    arm.setControl(request.withPosition(position));
  }

  public void armOut() {
    setArmPosition(10);
  }

  public void armIn() {
    setArmPosition(0);
  }

  public boolean armInPosition() {
    return (Math.abs(arm.getClosedLoopError().getValueAsDouble()) < 0.3);
  }
}
