// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class Intake extends SubsystemBase {
  TalonFX rollers;
  TalonFX arm;

  TalonFXConfiguration rollerConfig;
  TalonFXConfiguration armConfig;

  SlewRateLimiter rateLimiter;

  double targetPosition;

  public boolean armOut;
  /** Creates a new Intake. */
  public Intake() {
    rollers = new TalonFX(MotorIDs.INTAKE_ROLLERS,  new CANBus("1912CANivore"));
    arm = new TalonFX(MotorIDs.INTAKE_ARM, new CANBus("1912CANivore"));
    rollerConfig = new TalonFXConfiguration();
    rollerConfig.Slot0.kS = 0.5;
    rollerConfig.Slot0.kV = 0.03;
    rollerConfig.Slot0.kP = 0.11;
    rollerConfig.Slot0.kI = 0;
    rollerConfig.Slot0.kD = 0;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    rollers.getConfigurator().apply(rollerConfig);

    armConfig = new TalonFXConfiguration();
    armConfig.Slot0.kG = 0.5;
    armConfig.Slot0.kP = 11;
    armConfig.Slot0.kI = 0;
    armConfig.Slot0.kD = 0;
    armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.Feedback.SensorToMechanismRatio = 20;
    // UPPER LIMIT //
    /*armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    // LOWER LIMIT //
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 4.2;*/

    arm.getConfigurator().apply(armConfig);
    arm.setPosition(0.22); 

    rateLimiter = new SlewRateLimiter(2);

    armOut = false;

    armIn();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setRollerSpeed(double speed) {
    final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    rollers.setControl(request.withVelocity(speed));
  }

  public void intake() {
    setRollerSpeed(80);
  }

  public void expel() {
    setRollerSpeed(-40);
  }

  public boolean rollersAtSpeed() {
    return (Math.abs(rollers.getClosedLoopError().getValueAsDouble()) < 20);
  }

  public void setArmPosition(double position) {
    final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    arm.setControl(request.withPosition(position));
  }

  public void armOut() {
    armOut = true;
    setArmPosition(0);
  }

  public void armIn() {
    armOut = false;
    setArmPosition(0.24);
  }

  public boolean armInPosition() {
    return (Math.abs(arm.getClosedLoopError().getValueAsDouble()) < 0.3);
  }
}
