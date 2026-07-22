// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class IntakeRollers extends SubsystemBase {
  TalonFX rollersLeft;
  TalonFX rollersRight;

  TalonFXConfiguration rollerConfig;

  double targetPosition;

  /** Creates a new IntakeRollers. */
  public IntakeRollers() {
    rollersLeft = new TalonFX(MotorIDs.INTAKE_ROLLERS_LEFT,  new CANBus("1912CANivore"));
    rollersRight = new TalonFX(MotorIDs.INTAKE_ROLLERS_RIGHT,  new CANBus("1912CANivore"));
    rollerConfig = new TalonFXConfiguration();
    rollerConfig.Slot0.kS = 0.35;
    rollerConfig.Slot0.kV = 0.12;
    rollerConfig.Slot0.kP = 0.2;
    rollerConfig.Slot0.kI = 0;
    rollerConfig.Slot0.kD = 0;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rollersLeft.getConfigurator().apply(rollerConfig);
    rollersRight.getConfigurator().apply(rollerConfig);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setRollerSpeed(double speed) {
    final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    rollersLeft.setControl(request.withVelocity(speed).withEnableFOC(true));
    final Follower followerRequest = new Follower(MotorIDs.INTAKE_ROLLERS_LEFT, MotorAlignmentValue.Opposed);
    rollersRight.setControl(followerRequest);
  }

  public void intake() {
    setRollerSpeed(100);
  }

  public void intakeSlow() {
    setRollerSpeed(15);
  }

  public void expel() {
    setRollerSpeed(-90);
  }

  public boolean rollersAtSpeed() {
    return (Math.abs(rollersLeft.getClosedLoopError().getValueAsDouble()) < 20);
  }

}
