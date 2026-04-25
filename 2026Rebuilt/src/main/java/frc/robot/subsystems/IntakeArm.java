// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class IntakeArm extends SubsystemBase {
  TalonFX arm;

  TalonFXConfiguration armConfig;

  public boolean armOut;

  double outPosition, inPosition;
  public double intakeAdjust;

  Debouncer debouncer;

  SlewRateLimiter intakeRateLimiter;
  double rateLimitedPosition;

  /** Creates a new IntakeArm. */
  public IntakeArm() {
    arm = new TalonFX(MotorIDs.INTAKE_ARM, new CANBus("1912CANivore"));

    armConfig = new TalonFXConfiguration();
    armConfig.Slot0.kS = 0.3;
    armConfig.Slot0.kP = 0.7;
    armConfig.Slot0.kI = 0;
    armConfig.Slot0.kD = 0;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    armConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    // UPPER LIMIT //
    /*armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    // LOWER LIMIT //
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 4.2;*/

    outPosition = -11.5;
    inPosition = 0;
    intakeAdjust = 0;

    arm.getConfigurator().apply(armConfig);
    arm.setPosition(0); 

    armOut = false;

    debouncer = new Debouncer(0.5);

    intakeRateLimiter = new SlewRateLimiter(1);
    rateLimitedPosition = 0;

    armIn();
  }

  @Override
  public void periodic() {
    //rateLimitedPosition = intakeRateLimiter.calculate(arm.getClosedLoopReference().getValueAsDouble());
    setArmPosition(armOut ? outPosition : inPosition);

    SmartDashboard.putNumber("intake arm adjust", intakeAdjust);
    SmartDashboard.putNumber("rate limited intake position", rateLimitedPosition);
    // This method will be called once per scheduler run
  }

  public void setArmPosition(double position) {
    final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    arm.setControl(request.withPosition(position).withEnableFOC(true));
  }

  public void armIn() {
    armOut = false;
    //setArmPosition(rateLimitedPosition);
  }

  public void armOut() {
    armOut = true;
    //setArmPosition(outPosition);
  }

  public void armInSlow() {
    armOut = false;
    setArmPosition(rateLimitedPosition);
  }

  public void armWiggle() {
    if (debouncer.calculate(armOut)) {
      armIn();
    } else {
      armOut();
    }
  }

  public boolean armInPosition() {
    return (Math.abs(arm.getClosedLoopError().getValueAsDouble()) < 5);
  }
}
