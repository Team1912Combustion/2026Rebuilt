// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class IntakeArm extends SubsystemBase {
  TalonFX left_arm;
  TalonFX right_arm;

  TalonFXConfiguration armConfig;
  TalonFXConfiguration armConfig_left;
  TalonFXConfiguration armConfig_right;

  public boolean armOut, armPulling;

  double outPosition, inPosition;
  public double intakeAdjust;

  Debouncer debouncer;

  SlewRateLimiter intakeRateLimiter;
  double rateLimitedPosition;

  Timer timer;

  double stall_current = 1.;
  double stall_velocity = 1.;

  /** Creates a new IntakeArm. */
  public IntakeArm() {
    left_arm = new TalonFX(MotorIDs.INTAKE_ARM_LEFT, new CANBus("1912CANivore"));
    right_arm = new TalonFX(MotorIDs.INTAKE_ARM_RIGHT, new CANBus("1912CANivore"));

    armConfig = new TalonFXConfiguration();
    armConfig.Slot0.kS = 0.3;
    armConfig.Slot0.kV = 0;
    armConfig.Slot0.kA = 0;
    armConfig.Slot0.kP = 0.7;
    armConfig.Slot0.kI = 0;
    armConfig.Slot0.kD = 0;
    armConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    armConfig.Slot1.kS = 1.5;
    armConfig.Slot1.kV = 0.06;
    armConfig.Slot1.kA = 0.01;
    armConfig.Slot1.kP = 0.3;
    armConfig.Slot1.kI = 0;
    armConfig.Slot1.kD = 0;
    armConfig.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // UPPER LIMIT //
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    // LOWER LIMIT //
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -11.5;

    armConfig.MotionMagic.MotionMagicCruiseVelocity = 800;
    armConfig.MotionMagic.MotionMagicAcceleration = 800;

    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.CurrentLimits.SupplyCurrentLimit = 40;

    outPosition = -11.5;
    inPosition = 0;
    intakeAdjust = 0;

    armConfig_left = armConfig.clone();
    left_arm.getConfigurator().apply(armConfig_left);
    left_arm.setPosition(0); 

    armConfig_right = armConfig.clone()
      .withMotorOutput(
        armConfig.MotorOutput.clone().withInverted(InvertedValue.Clockwise_Positive)
      );

    right_arm.getConfigurator().apply(armConfig_right);
    right_arm.setPosition(0); 

    armOut = false;
    armPulling = false;

    debouncer = new Debouncer(0.5);

    rateLimitedPosition = 0;

    armIn();

    timer = new Timer();
  }

  @Override
  public void periodic() {
    //rateLimitedPosition = intakeRateLimiter.calculate(arm.getClosedLoopReference().getValueAsDouble());
    if (!armPulling) {
      //check_arm_stall(left_arm);
      //check_arm_stall(right_arm);
      setArmPosition(armOut ? outPosition + intakeAdjust : inPosition);
    }

    SmartDashboard.putNumber("left arm current",
        left_arm.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("right arm current",
        right_arm.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("left arm velocity",
        left_arm.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("right arm velocity",
        right_arm.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("right arm stall",
        arm_stall(right_arm));
    SmartDashboard.putBoolean("left arm stall",
        arm_stall(left_arm));
    SmartDashboard.putNumber("left arm position",
        left_arm.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("right arm position",
        right_arm.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("intake arm adjust", intakeAdjust);
    SmartDashboard.putNumber("rate limited intake position", rateLimitedPosition);
    // This method will be called once per scheduler run
  }

  public boolean arm_stall(TalonFX arm) {
    double current = arm.getStatorCurrent().getValueAsDouble();
    double velocity = arm.getVelocity().getValueAsDouble();
    if (current > stall_current && Math.abs(velocity) < stall_velocity) return true;
    return false;
  }

  public void check_arm_stall(TalonFX arm) {
    double current = arm.getStatorCurrent().getValueAsDouble();
    double velocity = arm.getVelocity().getValueAsDouble();
    if (current > stall_current && Math.abs(velocity) < stall_velocity) {
      if (armOut) arm.setPosition(outPosition);
      if (!armOut) arm.setPosition(0.);
    }
  }

  public void setArmPosition(double position) {
    final PositionVoltage requestl = new PositionVoltage(0).withSlot(0);
    left_arm.setControl(requestl.withPosition(position).withEnableFOC(true));
    final PositionVoltage requestr = new PositionVoltage(0).withSlot(0);
    right_arm.setControl(requestr.withPosition(position).withEnableFOC(true));
  }

  public void armIn() {
    armOut = false;
  }

  public void armOut() {
    armOut = true;
  }

  public void armInSlow() {
    armPulling = true;
    final VelocityVoltage requestl = new VelocityVoltage(0).withSlot(1);
    left_arm.setControl(requestl.withVelocity(10).withEnableFOC(true));
    final VelocityVoltage requestr = new VelocityVoltage(0).withSlot(1);
    right_arm.setControl(requestr.withVelocity(10).withEnableFOC(true));
  }

  public void armWiggle() {
    if (debouncer.calculate(armOut)) {
      armIn();
    } else {
      armOut();
    }
  }

  public boolean leftInPosition() {
    return (Math.abs(left_arm.getClosedLoopError().getValueAsDouble()) < 5);
  }
  public boolean rightInPosition() {
    return (Math.abs(right_arm.getClosedLoopError().getValueAsDouble()) < 5);
  }

  public void intakeConfigSlow() {
    armConfig_left.MotionMagic.MotionMagicCruiseVelocity = 6;
    armConfig_right.MotionMagic.MotionMagicCruiseVelocity = 6;
    left_arm.getConfigurator().apply(armConfig_left);
    right_arm.getConfigurator().apply(armConfig_right);
  }

  public void intakeConfigRegular() {
    armConfig_left.MotionMagic.MotionMagicCruiseVelocity = 800;
    armConfig_right.MotionMagic.MotionMagicCruiseVelocity = 800;
    left_arm.getConfigurator().apply(armConfig_left);
    right_arm.getConfigurator().apply(armConfig_right);
  }
}
