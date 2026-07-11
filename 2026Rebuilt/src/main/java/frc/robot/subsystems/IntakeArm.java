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
  public TalonFX left_arm;
  public TalonFX right_arm;

  TalonFXConfiguration armConfig;
  TalonFXConfiguration armConfig_left;
  TalonFXConfiguration armConfig_right;

  public boolean armOut, armPulling, armIsStalled;

  double outPosition, inPosition;
  public double intakeAdjust;

  Debouncer wiggleDebouncer;
  Debouncer stallDebouncer;

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
    armConfig.Slot0.kS = 1;
    armConfig.Slot0.kV = 0;
    armConfig.Slot0.kA = 0;
    armConfig.Slot0.kP = 1.2;
    armConfig.Slot0.kI = 0;
    armConfig.Slot0.kD = 0;
    armConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    armConfig.Slot1.kS = 1.84;
    armConfig.Slot1.kV = 0.073;
    armConfig.Slot1.kA = 0.0122;
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

    outPosition = -9.4;
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

    wiggleDebouncer = new Debouncer(0.5);
    stallDebouncer = new Debouncer(1.5);

    rateLimitedPosition = 0;



    timer = new Timer();
  }

  @Override
  public void periodic() {
    //rateLimitedPosition = intakeRateLimiter.calculate(arm.getClosedLoopReference().getValueAsDouble());
    if (!armPulling) {
      check_arm_stall(left_arm);
      check_arm_stall(right_arm);

      if (stallDebouncer.calculate(armIsStalled)) {
        stopArmIfStalling(right_arm);
        stopArmIfStalling(left_arm);
      } else {
        double newPosition = armOut ? outPosition + intakeAdjust : inPosition;
        setArmPosition(right_arm, newPosition);
        setArmPosition(left_arm, newPosition);
      }
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
    if (arm_stall(arm)) {
      armIsStalled = true;
    }
  }

  public static void offsetIn(TalonFX arm){
    arm.setPosition(arm.getPosition().getValueAsDouble() - .05);
  }

  public static void offsetOut(TalonFX arm){
    arm.setPosition(arm.getPosition().getValueAsDouble() + .05);
  }

  public void leftOffsetIn() {
    offsetIn(left_arm);
  }

  public void leftOffsetOut() {
    offsetOut(left_arm);
  }

  public void rightOffsetIn() {
    offsetIn(right_arm);
  }

  public void rightOffsetOut() {
    offsetOut(right_arm);
  }

  public void stopArmIfStalling(TalonFX arm) {
    arm.setPosition(arm.getPosition().getValueAsDouble());
  }


  public void setArmPosition(TalonFX arm, double position) {
    final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    arm.setControl(request.withPosition(position).withEnableFOC(true));
  }

  public void armIn() {
    armOut = false;
    resetStallState();
  }

  public void armOut() {
    armOut = true;
    resetStallState();
  }

  // why this magic number? constantize / constantine / constantinople
  public void armInSlow() {
    armPulling = true;
    resetStallState();
    final VelocityVoltage requestl = new VelocityVoltage(0).withSlot(1);
    left_arm.setControl(requestl.withVelocity(10).withEnableFOC(true));
    final VelocityVoltage requestr = new VelocityVoltage(0).withSlot(1);
    right_arm.setControl(requestr.withVelocity(10).withEnableFOC(true));
  }

  public void resetStallState() {
    armIsStalled = false;
  }

  public void armWiggle() {
    if (wiggleDebouncer.calculate(armOut)) {
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
    resetStallState();
    armConfig_left.MotionMagic.MotionMagicCruiseVelocity = 6;
    armConfig_right.MotionMagic.MotionMagicCruiseVelocity = 6;
    left_arm.getConfigurator().apply(armConfig_left);
    right_arm.getConfigurator().apply(armConfig_right);
  }

  public void intakeConfigRegular() {
    resetStallState();
    armConfig_left.MotionMagic.MotionMagicCruiseVelocity = 800;
    armConfig_right.MotionMagic.MotionMagicCruiseVelocity = 800;
    left_arm.getConfigurator().apply(armConfig_left);
    right_arm.getConfigurator().apply(armConfig_right);
  }
}
