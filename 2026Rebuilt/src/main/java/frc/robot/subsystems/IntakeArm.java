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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class IntakeArm extends SubsystemBase {
  TalonFX arm;

  TalonFXConfiguration armConfig;

  public boolean armOut;

  double outPosition, inPosition;
  public double intakeAdjust;

  /** Creates a new IntakeArm. */
  public IntakeArm() {
    arm = new TalonFX(MotorIDs.INTAKE_ARM, new CANBus("1912CANivore"));

    armConfig = new TalonFXConfiguration();
    armConfig.Slot0.kS = 0.1;
    armConfig.Slot0.kP = 0.55;
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

    outPosition = -12;
    inPosition = 0;
    intakeAdjust = 0;

    arm.getConfigurator().apply(armConfig);
    arm.setPosition(0); 

    armOut = false;

    armIn();
  }

  @Override
  public void periodic() {
    setArmPosition(armOut ? outPosition + intakeAdjust : inPosition);

    SmartDashboard.putNumber("intake arm adjust", intakeAdjust);
    // This method will be called once per scheduler run
  }

  public void setArmPosition(double position) {
    final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    arm.setControl(request.withPosition(position).withEnableFOC(true));
  }

  public void armOut() {
    armOut = true;
    setArmPosition(outPosition);
  }

  public void armIn() {
    armOut = false;
    setArmPosition(inPosition);
  }

  public void armWiggle() {
    setArmPosition(0.18);
  }

  public boolean armInPosition() {
    return (Math.abs(arm.getClosedLoopError().getValueAsDouble()) < 5);
  }
}
