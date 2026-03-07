// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class Spindexer extends SubsystemBase {
  TalonFX spindexer;
  TalonFXConfiguration spindexerConfig;
  /** Creates a new Spindexer. */
  public Spindexer() {
    spindexer = new TalonFX(MotorIDs.SPINDEXER, new CANBus("1912CANivore"));
    spindexerConfig = new TalonFXConfiguration();

    spindexerConfig.Slot0.kS = 0.2;
    spindexerConfig.Slot0.kV = 0.1;
    spindexerConfig.Slot0.kP = 0.3;
    spindexerConfig.Slot0.kI = 0;
    spindexerConfig.Slot0.kD = 0;
    spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    spindexerConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    spindexerConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

    spindexer.getConfigurator().apply(spindexerConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the speed of the spindexer to a desired speed.
   * @param speed The speed to set the PID to, in rotations per second
   */
  public void setSpeed(double speed) {
    final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    spindexer.setControl(request.withVelocity(speed));
  }

  /**
   * Gets whether or not the spindexer speed is within a certain error limit of its target speed.
   * @return True if the shooter is within in the limit, false if it isn't
   */
  public boolean spindexerAtSpeed() {
    return (spindexer.getClosedLoopError().getValueAsDouble() < 20);
  }

  /**
   * Turns the spindexer off.
   */
  public void spindexerOff() {
    spindexer.set(0);
  }
}
