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

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class Floor extends SubsystemBase {
  TalonFX floor1, floor2;
  TalonFXConfiguration floorConfig;
  /** Creates a new Floor. */
  public Floor() {
    floor1 = new TalonFX(MotorIDs.FLOOR_1, new CANBus("1912CANivore"));
    floor2 = new TalonFX(MotorIDs.FLOOR_2, new CANBus("1912CANivore"));
    floorConfig = new TalonFXConfiguration();

    floorConfig.Slot0.kS = 0.2;
    floorConfig.Slot0.kV = 0.1;
    floorConfig.Slot0.kP = 0.25;
    floorConfig.Slot0.kI = 0;
    floorConfig.Slot0.kD = 0;
    floorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    floorConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    floorConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

    floor1.getConfigurator().apply(floorConfig);
    floor2.getConfigurator().apply(floorConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the speed of the floor to a desired speed.
   * @param speed The speed to set the PID to, in rotations per second
   */
  public void setSpeed(double speed) {
    final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    floor1.setControl(request.withVelocity(speed));
    final Follower followerRequest = new Follower(0, MotorAlignmentValue.Aligned);
    floor2.setControl(followerRequest.withLeaderID(MotorIDs.FLOOR_2).withMotorAlignment(MotorAlignmentValue.Aligned));
  }

  /**
   * Gets whether or not the floor speed is within a certain error limit of its target speed.
   * @return True if the floor is within in the limit, false if it isn't
   */
  public boolean floorAtSpeed() {
    return (floor1.getClosedLoopError().getValueAsDouble() < 20);
  }

  /**
   * Turns the floor off.
   */
  public void floorOff() {
    floor1.set(0);
  }
}
