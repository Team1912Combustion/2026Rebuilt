// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  CANdle candle;
  /** Creates a new LEDs. */
  public LEDs() {
    candle = new CANdle(0, new CANBus("1912CANivore"));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setBlueStatic() {
    final SolidColor request = new SolidColor(0, 400);
    candle.setControl(request.withColor(new RGBWColor(0, 140, 220)));
  }

  public void setOrangeStatic() {
    final SolidColor request = new SolidColor(0, 400);
    candle.setControl(request.withColor(new RGBWColor(230, 40, 0)));
  }

  public void setOrangeFlashing() {
    final ColorFlowAnimation request = new ColorFlowAnimation(0, 400);
    candle.setControl(request.withColor(new RGBWColor(230, 40, 0)));
  }
}
