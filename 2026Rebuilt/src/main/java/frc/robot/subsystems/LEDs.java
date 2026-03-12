// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  CANdle candle;

  int matchPeriod;
  String alliance;
  // 1 - AUTO
  // 2 - TRANSITION SHIFT
  // 3 - SHIFT 1
  // 4 - SHIFT 2
  // 5 - SHIFT 3
  // 6 - SHIFT 4
  // 7 - END GAME
  /** Creates a new LEDs. */
  public LEDs() {
    candle = new CANdle(9, new CANBus("1912CANivore"));

    matchPeriod = 1;
    alliance = "B";
  }

  @Override
  public void periodic() {
    if (DriverStation.isAutonomous()) {
      matchPeriod = 1;
    } else if (DriverStation.getMatchTime() > 130) {
      matchPeriod = 2;
    } else if (DriverStation.getMatchTime() > 30) {
      matchPeriod = 6 - Math.floorDiv((int) DriverStation.getMatchTime() - 30, 25);
    } else {
      matchPeriod = 7;
    }

    if (DriverStation.isDisabled() && DriverStation.isDSAttached()) {
      alliance = getAlliance();
    }
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

  public int getMatchPeriod() {
    return matchPeriod;
  }

  public String getAutoVictor() {
    return DriverStation.getGameSpecificMessage();
  }

  public String getAutoLoser() {
    return (getAutoVictor() == "B" ? "R" : "B");
  }

  public String getActiveHub() {
    return (getMatchPeriod() % 2 == 0 ? getAutoVictor() : getAutoLoser());
  }

  public String getAlliance() {
    return (DriverStation.getAlliance().get() == Alliance.Blue ? "B" : "R"); 
  }
}
