// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIDs;
import frc.robot.Constants.DriveConstants;

public class LEDs extends SubsystemBase {
  CANdle candle;
  CANdleConfiguration config;

  DriveTrain driveTrain;

  int matchPeriod;
  String alliance;

  double distanceFromStart;
  // 1 - AUTO
  // 2 - TRANSITION SHIFT
  // 3 - SHIFT 1
  // 4 - SHIFT 2
  // 5 - SHIFT 3
  // 6 - SHIFT 4
  // 7 - END GAME
  /** Creates a new LEDs. */
  public LEDs(DriveTrain dt) {
    candle = new CANdle(9, "1912CANivore");
    config = new CANdleConfiguration();
    config.stripType = LEDStripType.GRB;
    candle.configAllSettings(config);
    /*config = new CANdleConfiguration();
    config.LED.BrightnessScalar = 1;
    config.LED.StripType = StripTypeValue.RGB;
    candle.getConfigurator().apply(config);*/

    driveTrain = dt;

    matchPeriod = 1;
    alliance = "B";

    setBlueStatic();

    distanceFromStart = 0;

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

    /*if (DriverStation.isDisabled()) {
      if (isPoseWithinTranslationError() && isPoseWithinRotationError(driveTrain.getPose())) {
        setGreenStatic();
      } else if (isPoseWithinTranslationMediumError() && isPoseWithinRotationError(driveTrain.getPose())) {
        setOrangeStatic();
      } else {
        setRedStatic();
      }
    }*/

    //distanceFromStart = driveTrain.getPose().getTranslation().getDistance(driveTrain.autoChooser.getSelected().getStartingPose().getTranslation());

    SmartDashboard.putNumber("distance from auto start", distanceFromStart);
    // This method will be called once per scheduler run
  }

  public void setBlueStatic() {
    /*final SolidColor request = new SolidColor(0, 380);
    candle.setControl(request.withColor(new RGBWColor(0, 140, 220)));*/
    candle.clearAnimation(0);
    candle.setLEDs(0, 0, 255);
  }

  public void setOrangeStatic() {
    /*final SolidColor request = new SolidColor(0, 380);
    candle.setControl(request.withColor(new RGBWColor(0, 140, 220)));*/
    candle.clearAnimation(0);
    candle.setLEDs(230, 40, 0);
  }

  public void setOrangeFlashing() {
    //candle.setControl(request.withColor(new RGBWColor(230, 40, 0)));
    candle.animate(new ColorFlowAnimation(230, 40, 0, 255, 1, 300, Direction.Forward));
  }

  public void setRedStatic() {
    /*final SolidColor request = new SolidColor(0, 380);
    candle.setControl(request.withColor(new RGBWColor(230, 10, 10)));*/
    candle.clearAnimation(0);
    candle.setLEDs(230, 40, 0);
  }

  public void setGreenStatic() {
    /*final SolidColor request = new SolidColor(0, 380);
    candle.setControl(request.withColor(new RGBWColor(10, 230, 20)));*/
    candle.clearAnimation(0);
    candle.setLEDs(10, 230, 20);
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

  public boolean isPoseWithinTranslationError() {
    return (Math.abs(distanceFromStart) < DriveConstants.AUTO_TRANSLATION_ERROR);
  }

  public boolean isPoseWithinTranslationMediumError() {
    return (Math.abs(distanceFromStart) < DriveConstants.AUTO_TRANSLATION_MEDIUM_ERROR);
  }

  /*public boolean isPoseWithinRotationError(Pose2d pose) {
    return (Math.abs(pose.getRotation().getDegrees() - driveTrain.autoChooser.getSelected().getStartingPose().getRotation().getDegrees()) < DriveConstants.AUTO_ROTATION_ERROR);
  }*/

}
