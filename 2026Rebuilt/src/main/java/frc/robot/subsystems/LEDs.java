// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class LEDs extends SubsystemBase {
  DriveTrain driveTrain;

  int matchPeriod;
  String alliance;

  double distanceFromStart;

  Integer[] allHubsActiveArray = {1, 2, 7};

  List<Integer> allHubsActive = Arrays.asList(allHubsActiveArray);

  // BFR - what is this for?
  // 1 - AUTO
  // 2 - TRANSITION SHIFT
  // 3 - SHIFT 1
  // 4 - SHIFT 2
  // 5 - SHIFT 3
  // 6 - SHIFT 4
  // 7 - END GAME
  /** Creates a new LEDs. */
  public LEDs(DriveTrain dt) {
    driveTrain = dt;

    matchPeriod = 1;
    alliance = "B";

    distanceFromStart = 0;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDSAttached()) {
      if (DriverStation.isAutonomous()) {
        matchPeriod = 1;
      } else if (DriverStation.getMatchTime() > 130) {
        matchPeriod = 2;
      } else if (DriverStation.getMatchTime() > 30) {
        matchPeriod = 6 - Math.floorDiv((int) DriverStation.getMatchTime() - 30, 25);
      } else {
        matchPeriod = 7;
      }

      if (DriverStation.isDisabled()) {
        alliance = getAlliance();
      }

      SmartDashboard.putBoolean("Is hub active?", (getActiveHub() == getAlliance()) || allHubsActive.contains(matchPeriod));
      SmartDashboard.putNumber("Current Shift", getMatchPeriod());
      SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }
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
}
