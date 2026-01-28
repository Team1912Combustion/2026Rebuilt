// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorIDs {

    public static final int TURRET = 1;
    public static final int TURRET_HOOD = 2;

    public static final int SHOOTER = 3;
    public static final int KICKER = 4;
    public static final int SPINDEXER = 5;

    public static final int SERVO_HUB = 6;

    public static final int INTAKE_ROLLERS = 7;
    public static final int INTAKE_ARM = 8;

  }

  public static class SensorIDs {
    public static final int GYRO = 1;
    
    public static final int TURRET_BEAMBREAK = 1;
    
    public static final int TURRET_LEFT_LIMIT_SWITCH = 1;
    public static final int TURRET_RIGHT_LIMIT_SWITCH = 1;

  }

  public static class FieldZoneConstants {

    // TL is Top Left, BR is Bottom Right

    // BLUE ZONES //
    public static final Translation2d BLUE_DEPOT_ZONE_TL = new Translation2d();
    public static final Translation2d BLUE_DEPOT_ZONE_BR = new Translation2d();

    public static final Translation2d BLUE_OUTPOST_ZONE_TL = new Translation2d();
    public static final Translation2d BLUE_OUTPOST_ZONE_BR = new Translation2d();

    // BLUE SHOT POINTS //
    public static final Translation2d BLUE_HUB_SHOT_POINT = new Translation2d();
    public static final Translation2d BLUE_DEPOT_SHOT_POINT = new Translation2d();
    public static final Translation2d BLUE_OUTPOST_SHOT_POINT = new Translation2d();

    // RED ZONES // 
    public static final Translation2d RED_DEPOT_ZONE_TL = new Translation2d();
    public static final Translation2d RED_DEPOT_ZONE_BR = new Translation2d();

    public static final Translation2d RED_OUTPOST_ZONE_TL = new Translation2d();
    public static final Translation2d RED_OUTPOST_ZONE_BR = new Translation2d();

    // RED SHOT POINTS //
    public static final Translation2d RED_HUB_SHOT_POINT = new Translation2d();
    public static final Translation2d RED_DEPOT_SHOT_POINT = new Translation2d();
    public static final Translation2d RED_OUTPOST_SHOT_POINT = new Translation2d();

    // NEUTRAL ZONES //
    public static final Translation2d NEUTRAL_TOP_ZONE_TL = new Translation2d();
    public static final Translation2d NEUTRAL_TOP_ZONE_BR = new Translation2d();

    public static final Translation2d NEUTRAL_BOTTOM_ZONE_TL = new Translation2d();
    public static final Translation2d NEUTRAL_BOTTOM_ZONE_BR = new Translation2d();

    // NEUTRAL SHOT POINTS //
    public static final Translation2d NEUTRAL_TOP_ZONE_BLUE_SHOT_POINT = new Translation2d();
    public static final Translation2d NEUTRAL_TOP_ZONE_RED_SHOT_POINT = new Translation2d();
    public static final Translation2d NEUTRAL_BOTTOM_ZONE_BLUE_SHOT_POINT = new Translation2d();
    public static final Translation2d NEUTRAL_BOTTOM_ZONE_RED_SHOT_POINT = new Translation2d();

    // CONSTRUCTED FIELD ZONES //
    public static final FieldZone BLUE_DEPOT_ZONE = new FieldZone(BLUE_DEPOT_ZONE_TL, BLUE_DEPOT_ZONE_BR);
    public static final FieldZone BLUE_OUTPOST_ZONE = new FieldZone(BLUE_OUTPOST_ZONE_TL, BLUE_OUTPOST_ZONE_BR);
    public static final FieldZone RED_DEPOT_ZONE = new FieldZone(RED_DEPOT_ZONE_TL, RED_DEPOT_ZONE_BR);
    public static final FieldZone RED_OUTPOST_ZONE = new FieldZone(RED_OUTPOST_ZONE_TL, RED_OUTPOST_ZONE_BR);
    public static final FieldZone NEUTRAL_TOP_ZONE = new FieldZone(NEUTRAL_TOP_ZONE_TL, NEUTRAL_TOP_ZONE_BR);
    public static final FieldZone NEUTRAL_BOTTOM_ZONE = new FieldZone(NEUTRAL_BOTTOM_ZONE_TL, NEUTRAL_BOTTOM_ZONE_BR);

  }

  public static class TurretConstants {

    public static final Pose2d TURRET_POSE_OFFSET = new Pose2d(new Translation2d(0, 0), new Rotation2d());

    public static final double[][] DISTANCES = { {0.0, 1.5}, {1.51, 3.0}, {3.01, 4.5}, {4.51, 6.0}, {6.01, 7.5}, {7.51, 9.0} };
    public static final double[] SPEEDS = { 500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0 };
    public static final double[] HIGH_HOOD_ANGLES = { 500.0, 1000.0, 1500.0, 1750.0, 2000.0, 2500.0 };
    public static final double[] LOW_HOOD_ANGLES = { 100.0, 200.0, 300.0, 400.0, 500.0, 600.0 };

    public static final double X_OFFSET_THRESHHOLD = 0.2;

    public static final double SHOOTER_WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double SHOOTER_WHEEL_CIRCUMFERENCE = SHOOTER_WHEEL_DIAMETER * Math.PI;

  }

  public static class VisionConstants {

    public static final double TARGET_AREA_THRESHHOLD = 0.15;
    public static final double TOTAL_TARGET_AREA_THRESHHOLD = 0.25;

    public static AprilTagFieldLayout aprilTagLayout = 
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  }

  public static class DeviceIDs {

    public static final class FRONT_LEFT {
      public static final int drive = 2;
      public static final int angle = 1;
      public static final int cancoder = 1;
      public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(drive, angle, cancoder, Rotation2d.fromDegrees(0));
    }

    public static final class FRONT_RIGHT {
      public static final int drive = 4;
      public static final int angle = 3;
      public static final int cancoder = 2;
      public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(drive, angle, cancoder, Rotation2d.fromDegrees(0));
    }

    public static final class REAR_LEFT {
      public static final int drive = 6;
      public static final int angle = 5;
      public static final int cancoder = 3;
      public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(drive, angle, cancoder, Rotation2d.fromDegrees(0));
    }

    public static final class REAR_RIGHT {
      public static final int drive = 8;
      public static final int angle = 7;
      public static final int cancoder = 4;
      public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(drive, angle, cancoder, Rotation2d.fromDegrees(0));
    }
  }
  public static final class ModuleConstants {

    public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

    public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 10000.;
    public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECONDSQUARED = 10000.; 

    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    public static final double WHEEL_CIRCRUMFERENCE_METERS = (WHEEL_DIAMETER_METERS * Math.PI);
    public static final double WHEEL_GEAR_RATIO = 6.75;
    public static final double DRIVE_ENCODER_DISTANCE_PER_ROTATION =
      (WHEEL_DIAMETER_METERS * Math.PI) / WHEEL_GEAR_RATIO;

    public static final double kTurnGearRatio = 1.; // 150./7.;

    public static final double kPModuleDriveController = 0.15;
    public static final double kIModuleDriveController = 0.0;
    public static final double kDModuleDriveController = 0.0;

    public static final double kPModuleTurningController = 50.0;
    public static final double kIModuleTurningController = 0.0;
    public static final double kDModuleTurningController = 0.0;

    public static final double OPENLOOPRAMP = 0.25;
    public static final double CLOSEDLOOPRAMP = 0.0;

    public static final int ANGLECURRENTLIMIT = 30;
    public static final int ANGLECURRENTTHRESHOLD = 50;
    public static final double ANGLECURRENTTHRESHOLDTIME = 0.1;
    public static final boolean ANGLEENABLECURRENTLIMIT = true;

    public static final int DRIVECURRENTLIMIT = 40;
    public static final int DRIVECURRENTTHRESHOLD = 50;
    public static final double DRIVECURRENTTHRESHOLDTIME = 0.1;
    public static final boolean DRIVEENABLECURRENTLIMIT = true;

    public static final InvertedValue ANGLEMOTORINVERT = chosenModule.angleMotorInvert;
    public static final InvertedValue DRIVEMOTORINVERT = chosenModule.driveMotorInvert;

    public static final SensorDirectionValue CANCODERINVERT = chosenModule.cancoderInvert;

  }

  public static class DriveConstants {
    public static final double DRIVE_SPEED = 1.0;
    public static final double RAMP_TIME = 0.07;
    public static final double TRACK_WIDTH = Units.inchesToMeters(29);
    public static final double WHEEL_BASE = Units.inchesToMeters(29);
    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),   //FL
          new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),  //FR
          new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),  //RL
          new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); //RR

    public static final double S_VOLTS = 1;
    public static final double V_VOLT_SECONDS_PER_METER = 0.8;
    public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.15;
      
    public static final double MAX_SPEED_METERS_PER_SECOND = 5;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;
  }
}