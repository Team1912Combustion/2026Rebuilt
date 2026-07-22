// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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
  public static class VisionConstants {
    public static final double TARGET_AREA_THRESHHOLD = 0.05;
    public static final double TOTAL_TARGET_AREA_THRESHHOLD = 0.1;
    public static final AprilTagFields APRILTAG_FIELD = AprilTagFields.k2026RebuiltWelded;
  }

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class CANConstants {
    public static final CANBUS_NAME = "1912CANivore";
  }

  public static class MotorIDs {

    public static final int HOOD = 9;

    public static final int SHOOTER_1 = 10;
    public static final int SHOOTER_2 = 11;
    public static final int SHOOTER_3 = 12;
    public static final int SHOOTER_4 = 13;

    public static final int KICKER = 14;

    public static final int FLOOR_1 = 15;
    public static final int FLOOR_2 = 16;

    public static final int INTAKE_ROLLERS_LEFT = 17;
    public static final int INTAKE_ROLLERS_RIGHT = 18;
    public static final int INTAKE_ARM_LEFT = 19;
    public static final int INTAKE_ARM_RIGHT = 20;

  }

  public static class SensorIDs {
    public static final int GYRO = 1;
  }

  public static class FieldZoneConstants {

    public static final double FIELD_WIDTH = 8.07;
    public static final double FIELD_LENGTH = 16.54;
    public static final double ALLIANCE_ZONE_LENGTH = 4.625;

    // TL is Top Left, BR is Bottom Right

    // BLUE ZONES //
    public static final Translation2d BLUE_DEPOT_ZONE_TL = new Translation2d(0, FIELD_WIDTH);
    public static final Translation2d BLUE_DEPOT_ZONE_BR = new Translation2d(ALLIANCE_ZONE_LENGTH, FIELD_WIDTH / 2);

    public static final Translation2d BLUE_OUTPOST_ZONE_TL = new Translation2d(0, FIELD_WIDTH / 2);
    public static final Translation2d BLUE_OUTPOST_ZONE_BR = new Translation2d(ALLIANCE_ZONE_LENGTH, 0);

    // BLUE SHOT POINTS //
    public static final Translation2d BLUE_HUB_SHOT_POINT = new Translation2d(ALLIANCE_ZONE_LENGTH, FIELD_WIDTH / 2);
    public static final Translation2d BLUE_DEPOT_SHOT_POINT = new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH * (5.0 / 8.0));
    public static final Translation2d BLUE_OUTPOST_SHOT_POINT = new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH * (3.0 / 8.0));

    // RED ZONES // 
    public static final Translation2d RED_DEPOT_ZONE_TL = new Translation2d(FIELD_LENGTH - ALLIANCE_ZONE_LENGTH, FIELD_WIDTH / 2);
    public static final Translation2d RED_DEPOT_ZONE_BR = new Translation2d(FIELD_LENGTH, 0);

    public static final Translation2d RED_OUTPOST_ZONE_TL = new Translation2d(FIELD_LENGTH - ALLIANCE_ZONE_LENGTH, FIELD_WIDTH);
    public static final Translation2d RED_OUTPOST_ZONE_BR = new Translation2d(FIELD_LENGTH, FIELD_WIDTH / 2);

    // RED SHOT POINTS //
    public static final Translation2d RED_HUB_SHOT_POINT = new Translation2d(FIELD_LENGTH - ALLIANCE_ZONE_LENGTH, FIELD_WIDTH / 2);
    public static final Translation2d RED_DEPOT_SHOT_POINT = new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH * (3.0 / 8.0));
    public static final Translation2d RED_OUTPOST_SHOT_POINT = new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH * (5.0 / 8.0));

    // NEUTRAL ZONES //
    public static final Translation2d NEUTRAL_TOP_ZONE_TL = new Translation2d(ALLIANCE_ZONE_LENGTH, FIELD_WIDTH);
    public static final Translation2d NEUTRAL_TOP_ZONE_BR = new Translation2d(FIELD_LENGTH - ALLIANCE_ZONE_LENGTH, FIELD_WIDTH / 2);

    public static final Translation2d NEUTRAL_BOTTOM_ZONE_TL = new Translation2d(ALLIANCE_ZONE_LENGTH, FIELD_WIDTH / 2);
    public static final Translation2d NEUTRAL_BOTTOM_ZONE_BR = new Translation2d(FIELD_LENGTH - ALLIANCE_ZONE_LENGTH, 0);

    // NEUTRAL SHOT POINTS //
    public static final Translation2d NEUTRAL_TOP_ZONE_BLUE_SHOT_POINT = new Translation2d(ALLIANCE_ZONE_LENGTH / 2, FIELD_WIDTH * (3.0 / 4.0));
    public static final Translation2d NEUTRAL_TOP_ZONE_RED_SHOT_POINT = new Translation2d(FIELD_LENGTH - (ALLIANCE_ZONE_LENGTH / 2), FIELD_WIDTH * (3.0 / 4.0));
    public static final Translation2d NEUTRAL_BOTTOM_ZONE_BLUE_SHOT_POINT = new Translation2d(ALLIANCE_ZONE_LENGTH / 2, FIELD_WIDTH * (1.0 / 4.0));
    public static final Translation2d NEUTRAL_BOTTOM_ZONE_RED_SHOT_POINT = new Translation2d(FIELD_LENGTH - (ALLIANCE_ZONE_LENGTH / 2), FIELD_WIDTH * (1.0 / 4.0));

    // CONSTRUCTED FIELD ZONES //
    public static final FieldZone BLUE_DEPOT_ZONE = new FieldZone(BLUE_DEPOT_ZONE_TL, BLUE_DEPOT_ZONE_BR, "BLUE DEPOT");
    public static final FieldZone BLUE_OUTPOST_ZONE = new FieldZone(BLUE_OUTPOST_ZONE_TL, BLUE_OUTPOST_ZONE_BR, "BLUE OUTPOST");
    public static final FieldZone RED_DEPOT_ZONE = new FieldZone(RED_DEPOT_ZONE_TL, RED_DEPOT_ZONE_BR, "RED DEPOT");
    public static final FieldZone RED_OUTPOST_ZONE = new FieldZone(RED_OUTPOST_ZONE_TL, RED_OUTPOST_ZONE_BR, "RED OUTPOST");
    public static final FieldZone NEUTRAL_TOP_ZONE = new FieldZone(NEUTRAL_TOP_ZONE_TL, NEUTRAL_TOP_ZONE_BR, "NEUTRAL TOP");
    public static final FieldZone NEUTRAL_BOTTOM_ZONE = new FieldZone(NEUTRAL_BOTTOM_ZONE_TL, NEUTRAL_BOTTOM_ZONE_BR, "NEUTRAL BOTTOM");

    /*
     * TRENCH ZONES
     */
    public static final double WALL_TO_TRENCH = 3.977;
    public static final double TRENCH_WIDTH = 1.285;
    public static final double TRENCH_LENGTH = 1.194;

    // CONSTRUCTED FIELD ZONES //
    public static final Translation2d BLUE_DEPOT_TRENCH_TL = new Translation2d(WALL_TO_TRENCH, FIELD_WIDTH);
    public static final Translation2d BLUE_DEPOT_TRENCH_BR = new Translation2d(WALL_TO_TRENCH + TRENCH_LENGTH, FIELD_WIDTH - TRENCH_WIDTH);

    public static final Translation2d BLUE_OUTPOST_TRENCH_TL = new Translation2d(WALL_TO_TRENCH, TRENCH_WIDTH);
    public static final Translation2d BLUE_OUTPOST_TRENCH_BR = new Translation2d(WALL_TO_TRENCH + TRENCH_LENGTH, 0);

    public static final Translation2d RED_DEPOT_TRENCH_TL = new Translation2d(FIELD_LENGTH - (WALL_TO_TRENCH + TRENCH_LENGTH), FIELD_WIDTH);
    public static final Translation2d RED_DEPOT_TRENCH_BR = new Translation2d(FIELD_LENGTH - WALL_TO_TRENCH, FIELD_WIDTH - TRENCH_WIDTH);

    public static final Translation2d RED_OUTPOST_TRENCH_TL = new Translation2d(FIELD_LENGTH - (WALL_TO_TRENCH + TRENCH_LENGTH), TRENCH_WIDTH);
    public static final Translation2d RED_OUTPOST_TRENCH_BR = new Translation2d(FIELD_LENGTH - WALL_TO_TRENCH, 0);

    public static final double ADD = 0.25;

    public static final Translation2d BLUE_DEPOT_TRENCH_TELEOP_TL = new Translation2d(WALL_TO_TRENCH - ADD, FIELD_WIDTH);
    public static final Translation2d BLUE_DEPOT_TRENCH_TELEOP_BR = new Translation2d(WALL_TO_TRENCH + TRENCH_LENGTH + ADD, FIELD_WIDTH - TRENCH_WIDTH);

    public static final Translation2d BLUE_OUTPOST_TRENCH_TELEOP_TL = new Translation2d(WALL_TO_TRENCH - ADD, TRENCH_WIDTH);
    public static final Translation2d BLUE_OUTPOST_TRENCH_TELEOP_BR = new Translation2d(WALL_TO_TRENCH + TRENCH_LENGTH + ADD, 0);

    public static final Translation2d RED_DEPOT_TRENCH_TELEOP_TL = new Translation2d(FIELD_LENGTH - (WALL_TO_TRENCH + TRENCH_LENGTH) - ADD, FIELD_WIDTH);
    public static final Translation2d RED_DEPOT_TRENCH_TELEOP_BR = new Translation2d(FIELD_LENGTH - WALL_TO_TRENCH + ADD, FIELD_WIDTH - TRENCH_WIDTH);

    public static final Translation2d RED_OUTPOST_TRENCH_TELEOP_TL = new Translation2d(FIELD_LENGTH - (WALL_TO_TRENCH + TRENCH_LENGTH) - ADD, TRENCH_WIDTH);
    public static final Translation2d RED_OUTPOST_TRENCH_TELEOP_BR = new Translation2d(FIELD_LENGTH - WALL_TO_TRENCH + ADD, 0);

    public static final FieldZone BLUE_DEPOT_TRENCH_ZONE = new FieldZone(BLUE_DEPOT_TRENCH_TL, BLUE_DEPOT_TRENCH_BR, "BLUE DEPOT TRENCH");
    public static final FieldZone BLUE_OUTPOST_TRENCH_ZONE = new FieldZone(BLUE_OUTPOST_TRENCH_TL, BLUE_OUTPOST_TRENCH_BR, "BLUE OUTPOST TRENCH");
    public static final FieldZone RED_DEPOT_TRENCH_ZONE = new FieldZone(RED_DEPOT_TRENCH_TL, RED_DEPOT_TRENCH_BR, "RED DEPOT TRENCH");
    public static final FieldZone RED_OUTPOST_TRENCH_ZONE = new FieldZone(RED_OUTPOST_TRENCH_TL, RED_OUTPOST_TRENCH_BR, "RED OUTPOST TRENCH");

    public static final FieldZone BLUE_DEPOT_TRENCH_ZONE_TELEOP = new FieldZone(BLUE_DEPOT_TRENCH_TELEOP_TL, BLUE_DEPOT_TRENCH_TELEOP_BR, "BLUE DEPOT TRENCH");
    public static final FieldZone BLUE_OUTPOST_TRENCH_ZONE_TELEOP = new FieldZone(BLUE_OUTPOST_TRENCH_TELEOP_TL, BLUE_OUTPOST_TRENCH_TELEOP_BR, "BLUE OUTPOST TRENCH");
    public static final FieldZone RED_DEPOT_TRENCH_ZONE_TELEOP = new FieldZone(RED_DEPOT_TRENCH_TELEOP_TL, RED_DEPOT_TRENCH_TELEOP_BR, "RED DEPOT TRENCH");
    public static final FieldZone RED_OUTPOST_TRENCH_ZONE_TELEOP = new FieldZone(RED_OUTPOST_TRENCH_TELEOP_TL, RED_OUTPOST_TRENCH_TELEOP_BR, "RED OUTPOST TRENCH");

  }

  public static class TurretConstants {

    public static final Translation2d TURRET_OFFSET = new Translation2d(-0.1282, -0.030932);

    public static final double TURRET_ALLOWED_ERROR = 0.5;

    public static final double[][] DISTANCES = { {0.0, 1.49}, {1.5, 2.99}, {3.0, 4.49}, {4.5, 5.99}, {6.0, 7.49}, {7.5, 8.99} };
    public static final double DELTA_DISTANCE = 1.49;
    public static final double[][] TAG_AREA = { {0.4, 0.36}, {0.35, 0.31}, {0.30, 0.26}, {0.25, 0.21}, {0.20, 0.16}, {0.15, 0.1}};
    public static final double DELTA_AREA = 0.04;
    public static final double[] SPEEDS = { 500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0 };
    public static final double[] HIGH_HOOD_ANGLES = { 500.0, 1000.0, 1500.0, 1750.0, 2000.0, 2500.0 };
    public static final double[] LOW_HOOD_ANGLES = { 100.0, 200.0, 300.0, 400.0, 500.0, 600.0 };
    public static final double[] TRAVEL_TIMES = { 0.12, 0.24, 0.36, 0.48, 0.6, 0.72 };

    public static final double X_OFFSET_THRESHHOLD = 0.2;

    public static final double SHOOTER_WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double SHOOTER_WHEEL_CIRCUMFERENCE = SHOOTER_WHEEL_DIAMETER * Math.PI;

  }

  public static class DeviceIDs {

    public static final class FRONT_LEFT {
      public static final int drive = 1;
      public static final int angle = 2;
      public static final int cancoder = 1;
      public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(drive, angle, cancoder, Rotation2d.fromDegrees(0));
    }

    public static final class FRONT_RIGHT {
      public static final int drive = 3;
      public static final int angle = 4;
      public static final int cancoder = 2;
      public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(drive, angle, cancoder, Rotation2d.fromDegrees(0));
    }

    public static final class REAR_LEFT {
      public static final int drive = 5;
      public static final int angle = 6;
      public static final int cancoder = 3;
      public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(drive, angle, cancoder, Rotation2d.fromDegrees(0));
    }

    public static final class REAR_RIGHT {
      public static final int drive = 7;
      public static final int angle = 8;
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
    public static final double TREAD_THICKNESS = 0.007557;
    public static final double FULL_WHEEL_DIAMETER = WHEEL_DIAMETER_METERS + (TREAD_THICKNESS * 2);
    public static final double WHEEL_CIRCRUMFERENCE_METERS = (FULL_WHEEL_DIAMETER * Math.PI);
    public static final double WHEEL_GEAR_RATIO = 6.75;
    public static final double DRIVE_ENCODER_DISTANCE_PER_ROTATION =
      (FULL_WHEEL_DIAMETER * Math.PI) / WHEEL_GEAR_RATIO;

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
    public static final double AUTO_TRANSLATION_ERROR = 0.1;
    public static final double AUTO_TRANSLATION_MEDIUM_ERROR = 0.2;
    public static final double AUTO_ROTATION_ERROR = 3;

    public static final double DRIVE_SPEED = 1.0;
    public static final double RAMP_TIME = 0.05;
    public static final double TRACK_WIDTH = 0.55245;
    public static final double WHEEL_BASE = 0.55245;
    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),   //FL
          new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),  //FR
          new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),  //RL
          new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); //RR

    /*
      POSSIBLE FEEDFORWARD ISSUE
    public static final double S_VOLTS = 1;
    public static final double V_VOLT_SECONDS_PER_METER = 0.8;
    public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.15;*/

    public static final double S_VOLTS = 0.2;
    public static final double V_VOLT_SECONDS_PER_METER = 2.4;
    public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.15;
      
    public static final double MAX_SPEED_METERS_PER_SECOND = 5;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;
  }
}
