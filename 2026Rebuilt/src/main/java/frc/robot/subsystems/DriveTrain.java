// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIDs;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorIDs;
import frc.robot.Constants.VisionConstants;

public class DriveTrain extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(0, DeviceIDs.FRONT_LEFT.constants);
  private final SwerveModule frontRight = new SwerveModule(1, DeviceIDs.FRONT_RIGHT.constants);
  private final SwerveModule rearLeft = new SwerveModule(2, DeviceIDs.REAR_LEFT.constants);
  private final SwerveModule rearRight = new SwerveModule(3, DeviceIDs.REAR_RIGHT.constants);

  public final Pigeon2 gyro = new Pigeon2(SensorIDs.GYRO, "1912CANivore");

  public double driveYaw;
  public double driveYawDirection;
  public double driveYawOffset;

  public boolean fieldRelative;

  LimelightFrontLeft limelightFrontLeft;
  LimelightFrontRight limelightFrontRight;
  LimelightRear limelightRear;

  MedianFilter limelightXFilter;
  MedianFilter limelightYFilter;
  MedianFilter limelightYawFilter;

  MedianFilter targetSpaceXFilter;
  MedianFilter targetSpaceYFilter;
  MedianFilter targetSpaceYawFilter;

  public SwerveDrivePoseEstimator poseEstimator;

  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.25, 0.25, Units.degreesToRadians(.1));
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1., 1., Units.degreesToRadians(5));

  public SendableChooser<Command> autoChooser;

  public double poseX, poseY, poseYaw;

  double compositeLatency;
  Pose2d compositeVisionPose;
  Pose2d compositeTargetSpacePose;

  boolean isVisionValid;
  boolean isTargetSpacePoseValid;

  SlewRateLimiter xRateLimiter, yRateLimiter;

  PIDController rotationPID, distancePID;

  public XboxController driverController;

  public final double DISTANCE_ERROR_FRACTION = 0.1;

  public boolean isAimingReef;
  public boolean isAimedReef;
  public boolean isAimingSource;
  public boolean isAimedSource;

  Integer[] reefTagArray = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  Integer[] sourceTagArray = {1, 2, 12, 13};

  List<Integer> reefTagIDs = Arrays.asList(reefTagArray);
  List<Integer> sourceTagIDs = Arrays.asList(sourceTagArray);

  Field2d field;

  boolean autoBuilderConfigured;

  RobotConfig cfg;

  /** Creates a new DriveTrain. */
  public DriveTrain(LimelightRear lr) {
    fieldRelative = true;

    driveYaw = 0;
    driveYawDirection = 0;
    driveYawOffset = 0;

    gyro.setYaw(0);

    limelightFrontLeft = new LimelightFrontLeft();
    limelightFrontRight = new LimelightFrontRight();
    limelightRear = lr;

    limelightXFilter = new MedianFilter(3);
    limelightYFilter = new MedianFilter(3);
    limelightYawFilter = new MedianFilter(3);

    targetSpaceXFilter = new MedianFilter(3);
    targetSpaceYFilter = new MedianFilter(3);
    targetSpaceYawFilter = new MedianFilter(3);

    autoChooser = new SendableChooser<>();

    poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.DRIVE_KINEMATICS, 
      getRotation2d(), 
      get_positions(), 
      new Pose2d(),
      stateStdDevs,
      visionMeasurementStdDevs
      );

    poseX = 0;
    poseY = 0;
    poseYaw = 0;

    compositeLatency = 0;
    compositeVisionPose = new Pose2d();
    compositeTargetSpacePose = new Pose2d();

    isVisionValid = false;
    isTargetSpacePoseValid = true;

    xRateLimiter = new SlewRateLimiter(1 / DriveConstants.RAMP_TIME);
    yRateLimiter = new SlewRateLimiter(1 / DriveConstants.RAMP_TIME);

    rotationPID = new PIDController(0.01, 0, 0);
    distancePID = new PIDController(0.001, 0, 0);

    driverController = new XboxController(0);

    field = new Field2d();

    try {
      cfg = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      cfg = null;
      // TODO: handle exception
    }

    AutoBuilder.configure(
      this::getPose,
      this::resetPose,
      this::getChassisSpeeds,
      this::drive, 
      new PPHolonomicDriveController(new PIDConstants(10, 0, 0), new PIDConstants(2.5, 0, 0)), 
      cfg, 
      this::flipPath, 
      this
    );

    isAimingReef = false;
    isAimedReef = false;
    isAimingSource = false;
    isAimedSource = false;

    autoBuilderConfigured = false;

    Logger.recordOutput("Pose", poseEstimator.getEstimatedPosition());

  }

  @Override
  public void periodic() {

    // set drive yaw direction
    if (flipPath()) {
      driveYawDirection = 180;
    } else if (!flipPath()) {
      driveYawDirection = 0;
    }
    
    // update drive yaw
    driveYaw = gyro.getYaw().getValueAsDouble() + driveYawOffset;
    driveYaw = MathUtil.angleModulus(Math.toRadians(driveYaw));
    driveYaw = Math.toDegrees(driveYaw);

    // update pose estimator
    poseEstimator.update(getRotation2d(), get_positions());

    // update drive yaw while disabled
    if (DriverStation.isDisabled()) {
      if(limelightFrontLeft.getTagId() > 0) {
        driveYawOffset = (limelightFrontLeft.getBotPose()[5] + driveYawDirection - gyro.getYaw().getValueAsDouble() * (flipPath() ? -1 : 1));
      } else if (limelightFrontRight.getTagId() > 0) {
        driveYawOffset = (limelightFrontRight.getBotPose()[5] + driveYawDirection - gyro.getYaw().getValueAsDouble() * (flipPath() ? -1 : 1));
      }
    }
    
    // calculate composite poses
    processFrame();
    calculateFrameTargetSpace();

    // add pose to pose estimator
    if (isVisionValid) {
      poseEstimator.addVisionMeasurement(compositeVisionPose, Timer.getFPGATimestamp() - (compositeLatency / 1000));
    }

    // update pose variables
    poseX = poseEstimator.getEstimatedPosition().getX();
    poseY = poseEstimator.getEstimatedPosition().getY();
    poseYaw = poseEstimator.getEstimatedPosition().getRotation().getDegrees();

    // update field2d object
    field.setRobotPose(poseEstimator.getEstimatedPosition());

    // print to smart dashboard
    SmartDashboard.putData(field);
    SmartDashboard.putNumber("Drive yaw", driveYaw);

    SmartDashboard.putNumber("Target space x", getPose2dTargetSpace().getX());
    SmartDashboard.putNumber("Target space y", getPose2dTargetSpace().getY());
    SmartDashboard.putNumber("Target space yaw", getPose2dTargetSpace().getRotation().getDegrees());
    SmartDashboard.putBoolean("reef tag?", isTargetSpacePoseValid);

    SmartDashboard.putData("Auto?:", autoChooser);

    // This method will be called once per scheduler run
  }

  /**
   * Takes in driver joystick values and applies them to the swerves.
   * Raw joystick values are multiplied by the motor's max speed in meters per second and then filtered throught a slew rate limiter.
   * Field relative movement is based off of the drive yaw value in degrees, independent of the pose estimator.
   * @param xSpeed The forward and backward speed of the robot from -1 to 1
   * @param ySpeed The left and right speed of the robot from -1 to 1
   * @param rot The rotation speed of the robot from -1 to 1
   * @param fieldRelative True orients the modules based on the gyroscope's forward, false orients the modules based on the robot's forward
   */
  public void drive(double xSpeed, double ySpeed, double rot,
    boolean fieldRelative) {
      
      double m_xSpeed;
      double m_ySpeed;
      double m_rot;

      m_xSpeed = xSpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
      m_ySpeed = ySpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
      m_rot = rot * DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

      m_xSpeed = xRateLimiter.calculate(m_xSpeed);
      m_ySpeed = yRateLimiter.calculate(m_ySpeed);

    var swerveModuleStates =
        DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
          (fieldRelative)
            //   Teleop otherwise auto
            ? ChassisSpeeds.fromFieldRelativeSpeeds(m_xSpeed, m_ySpeed, m_rot,
                getRotation2dDriver())
            : new ChassisSpeeds(m_xSpeed, m_ySpeed, m_rot));

    setModuleStates(swerveModuleStates, true);
  }

  /**
   * Takes in percentage speed values for x, y, and rotation and applies them to the swerves. 
   * Raw joystick values are multiplied by the motor's max speed in meters per second and then filtered throught a slew rate limiter.
   * Field relative movement is based off of the raw gyro value in degrees, which is subject to change from the pose estimator.
   * @param xSpeed The forward and backward speed of the robot from -1 to 1
   * @param ySpeed The left and right speed of the robot from -1 to 1
   * @param rot The rotation speed of the robot from -1 to 1
   * @param fieldRelative True orients the modules based on the gyroscope's forward, false orients the modules based on the robot's forward
   */
  public void driveAuto(double xSpeed, double ySpeed, double rot,
    boolean fieldRelative) {
      
      double m_xSpeed;
      double m_ySpeed;
      double m_rot;

      m_xSpeed = xSpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
      m_ySpeed = ySpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
      m_rot = rot * DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

      m_xSpeed = xRateLimiter.calculate(m_xSpeed);
      m_ySpeed = yRateLimiter.calculate(m_ySpeed);

    var swerveModuleStates =
        DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
          (fieldRelative)
            //   Teleop otherwise auto
            ? ChassisSpeeds.fromFieldRelativeSpeeds(m_xSpeed, m_ySpeed, m_rot,
                getRotation2d())
            : new ChassisSpeeds(m_xSpeed, m_ySpeed, m_rot));

    setModuleStates(swerveModuleStates, true);
  }
  /**
   * Applies chassis speeds directly to the swerve modules.
   * @param chassisSpeeds The desired chassis speeds
   * @param feedForwards The feed forwards for the drive (not used)
   */
  public void drive(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedForwards) {
    var swerveModuleStates =
      DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(swerveModuleStates, false);
  }
  /**
   * Gets all the swerve module states.
   * @return An array of all 4 swerve module states
   */
  public SwerveModuleState[] getModuleStates() {
    var myModuleStates = new SwerveModuleState[4];
    myModuleStates[0] = frontLeft.getState();
    myModuleStates[1] = frontRight.getState();
    myModuleStates[2] = rearLeft.getState();
    myModuleStates[3] = rearRight.getState();
    return myModuleStates;
  }
  /**
   * Set the swerve modules to the desired states.
   * @param desiredStates The desired swerve modules states
   * @param isOpenLoop Whether or not the PID controllers should be open loop
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(desiredStates[0], isOpenLoop);
    frontRight.setDesiredState(desiredStates[1], isOpenLoop);
    rearLeft.setDesiredState(desiredStates[2], isOpenLoop);
    rearRight.setDesiredState(desiredStates[3], isOpenLoop);
  }
  /**
   * Gets the chassis speeds of all 4 swerve modules.
   * @return The chassis speeds of all swerves
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }
  /**
   * Resets the encoders on the angle motors of each swerve to 0.
   */
  public void resetEncoders() {
    frontLeft.reset();
    frontRight.reset();
    rearLeft.reset();
    rearRight.reset();
  }
  /**
   * Sets all swerve modules to 0 speed.
   */
  public void stop() {
    var swerveModuleStates =
      DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        new ChassisSpeeds(0., 0., 0.));
    setModuleStates(swerveModuleStates, false);
  }
  /**
   * Gets the gyro's yaw.
   * @return The gyro's yaw as a Rotation2d
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }
  /**
   * Gets the drive yaw.
   * @return The drive yaw as a Rotation2d
   */
  public Rotation2d getRotation2dDriver() {
    return Rotation2d.fromDegrees(driveYaw);
  }
  /**
   * Gets the gyro's yaw.
   * @return The gyro's yaw in degrees
   */
  public double getHeading() {
    return gyro.getYaw().getValueAsDouble();
  }
  /**
   * Gets the estimated field pose from the pose estimator.
   * @return The pose of the robot
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition(); 
  }
  /**
   * Resets the pose estimator to a new pose.
   * @param pose The pose to reset to
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getRotation2d(), get_positions(), pose);
  }
  /**
   * Resets the gyro's yaw to 0.
   */
  public void zeroHeading() {
    //resetPose(new Pose2d(getPose().getTranslation(), new Rotation2d(0.)));
    gyro.reset();
  }
  /**
   * Gets the swerve module positions.
   * @return An array of all 4 swerve module positions
   */
  public SwerveModulePosition[] get_positions() {
    SwerveModulePosition[] m_positions = {frontLeft.getPosition(),frontRight.getPosition(),
    rearLeft.getPosition(),rearRight.getPosition()};
    return m_positions;
  }
  /**
   * Gets whether or not to flip autonomous paths.
   * @return True means paths should be flipped, false means not
   */
  public boolean flipPath() {
    var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
  }
  /**
   * Compiles all limelight pose measurements into a single pose called CompositeVisionPose.
   * Measurements are weighted based on target area.
   */
  public void processFrame() {

    double x = 0;
    double y = 0;
    double yaw = 0;
    double totalArea = 0;
    isVisionValid = false;

    if (limelightFrontLeft.getTagId() > 0) {
      if (limelightFrontLeft.getTargetArea() > VisionConstants.TARGET_AREA_THRESHHOLD) {
        totalArea += limelightFrontLeft.getTargetArea();
        x += limelightFrontLeft.getBotPose2d().getX() * limelightFrontLeft.getTargetArea();
        y += limelightFrontLeft.getBotPose2d().getY() * limelightFrontLeft.getTargetArea();
        yaw += limelightFrontLeft.getBotPose2d().getRotation().getDegrees() * limelightFrontLeft.getTargetArea();
        compositeLatency += limelightFrontLeft.getLatency();
      }
    } 

    if (limelightFrontRight.getTagId() > 0) {
      if (limelightFrontRight.getTargetArea() > VisionConstants.TARGET_AREA_THRESHHOLD) {
        totalArea += limelightFrontRight.getTargetArea();
        x += limelightFrontRight.getBotPose2d().getX() * limelightFrontRight.getTargetArea();
        y += limelightFrontRight.getBotPose2d().getY() * limelightFrontRight.getTargetArea();
        yaw += limelightFrontRight.getBotPose2d().getRotation().getDegrees() * limelightFrontRight.getTargetArea();
        compositeLatency += limelightFrontRight.getLatency();
      }
    }

    if (limelightRear.getTagId() > 0) {
      if (limelightRear.getTargetArea() > VisionConstants.TARGET_AREA_THRESHHOLD) {
        totalArea += limelightRear.getTargetArea();
        x += limelightRear.getBotPose2d().getX() * limelightRear.getTargetArea();
        y += limelightRear.getBotPose2d().getY() * limelightRear.getTargetArea();
        yaw += limelightRear.getBotPose2d().getRotation().getDegrees() * limelightRear.getTargetArea();
        compositeLatency += limelightRear.getLatency();
      }
    }

    if (totalArea < VisionConstants.TOTAL_TARGET_AREA_THRESHHOLD) {
      isVisionValid = false;
      compositeLatency = 0;
    } else {
      isVisionValid = true;
      x /= totalArea;
      y /= totalArea;
      yaw /= totalArea;
      compositeLatency /= totalArea;

      compositeVisionPose = new Pose2d(
        limelightXFilter.calculate(x),
        limelightYFilter.calculate(y),
        Rotation2d.fromDegrees(limelightYawFilter.calculate(yaw))
      );
    }

  }
  /**
   * Compiles all limelight target space measurements into a single pose called CompositeTargetSpacePose.
   * Measurements are weighted based on target area.
   */
  public void calculateFrameTargetSpace() {
    double x = 0;
    double y = 0;
    double yaw = 0;
    double totalArea = 0;
    isTargetSpacePoseValid = false;

    if (reefTagIDs.contains(limelightFrontLeft.getTagId())) {
      totalArea += limelightFrontLeft.getTargetArea();
      x += limelightFrontLeft.getBotPose2dTargetSpace().getX() * limelightFrontLeft.getTargetArea();
      y += limelightFrontLeft.getBotPose2dTargetSpace().getY() * limelightFrontLeft.getTargetArea();
      yaw += limelightFrontLeft.getBotPose2dTargetSpace().getRotation().getDegrees() * limelightFrontLeft.getTargetArea();
      compositeLatency += limelightFrontLeft.getLatency();
    }

    if (reefTagIDs.contains(limelightFrontRight.getTagId())) {
      totalArea += limelightFrontRight.getTargetArea();
      x += limelightFrontRight.getBotPose2dTargetSpace().getX() * limelightFrontRight.getTargetArea();
      y += limelightFrontRight.getBotPose2dTargetSpace().getY() * limelightFrontRight.getTargetArea();
      yaw += limelightFrontRight.getBotPose2dTargetSpace().getRotation().getDegrees() * limelightFrontRight.getTargetArea();
      compositeLatency += limelightFrontRight.getLatency();
    } 

    if (totalArea < VisionConstants.TOTAL_TARGET_AREA_THRESHHOLD) {
      isTargetSpacePoseValid = false;
      compositeLatency = 0;
    } else {
      isTargetSpacePoseValid = true;
      x /= totalArea;
      y /= totalArea;
      yaw /= totalArea;
      compositeLatency /= totalArea;

      compositeTargetSpacePose = new Pose2d(
        targetSpaceXFilter.calculate(x),
        targetSpaceYFilter.calculate(y),
        Rotation2d.fromDegrees(targetSpaceYawFilter.calculate(yaw))
      );
    }

  }
  /**
   * Gets the target space pose as a Pose2d.
   * @return The target space pose
   */
  public Pose2d getPose2dTargetSpace() {
    return compositeTargetSpacePose;
  }
  /**
   * Get whether or not the target space pose is available.
   * @return Whether or not the target space pose is available
   */
  public boolean isTargetSpacePoseValid() {
    return isTargetSpacePoseValid;
  }
  /**
   * Get whether or not the robot is ready to aim on the source.
   * @return Whether or not the rear limelight can see source tags
   */
  public boolean sourceReadyToAim() {
    return (sourceTagIDs.contains(limelightRear.getTagId()));
  }
  /**
   * Get whether or not the robot is ready to aim on the reef.
   * @return Whether or not the front limelights can see an AprilTag
   */
  public boolean reefReadyToAim() {
    return (isTargetSpacePoseValid);
  }


}