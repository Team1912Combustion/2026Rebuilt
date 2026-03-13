// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.text.FieldPosition;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DeviceIDs;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorIDs;
import frc.robot.Constants.VisionConstants;

public class DriveTrain extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(0, DeviceIDs.FRONT_LEFT.constants);
  private final SwerveModule frontRight = new SwerveModule(1, DeviceIDs.FRONT_RIGHT.constants);
  private final SwerveModule rearLeft = new SwerveModule(2, DeviceIDs.REAR_LEFT.constants);
  private final SwerveModule rearRight = new SwerveModule(3, DeviceIDs.REAR_RIGHT.constants);

  public final Pigeon2 gyro = new Pigeon2(SensorIDs.GYRO, new CANBus("1912CANivore"));
  Pigeon2Configuration gyroConfig;

  public double driveYaw;
  public double driveYawDirection;
  public double driveYawOffset;

  public boolean fieldRelative;

  LimelightClimberLeft limelightClimberLeft;
  LimelightClimberRight limelightClimberRight;

  MedianFilter limelightXFilter;
  MedianFilter limelightYFilter;
  MedianFilter limelightYawFilter;

  MedianFilter speedXFilter;
  MedianFilter speedYFilter;

  public SwerveDrivePoseEstimator poseEstimator;

  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(.1));
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, Units.degreesToRadians(50));
  private static final Vector<N3> visionStdDevsDisabled = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1));

  public SendableChooser<Command> autoChooser;

  public double poseX, poseY, poseYaw;
  Twist2d robotSpeed;

  double compositeLatency;
  Pose2d compositeVisionPose;

  boolean isVisionValid;

  SlewRateLimiter xRateLimiter, yRateLimiter;

  public XboxController driverController;

  public final double DISTANCE_ERROR_FRACTION = 0.1;

  boolean autoBuilderConfigured;

  RobotConfig cfg;

  Integer[] towerTagArray = {15, 31};

  List<Integer> towerTagIDs = Arrays.asList(towerTagArray);

  Field2d field;

  /** Creates a new DriveTrain. */
  public DriveTrain(LimelightClimberLeft llcl, LimelightClimberRight llcr) {

    gyroConfig = new Pigeon2Configuration();
    gyroConfig.MountPose.MountPoseYaw = 180;
    gyroConfig.MountPose.MountPoseRoll = 180;

    gyro.getConfigurator().apply(gyroConfig);

    fieldRelative = true;

    driveYaw = 0;
    driveYawDirection = 0;
    driveYawOffset = 0;

    gyro.setYaw(0);

    limelightClimberLeft = llcl;
    limelightClimberRight = llcr;
    limelightXFilter = new MedianFilter(3);
    limelightYFilter = new MedianFilter(3);
    limelightYawFilter = new MedianFilter(3);

    speedXFilter = new MedianFilter(5);
    speedYFilter = new MedianFilter(5);

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

    robotSpeed = new Twist2d(0, 0, 0);

    compositeLatency = 0;
    compositeVisionPose = new Pose2d();

    isVisionValid = false;

    xRateLimiter = new SlewRateLimiter(1 / DriveConstants.RAMP_TIME);
    yRateLimiter = new SlewRateLimiter(1 / DriveConstants.RAMP_TIME);

    driverController = new XboxController(0);

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

    autoBuilderConfigured = false;

    Logger.recordOutput("Pose", poseEstimator.getEstimatedPosition());

    field = new Field2d();
  }

  @Override
  public void periodic() {

    if (limelightClimberLeft.acceptPose() || limelightClimberRight.acceptPose()) {
      processFrame();
    }
    
    // update drive yaw
    driveYaw = gyro.getYaw().getValueAsDouble() + driveYawOffset;
    driveYaw = MathUtil.angleModulus(Math.toRadians(driveYaw));
    driveYaw = Math.toDegrees(driveYaw);

    LimelightHelpers.SetRobotOrientation(limelightClimberLeft.getName(), getHeading(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(limelightClimberRight.getName(), getHeading(), 0, 0, 0, 0, 0);

    // update pose estimator
    poseEstimator.update(getRotation2d(), get_positions());

    // update drive yaw while disabled
    if (DriverStation.isDisabled()) {
      if(limelightClimberLeft.getTagId() > 0) {
        driveYawOffset = (limelightClimberLeft.getBotPose2dMT2().getRotation().getDegrees() + driveYawDirection - gyro.getYaw().getValueAsDouble() * (flipPath() ? -1 : 1));
      } else if (limelightClimberRight.getTagId() > 0) {
        driveYawOffset = (limelightClimberLeft.getBotPose2dMT2().getRotation().getDegrees() + driveYawDirection - gyro.getYaw().getValueAsDouble() * (flipPath() ? -1 : 1));
      }

      if (flipPath()) {
        driveYawDirection = 180;
      } else if (!flipPath()) {
        driveYawDirection = 0;
      }
      
      fixPose();
      //poseEstimator.resetPose(new Pose2d(compositeVisionPose.getTranslation(), Rotation2d.fromDegrees(getHeading())));
      //poseEstimator.addVisionMeasurement(compositeVisionPose, Timer.getFPGATimestamp() - (compositeLatency / 1000), visionStdDevsDisabled);
    }

    // add pose to pose estimator
    if (isVisionValid) {
      poseEstimator.addVisionMeasurement(compositeVisionPose, Timer.getFPGATimestamp() - (compositeLatency / 1000), visionMeasurementStdDevs);
    }

    robotSpeed = new Twist2d(
      speedXFilter.calculate((poseEstimator.getEstimatedPosition().getX() - poseX) * 50),
      speedYFilter.calculate((poseEstimator.getEstimatedPosition().getY() - poseY) * 50),
      0
      );

    // update pose variables
    poseX = poseEstimator.getEstimatedPosition().getX();
    poseY = poseEstimator.getEstimatedPosition().getY();
    poseYaw = poseEstimator.getEstimatedPosition().getRotation().getDegrees();

    SmartDashboard.putData("Auto?:", autoChooser);

    field.setRobotPose(poseEstimator.getEstimatedPosition());
    field.getObject("pred").setPose(poseEstimator.getEstimatedPosition().exp(robotSpeed));
    SmartDashboard.putData(field);

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
   * Gets the direction from one pose to another.
   * @param origin The pose to start from
   * @param goal The pose to point at
   * @return The angle from the origin pose to the goal pose
   */
  public Rotation2d getDirection(Pose2d origin, Pose2d goal) {
    return Rotation2d.fromRadians(Math.atan2(
      origin.relativeTo(goal).getY(), 
      origin.relativeTo(goal).getX()
      )); 
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
   * Resets the pose estimator to the composite vision pose estimate.
   */
  public void fixPose() {
    poseEstimator.resetPose(new Pose2d(compositeVisionPose.getTranslation(), Rotation2d.fromDegrees(getHeading())));
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
  public Twist2d getRobotSpeed(){
    return robotSpeed;
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
   * Flips a pose around the origin to flip between red and blue.
   * @param pose The pose to flip
   * @return The flipped pose
   */
  public Pose2d flipCoordinates(Pose2d pose) {
    return pose.rotateAround(new Translation2d(8.1, 4.05), Rotation2d.fromDegrees(180));
  }
  /**
   * Compiles all limelight pose measurements into a single pose called compositeVisionPose.
   * Measurements are weighted based on target area.
   */
  public void processFrame() {

    double x = 0;
    double y = 0;
    double yaw = 0;
    double totalArea = 0;
    isVisionValid = false;

    if (limelightClimberLeft.acceptPose() && limelightClimberLeft.getPipeline() == 0) {
      if (limelightClimberLeft.getTargetArea() > VisionConstants.TARGET_AREA_THRESHHOLD) {
        totalArea += limelightClimberLeft.getTargetArea();
        x += limelightClimberLeft.getBotPose2dMT2().getX() * limelightClimberLeft.getTargetArea();
        y += limelightClimberLeft.getBotPose2dMT2().getY() * limelightClimberLeft.getTargetArea();
        yaw += limelightClimberLeft.getBotPose2dMT2().getRotation().getDegrees() * limelightClimberLeft.getTargetArea();
        compositeLatency += limelightClimberLeft.getLatency();
      }
    } 

    if (limelightClimberRight.acceptPose() && limelightClimberLeft.getPipeline() == 0) {
      if (limelightClimberRight.getTargetArea() > VisionConstants.TARGET_AREA_THRESHHOLD) {
        totalArea += limelightClimberRight.getTargetArea();
        x += limelightClimberRight.getBotPose2dMT2().getX() * limelightClimberRight.getTargetArea();
        y += limelightClimberRight.getBotPose2dMT2().getY() * limelightClimberRight.getTargetArea();
        yaw += limelightClimberRight.getBotPose2dMT2().getRotation().getDegrees() * limelightClimberRight.getTargetArea();
        compositeLatency += limelightClimberRight.getLatency();
      }
    }

    SmartDashboard.putNumber("Total tag area", totalArea);

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

  public boolean towerReadyToAim() {
    return towerTagIDs.contains(limelightClimberLeft.getTagId());
  }

  public Pose2d getFuelPosition() {
    if (limelightClimberLeft.getPipeline() == 1) {
      Pose3d limelightRobotPose = LimelightHelpers.getCameraPose3d_RobotSpace(limelightClimberLeft.getName());
      Pose2d limelightPose = getPose().transformBy(new Transform2d(limelightRobotPose.getX(), limelightRobotPose.getY(), new Rotation2d()));

      double distance = limelightClimberLeft.getTargetArea();
      double angle = (limelightClimberLeft.getXOffset() / 12) * (41);

      return limelightPose.transformBy(new Transform2d(distance * Math.cos(angle), distance * Math.sin(angle), new Rotation2d()));
    } else {
      return new Pose2d();
    }
  }

}