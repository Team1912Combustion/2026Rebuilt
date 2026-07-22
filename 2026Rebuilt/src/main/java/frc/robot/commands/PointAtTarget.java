// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PointAtTarget extends Command {
  DriveTrain driveTrain;
  LimelightShooter limelightShooter;

  PIDController rotPID;
  PIDController tagPID;
  Pose2d originPose;
  Pose2d goalPose;

  Timer timer;
  boolean poseFixed;

  Debouncer debouncer;
  
  /** Creates a new PointAtTarget. */
  public PointAtTarget(DriveTrain dt, LimelightShooter lls) {
    driveTrain = dt;
    limelightShooter = lls;
    addRequirements(driveTrain);

    rotPID = new PIDController(0.03, 0, 0.004);
    rotPID.enableContinuousInput(-180, 180);
    rotPID.setTolerance(2);

    tagPID = new PIDController(0.04, 0, 0.0);
    tagPID.setTolerance(1);

    originPose = new Pose2d(driveTrain.getPose().getTranslation(), new Rotation2d());
    goalPose = new Pose2d(driveTrain.getCurrentFieldZone().getShotPoint(), new Rotation2d());

    timer = new Timer();
    poseFixed = false;

    debouncer = new Debouncer(0.2, DebounceType.kBoth);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limelightShooter.setPipeline(1);
    originPose = new Pose2d(driveTrain.getPose().getTranslation(), new Rotation2d());
    goalPose = new Pose2d(driveTrain.getCurrentFieldZone().getShotPoint(), new Rotation2d());

    Rotation2d angle = Rotation2d.fromRadians(Math.atan2(
      goalPose.relativeTo(originPose).getY(), 
      goalPose.relativeTo(originPose).getX()
      )); 
    //angle.plus(Rotation2d.fromDegrees(180));

    double tx = limelightShooter.getXOffset();

    if (driveTrain.isHubTag(limelightShooter.getTagId())) {
      driveTrain.fixPose();
    }

    /*if (timer.get() > 2 && poseFixed) {
      poseFixed = false;
      timer.reset();
    }*/

    double output = rotPID.calculate(driveTrain.angleModulus(driveTrain.getPose().getRotation().getDegrees() + 180), angle.getDegrees());

    if (debouncer.calculate(Math.abs(output) > 0.06) || Math.abs(driveTrain.driverController.getLeftY()) > 0.01 || Math.abs(driveTrain.driverController.getLeftX()) > 0.01) {
      driveTrain.drive(-driveTrain.driverController.getLeftY(), -driveTrain.driverController.getLeftX(), rotPID.calculate(driveTrain.angleModulus(driveTrain.getPose().getRotation().getDegrees() + 180), angle.getDegrees()), true);
    } else {
      driveTrain.setXBrake();
    }

    driveTrain.isAimed = (rotPID.atSetpoint() ? true : false);
    //driveTrain.drive(-driveTrain.driverController.getLeftY(), -driveTrain.driverController.getLeftX(), rotPID.calculate((driveTrain.getPose().getRotation().getDegrees()), angle.getDegrees()), true);

    poseFixed = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.isAimed = false;
    limelightShooter.setPipeline(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
