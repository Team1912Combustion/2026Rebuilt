// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPosition extends Command {
  DriveTrain driveTrain;
  Pose2d pose;
  double xSpeed, ySpeed, rotSpeed;
  boolean flipPath;

  PIDController xController, yController, rotController;
  HolonomicDriveController pid;
  /** Creates a new DriveToPosition. */
  public DriveToPosition(DriveTrain dt, Pose2d pose) {
    driveTrain = dt;
    addRequirements(driveTrain);
    
    this.pose = pose;
    xSpeed = 0;
    ySpeed = 0;
    rotSpeed = 0;

    xController = new PIDController(0.42, 0, 0);
    xController.setTolerance(0.1);
    yController = new PIDController(0.42, 0, 0);
    yController.setTolerance(0.1);
    rotController = new PIDController(0.015, 0, 0);
    rotController.setTolerance(5);
    rotController.enableContinuousInput(-180, 180);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (driveTrain.flipPath()) { pose = driveTrain.flipCoordinates(pose); }

    xController.setSetpoint(pose.getX());
    yController.setSetpoint(pose.getY());
    rotController.setSetpoint(pose.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed = xController.calculate(driveTrain.getPose().getX());
    ySpeed = yController.calculate(driveTrain.getPose().getY());
    rotSpeed = rotController.calculate(driveTrain.getPose().getRotation().getDegrees());

    driveTrain.driveAuto(xSpeed, ySpeed, rotSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.driveAuto(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint());
  }
}
