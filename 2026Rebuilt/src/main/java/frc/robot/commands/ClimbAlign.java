// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbAlign extends Command {
  DriveTrain driveTrain;
  double x, y, rot;
  double xSpeed, ySpeed, rotSpeed;
  boolean flipPath;

  PIDController xController, yController, rotController;
  /** Creates a new DriveToPosition. */
  public ClimbAlign(DriveTrain dt, boolean isRightClimb) {
    driveTrain = dt;
    addRequirements(driveTrain);
    flipPath = driveTrain.flipPath();

    x = 1.05;
    y = (isRightClimb ? 2.8 : 4.56);
    rot = (isRightClimb ? 180 : 0);

    x = (flipPath ? driveTrain.flipCoordinates(new Pose2d(new Translation2d(x, y), new Rotation2d(rot))).getX() : x);
    y = (flipPath ? driveTrain.flipCoordinates(new Pose2d(new Translation2d(x, y), new Rotation2d(rot))).getY() : y);
    rot = (flipPath ? driveTrain.flipCoordinates(new Pose2d(new Translation2d(x, y), new Rotation2d(rot))).getRotation().getDegrees() : rot);
    xSpeed = 0;
    ySpeed = 0;
    rotSpeed = 0;

    xController = new PIDController(0.06, 0.02, 0);
    xController.setTolerance(0.05);
    yController = new PIDController(0.06, 0.02, 0);
    yController.setTolerance(0.03);
    rotController = new PIDController(0.01, 0, 0);
    rotController.setTolerance(5);
    rotController.enableContinuousInput(-180, 180);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed = xController.calculate(driveTrain.getPose().getX(), x);
    //xSpeed += Math.signum(xSpeed) * 0.02;
    if (xController.atSetpoint() && rotController.atSetpoint()) {
      ySpeed = yController.calculate(driveTrain.getPose().getY(), y);
      ySpeed += Math.signum(ySpeed) * 0.02;
    }
    rotSpeed = rotController.calculate(driveTrain.getPose().getRotation().getDegrees(), rot);

    driveTrain.driveAuto(xSpeed, ySpeed, rotSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint());
  }
}
