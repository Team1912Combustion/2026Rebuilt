// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldZoneConstants;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PointAtThing extends Command {
  DriveTrain driveTrain;

  PIDController rotPID;
  Pose2d originPose;
  Pose2d goalPose;
  
  /** Creates a new PointAtThing. */
  public PointAtThing(DriveTrain dt) {
    driveTrain = dt;
    addRequirements(driveTrain);

    rotPID = new PIDController(0.01, 0, 0);

    originPose = new Pose2d(driveTrain.getPose().getTranslation(), new Rotation2d());
    goalPose = new Pose2d(FieldZoneConstants.BLUE_HUB_SHOT_POINT, new Rotation2d());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotPID.setTolerance(5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    originPose = new Pose2d(driveTrain.getPose().getTranslation(), new Rotation2d());

    Rotation2d angle = Rotation2d.fromRadians(Math.atan2(
      goalPose.relativeTo(originPose).getY(), 
      goalPose.relativeTo(originPose).getX()
      )); 
    driveTrain.drive(-driveTrain.driverController.getLeftY(), -driveTrain.driverController.getLeftX(), rotPID.calculate(driveTrain.getPose().getRotation().getDegrees(), angle.getDegrees()), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
