// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightLeft;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoClimbAlign extends Command {
  DriveTrain driveTrain;
  LimelightLeft limelightFrontLeft;

  private PIDController xController, yController, rotController;
  private boolean isRightClimb;
  /** Creates a new AutoClimbAlign. */
  public AutoClimbAlign(DriveTrain dt, LimelightLeft llfl, boolean isRightClimb) {
    driveTrain = dt;
    limelightFrontLeft = llfl;
    addRequirements(driveTrain);

    xController = new PIDController(0.45, 0, 0);
    yController = new PIDController(0.48, 0, 0);
    rotController = new PIDController(0.006, 0, 0);

    this.isRightClimb = isRightClimb;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setSetpoint(0);
    xController.setTolerance(0.03);

    yController.setSetpoint(isRightClimb ? 0 : -0);
    yController.setTolerance(0.03);

    rotController.setSetpoint(0);
    rotController.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driveTrain.towerReadyToAim()) {
      double xSpeed = xController.calculate(limelightFrontLeft.getBotPose2dTargetSpace().getX());
      xSpeed += 0.005 * Math.signum(xSpeed);
      double ySpeed = -yController.calculate(limelightFrontLeft.getBotPose2dTargetSpace().getY());
      double rotSpeed = -rotController.calculate(limelightFrontLeft.getBotPose2dTargetSpace().getRotation().getDegrees());
      rotSpeed += 0.005 * Math.signum(rotSpeed);
      driveTrain.drive(xSpeed, ySpeed, rotSpeed, false);
    } else {
      driveTrain.drive(0, 0, 0, false);
    }
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
