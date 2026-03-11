// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightClimberLeft;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoIntake extends Command {
  DriveTrain driveTrain; 
  Intake intake;
  LimelightClimberLeft limelight;
  SlewRateLimiter rotLimiter;

  PIDController pid;

  Timer intakeTimer;

  Pose2d ballPose;
  /** Creates a new AutoIntake. */
  public AutoIntake(DriveTrain dt, Intake i, LimelightClimberLeft llcl) {
    driveTrain = dt;
    intake = i;
    limelight = llcl;
    addRequirements(driveTrain, intake);

    pid = new PIDController(0.01, 0, 0);

    rotLimiter = new SlewRateLimiter(1);

    intakeTimer = new Timer();

    ballPose = driveTrain.getFuelPosition();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setTolerance(5);
    limelight.setPipeline(1);
    intake.armOut();
    intake.intake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(ballPose.getTranslation().getDistance(driveTrain.getFuelPosition().getTranslation())) > 0.4) {
      intakeTimer.start();
    }

    ballPose = driveTrain.getFuelPosition();

    driveTrain.drive(0.15, 0, pid.calculate(driveTrain.getHeading(), (intakeTimer.get() < 0.5) ? driveTrain.getDirection(driveTrain.getPose(), ballPose).getDegrees() : 0), false);

    if (intakeTimer.get() >= 0.5) {
      intakeTimer.stop();
      intakeTimer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.setPipeline(0);
    driveTrain.drive(0, 0, 0, true);
    intake.setRollerSpeed(0);
    intake.armIn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
