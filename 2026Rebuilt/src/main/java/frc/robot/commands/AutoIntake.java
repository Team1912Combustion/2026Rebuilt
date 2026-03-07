// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

  double tx;
  double ty;

  PIDController pid;

  Timer intakeTimer;
  /** Creates a new AutoIntake. */
  public AutoIntake(DriveTrain dt, Intake i, LimelightClimberLeft llcl) {
    driveTrain = dt;
    intake = i;
    limelight = llcl;
    addRequirements(driveTrain, intake);

    pid = new PIDController(0.01, 0, 0);

    rotLimiter = new SlewRateLimiter(1);

    tx = 0;
    ty = 0;

    intakeTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setTolerance(0.1);
    pid.setSetpoint(0);
    limelight.setPipeline(1);
    intake.armOut();
    intake.intake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double deltatx = limelight.getXOffset() - tx;

    if (deltatx > 0.05) {
      intakeTimer.start();
    }

    tx = limelight.getXOffset();
    ty = limelight.getYOffset();

    driveTrain.drive(0.15, 0, rotLimiter.calculate((tx != 0) || (intakeTimer.get() >= 0.5) ? pid.calculate(tx) : 0), false);

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
