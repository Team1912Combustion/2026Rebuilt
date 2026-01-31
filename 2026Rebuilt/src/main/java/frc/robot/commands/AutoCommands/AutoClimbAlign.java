// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoClimbAlign extends Command {
  DriveTrain driveTrain;

  private PIDController xController, yController, rotController;
  private boolean isRightClimb;
  /** Creates a new AutoClimbAlign. */
  public AutoClimbAlign(DriveTrain dt, boolean isRightClimb) {
    driveTrain = dt;
    addRequirements(driveTrain);

    xController = new PIDController(0.45, 0, 0);
    yController = new PIDController(0.48, 0, 0);
    rotController = new PIDController(0.006, 0, 0);

    this.isRightClimb = isRightClimb;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
