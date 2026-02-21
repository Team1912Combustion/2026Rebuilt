// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightTurret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretHood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootFuel extends Command {
  Shooter shooter;
  Spindexer spindexer;
  DriveTrain driveTrain;
  Turret turret;
  TurretHood turretHood;
  LimelightTurret limelightTurret;
  /** Creates a new ShootFuel. */
  public ShootFuel(Shooter s, Spindexer sp, DriveTrain dt, Turret t, TurretHood th, LimelightTurret lt) {
    shooter = s;
    spindexer = sp;
    driveTrain = dt;
    turret = t;
    turretHood = th;
    limelightTurret = lt;
    addRequirements(shooter, spindexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpeed(shooter.calculateSpeed(turret.getDistance(turret.getTurretPose().getTranslation(), turret.getTarget().getTranslation())));
    if (turret.isAimed() && turretHood.isInPosiiton() && shooter.shooterAtSpeed()) {
      shooter.kickerOn();
    } else {
      shooter.kickerOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shooterOff();
    spindexer.spindexerOff();
    shooter.kickerOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
