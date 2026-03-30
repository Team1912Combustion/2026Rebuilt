// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootFuel extends Command {
  Shooter shooter;
  Floor spindexer;
  DriveTrain driveTrain;
  Hood hood;
  /** Creates a new ShootFuel. */
  public ShootFuel(Shooter s, Floor sp, DriveTrain dt, Hood h) {
    shooter = s;
    spindexer = sp;
    driveTrain = dt;
    hood = h;
    addRequirements(spindexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.duckHood = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpeed(shooter.calculateSpeedContinuous(driveTrain.getDistance(driveTrain.getPose().getTranslation(), driveTrain.getTarget().getTranslation())));
    //shooter.setSpeed(-40);
    if (driveTrain.isAimed && hood.isInPosiiton() && shooter.shooterAtSpeed() && hood.duckHood()) {
      shooter.kickerOn();
      spindexer.setSpeed(-40);
    } else {
      shooter.kickerOff();
      spindexer.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.duckHood = true;
    shooter.shooterOff();
    spindexer.floorOff();
    shooter.kickerOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
