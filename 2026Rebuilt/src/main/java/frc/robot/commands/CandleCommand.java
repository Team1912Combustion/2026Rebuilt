// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretHood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CandleCommand extends Command {
  LEDs leds;
  Turret turret;
  TurretHood hood;
  Shooter shooter;
  /** Creates a new CandleCommand. */
  public CandleCommand(LEDs l, Turret t, TurretHood h, Shooter s) {
    leds = l;
    turret = t;
    hood = h;
    shooter = s;
    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (turret.isAimed() && hood.isInPosiiton() && shooter.shooterAtSpeed()) {
      leds.setOrangeFlashing();
    } else {
      leds.setBlueStatic();
    }
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
