// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class FindTarget extends CommandBase {

  private Drivebase drivebase;
  private final DoubleSupplier speedX;
  private final DoubleSupplier speedY;
  private final DoubleSupplier rot;
  private boolean enemey;

  /** Creates a new FindTarget. */
  public FindTarget(Drivebase drivebase, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier rot) {
    this.drivebase = drivebase;
    this.speedX = speedX;
    this.speedY = speedY;
    this.rot = rot;
    enemey = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //turn on limelight
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.drive(speedX.getAsDouble(), speedY.getAsDouble(), rot.getAsDouble());
    //enemy = daMagicStuff;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return enemy;
  }
}
