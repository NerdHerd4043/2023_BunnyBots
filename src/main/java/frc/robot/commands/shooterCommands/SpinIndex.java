// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class SpinIndex extends CommandBase {
  private final Indexer indexer;
  private double timerStart;

  /** Creates a new SpinIndex. */
  public SpinIndex(Indexer indexer) {
    this.indexer = indexer;
    timerStart = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timerStart = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.drive(0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.drive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - timerStart > 0.1;
  }
}
