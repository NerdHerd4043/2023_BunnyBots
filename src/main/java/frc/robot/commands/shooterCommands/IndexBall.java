// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;

public class IndexBall extends CommandBase {
  private final Indexer indexer;
  private final Flywheel flywheel;
  /** Creates a new IndexBall. */
  public IndexBall(Indexer indexer, Flywheel flywheel) {
    this.indexer = indexer;
    this.flywheel = flywheel;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.indexer, this.flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.enable();
    flywheel.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.flipAtStart();
    flywheel.setSetpoint(ShooterConstants.flywheelSetpoint);
    indexer.setSetpoint(indexer.getAtStart() ? indexer.getStartPose() + 180 : indexer.getStartPose());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
