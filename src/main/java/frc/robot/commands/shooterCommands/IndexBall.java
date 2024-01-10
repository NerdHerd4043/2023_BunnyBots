// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;

public class IndexBall extends Command {
  private final Indexer indexer;
  private final Flywheel flywheel;
  private double setpoint = 0;
  private double timeCheck = 0;
  private boolean firstCheck = true;

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
    indexer.flipAtStart();
    indexer.enable();
    // indexer.getController().enableContinuousInput(0, 360);
    // flywheel.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setpoint = indexer.getAtStart() ? 60 : 240;
    flywheel.flywheelSpeed(ShooterConstants.flywheelSpeed);
    indexer.setSetpoint(setpoint);
    SmartDashboard.putNumber("Indexer Setpoint", setpoint);
    // flywheel.setSetpoint(ShooterConstants.flywheelSpeed);
    // indexer.setSetpoint(indexer.getAtStart() ? indexer.getStartPose() + 180 : indexer.getStartPose());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.disable();
    // flywheel.flywheelSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(setpoint - indexer.getEncoder()) < 20) {
      if(firstCheck) {
        timeCheck = Timer.getFPGATimestamp();
        firstCheck = !firstCheck;
      }
      if(Timer.getFPGATimestamp() - timeCheck > 0.1) {
        return true;
      }
    }
    else {
      firstCheck = true;
    }
    return false;
  }
}
