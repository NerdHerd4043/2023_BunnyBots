// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class AutoDrive extends CommandBase {
  private final Drivebase drivebase;
  private final AHRS gyro;
  private final double speed;
  private final double waitTime;
  private double startTime;

  /** Creates a new AutoDrive. */
  public AutoDrive(Drivebase drivebase, AHRS gyro, double speed, double waitTime) {
    this.drivebase = drivebase;
    this.gyro = gyro;
    this.speed = speed;
    this.waitTime = waitTime;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.robotOrientedDrive(speed * drivebase.getMaxVelocity(), 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.robotOrientedDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > waitTime;
    // return false;
  }
}
