// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class TargetDrive extends CommandBase {

  private final Drivebase drivebase;
  private final DoubleSupplier speedX;
  private final DoubleSupplier speedY;
  private final DoubleSupplier xPose;

  /** Creates a new Drive. */
  public TargetDrive(Drivebase drivebase, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier xPose) {
    this.drivebase = drivebase;
    this.speedX = speedX;
    this.speedY = speedY;
    this.xPose = xPose;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new TargetPID(drivebase, speedX, speedY, xPose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("").setNumber(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
