// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;

public class DistanceShoot extends CommandBase {
  // private final Shooter shooter;
  private final Hood hood;
  private final Flywheel flywheel;
  private final DoubleSupplier distance;
  private double hoodPosition;
  private double flywheelSpeed;

  /** Creates a new DistanceShoot. */
  public DistanceShoot(Hood hood, Flywheel flywheel, DoubleSupplier distance) {
    this.hood = hood;
    this.flywheel = flywheel;
    this.distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.hood, this.flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.enable();
    flywheel.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hoodPosition = 0; //funky math
    flywheelSpeed = 0; //more funky math
    hood.setSetpoint(hoodPosition);
    flywheel.setSetpoint(flywheelSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.disable();
    flywheel.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
