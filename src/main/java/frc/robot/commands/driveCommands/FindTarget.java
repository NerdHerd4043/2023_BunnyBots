// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import java.util.function.DoubleSupplier;

import org.json.JSONObject;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class FindTarget extends CommandBase {

  private Drivebase drivebase;
  private final DoubleSupplier speedX;
  private final DoubleSupplier speedY;
  private final DoubleSupplier rot;
  private boolean blueAlliance;

  /** Creates a new FindTarget. */
  public FindTarget(Drivebase drivebase, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier rot, boolean blueAlliance) {
    this.drivebase = drivebase;
    this.speedX = speedX;
    this.speedY = speedY;
    this.rot = rot;
    this.blueAlliance = blueAlliance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.drive(speedX.getAsDouble(), speedY.getAsDouble(), rot.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    String raw = NetworkTableInstance.getDefault().getTable("limelight").getEntry("json").getString("{}");
    // var obj = new JSONObject(raw);
    System.out.println(raw);
    return false;
    // return blueAlliance != NetworkTableInstance.getDefault().getTable("limelight").getEntry("json").stuff();
  }
}
