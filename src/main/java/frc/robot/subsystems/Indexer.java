// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.IndexerPIDs;

public class Indexer extends PIDSubsystem {
  private final CANSparkMax indexMotor = new CANSparkMax(IndexerConstants.indexMotorID, MotorType.kBrushless);
  private final WPI_CANCoder encoder = new WPI_CANCoder(IndexerConstants.indexMotorEncoderID);
  private final double startPose;
  private boolean atStart = false;

  /** Creates a new Indexer. */
  public Indexer() {
    super(
      // The PIDController used by the subsystem
      new PIDController(IndexerPIDs.p, IndexerPIDs.i, IndexerPIDs.d));

    indexMotor.restoreFactoryDefaults();
    indexMotor.setIdleMode(IdleMode.kBrake);
    startPose = encoder.getAbsolutePosition();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    indexMotor.set(output);
  }

  public void drive(double speed) {
    indexMotor.set(speed);
  }

  public double getStartPose() {
    return startPose;
  }

  public boolean getAtStart() {
    return atStart;
  }

  public void flipAtStart() {
    atStart = !atStart;
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return Math.abs(encoder.getAbsolutePosition());
  }
}
