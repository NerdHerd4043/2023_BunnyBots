// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.*;

public class Hood extends PIDSubsystem {
  private final CANSparkMax hoodMotor = new CANSparkMax(ShooterConstants.hoodID, MotorType.kBrushless);
  private WPI_CANCoder hoodEncoder = new WPI_CANCoder(ShooterConstants.hoodEncoderID);

  /** Creates a new Hood. */
  public Hood() {
    super(
        // The PIDController used by the subsystem
        new PIDController(HoodPIDs.p, HoodPIDs.i, HoodPIDs.d));

    hoodMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    hoodMotor.set(output);
  }

  public double getEncoderPosition() {
    return hoodEncoder.getAbsolutePosition();
  }

  public void adjust(double speed) {
    hoodMotor.set(speed);
  }

  public void stop() {
    hoodMotor.set(0);
  }

  private double map(double t) {
    return (t - HoodPositions.retracted) / (HoodPositions.extended - HoodPositions.retracted);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return map(hoodEncoder.getAbsolutePosition());
  }
}
