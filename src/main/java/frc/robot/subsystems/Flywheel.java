// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.*;

public class Flywheel extends PIDSubsystem {
  private final CANSparkMax flywheelMotor = new CANSparkMax(ShooterConstants.flywheelID, MotorType.kBrushless);
  private RelativeEncoder encoder;

  /** Creates a new Flywheel. */
  public Flywheel() {
    super(
        // The PIDController used by the subsystem
        new PIDController(FlywheelPIDs.p, FlywheelPIDs.i, FlywheelPIDs.d));

    flywheelMotor.restoreFactoryDefaults();
    flywheelMotor.setIdleMode(IdleMode.kBrake);
    encoder = flywheelMotor.getEncoder();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    flywheelMotor.setVoltage(output);
  }

  public void manualShoot(double speed) {
    flywheelMotor.set(speed);
  }

  public void stop() {
    flywheelMotor.set(0);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return encoder.getVelocity();
  }
}
