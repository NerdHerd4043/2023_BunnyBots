// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.*;

public class Flywheel extends PIDSubsystem {
  private final CANSparkMax flywheelMotor1 = new CANSparkMax(ShooterConstants.flywheel1ID, MotorType.kBrushless);
  private final CANSparkMax flywheelMotor2 = new CANSparkMax(ShooterConstants.flywheel2ID, MotorType.kBrushless);
  private RelativeEncoder encoder;

  /** Creates a new Flywheel. */
  public Flywheel() {
    super(
        // The PIDController used by the subsystem
        new PIDController(FlywheelPIDs.p, FlywheelPIDs.i, FlywheelPIDs.d));

        flywheelMotor1.restoreFactoryDefaults();
        flywheelMotor2.restoreFactoryDefaults();
        flywheelMotor1.setIdleMode(IdleMode.kBrake);
        flywheelMotor2.setIdleMode(IdleMode.kBrake);
        // flywheelMotor2.follow(flywheelMotor1);
        // flywheelMotor1.setInverted(false);
        // flywheelMotor2.setInverted(true);

        encoder = flywheelMotor1.getEncoder();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    SmartDashboard.putNumber("Flywheel Output", output);
    flywheelMotor1.setVoltage(output);
    flywheelMotor2.setVoltage(-output);
  }

  public void flywheelSpeed(double speed) {
    flywheelMotor1.set(speed);
    flywheelMotor2.set(-speed);
  }

  public void stop() {
    flywheelMotor1.set(0);
    flywheelMotor2.set(0);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return encoder.getVelocity();
  }
}
