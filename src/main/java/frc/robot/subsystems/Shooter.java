// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  //okay I think I might want to do PID subsystems instead for these but shhhhhhhhhhhhh
  private final CANSparkMax flywheelMotor = new CANSparkMax(ShooterConstants.flywheelID, MotorType.kBrushless);
  private final CANSparkMax hoodMotor = new CANSparkMax(ShooterConstants.hoodID, MotorType.kBrushless);

  private WPI_CANCoder flywheelEncoder = new WPI_CANCoder(ShooterConstants.flywheelEncoderID);
  private WPI_CANCoder hoodEncoder = new WPI_CANCoder(ShooterConstants.hoodEncoderID);

  /** Creates a new Shooter. */
  public Shooter() {
    flywheelMotor.setIdleMode(IdleMode.kBrake);
    hoodMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setHoodAngle(double angle){
    //new HoodPID(angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
