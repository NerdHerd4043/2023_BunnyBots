// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;

import cowlib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.ModuleLocations;
import frc.robot.Constants.DriveConstants.SwerveModules;

public class Drivebase extends SubsystemBase {
  private final double DRIVE_REDUCTION = 1.0 / 6.75;
  private final double NEO_FREE_SPEED = 5820.0 / 60.0;
  private final double WHEEL_DIAMETER = 0.1016;
  private final double MAX_VELOCITY = NEO_FREE_SPEED * DRIVE_REDUCTION * WHEEL_DIAMETER * Math.PI;
  private final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (ModuleLocations.dist / Math.sqrt(2.0));

  private final double MAX_VOLTAGE = 12;

  // private SwerveModule frontLeft = new SwerveModule(SwerveModules.frontLeft, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModule frontRight = new SwerveModule(SwerveModules.frontRight, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModule backLeft = new SwerveModule(SwerveModules.backLeft, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModule backRight = new SwerveModule(SwerveModules.backRight, MAX_VELOCITY, MAX_VOLTAGE);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    ModuleLocations.frontLeft,
    ModuleLocations.frontRight,
    ModuleLocations.backLeft,
    ModuleLocations.backRight
  );

  /** Creates a new Drivebase. */
  public Drivebase() {}
  
  public void drive(double speedX, double speedY, double rot) {
    // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, rot, Rotation2d.fromDegrees(angle));
    this.drive(new ChassisSpeeds(speedX, speedY, rot));
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY);

    // SmartDashboard.putNumber("FL Angle", moduleStates[0].angle.getDegrees());
    SmartDashboard.putNumber("FR Angle", moduleStates[1].angle.getDegrees());
    SmartDashboard.putNumber("BR Angle", moduleStates[2].angle.getDegrees());
    SmartDashboard.putNumber("BL Angle", moduleStates[3].angle.getDegrees());

    // this.frontLeft.drive(moduleStates[0]);
    this.frontRight.drive(moduleStates[1]);
    this.backLeft.drive(moduleStates[2]);
    this.backRight.drive(moduleStates[3]);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("FL Encoder", frontLeft.getEncoder());
    SmartDashboard.putNumber("FR Encoder", frontRight.getEncoder());
    SmartDashboard.putNumber("BR Encoder", backRight.getEncoder());
    SmartDashboard.putNumber("BL Encoder", backLeft.getEncoder());
    // This method will be called once per scheduler run
  }
}
