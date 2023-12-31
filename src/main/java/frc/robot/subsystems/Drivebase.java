// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  private SwerveModule frontLeft = new SwerveModule(SwerveModules.frontLeft, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModule frontRight = new SwerveModule(SwerveModules.frontRight, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModule backLeft = new SwerveModule(SwerveModules.backLeft, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModule backRight = new SwerveModule(SwerveModules.backRight, MAX_VELOCITY, MAX_VOLTAGE);

  private SwerveModuleState frontLeftOptimised;
  private SwerveModuleState frontRightOptimised;
  private SwerveModuleState backLeftOptimised;
  private SwerveModuleState backRightOptimised;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    ModuleLocations.frontLeft,
    ModuleLocations.frontRight,
    ModuleLocations.backLeft,
    ModuleLocations.backRight
  );

  /** Creates a new Drivebase. */
  public Drivebase() {}
  
  public void fieldOrientedDrive(double speedX, double speedY, double rot, double angle) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, rot, Rotation2d.fromDegrees(angle));
    this.drive(speeds);
  }

  public void robotOrientedDrive(double speedX, double speedY, double rot) {
    ChassisSpeeds speeds = new ChassisSpeeds(speedX, speedY, rot);
    this.drive(speeds);
  }
  
  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY + 10);
    // SmartDashboard.putNumber("MaxVelocity", MAX_VELOCITY);
    frontLeftOptimised = SwerveModuleState.optimize(moduleStates[0], new Rotation2d(frontLeft.getEncoderRadians()));
    frontRightOptimised = SwerveModuleState.optimize(moduleStates[1], new Rotation2d(frontRight.getEncoderRadians()));
    backLeftOptimised = SwerveModuleState.optimize(moduleStates[2], new Rotation2d(backLeft.getEncoderRadians()));
    backRightOptimised = SwerveModuleState.optimize(moduleStates[3], new Rotation2d(backRight.getEncoderRadians()));

    this.frontLeft.drive(frontLeftOptimised);
    this.frontRight.drive(frontRightOptimised);
    this.backLeft.drive(backLeftOptimised);
    this.backRight.drive(backRightOptimised);

    SmartDashboard.putNumber("Target Angle", moduleStates[0].angle.getDegrees());
    // SmartDashboard.putNumber("FR Target Angle", moduleStates[1].angle.getDegrees());
    // SmartDashboard.putNumber("BR Target Angle", moduleStates[2].angle.getDegrees());
    // SmartDashboard.putNumber("BL Target Angle", moduleStates[3].angle.getDegrees());

    // SmartDashboard.putNumber("FL Target Speed", moduleStates[0].speedMetersPerSecond);
    // SmartDashboard.putNumber("FR Target Speed", moduleStates[1].speedMetersPerSecond);
    // SmartDashboard.putNumber("BR Target Speed", moduleStates[2].speedMetersPerSecond);
    // SmartDashboard.putNumber("BL Target Speed", moduleStates[3].speedMetersPerSecond);
  }

  public double getMaxVelocity() {
    return MAX_VELOCITY;
  }

  public double getMaxAngleVelocity() {
    return MAX_ANGULAR_VELOCITY;
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("FL Encoder", frontLeft.getEncoder());
    SmartDashboard.putNumber("FR Encoder", frontRight.getEncoder());
    SmartDashboard.putNumber("BR Encoder", backRight.getEncoder());
    SmartDashboard.putNumber("BL Encoder", backLeft.getEncoder());

    // SmartDashboard.putNumber("FL Relative Encoder", frontLeft.getRelativeEncoder());
    // SmartDashboard.putNumber("FR Relative Encoder", frontRight.getRelativeEncoder());
    // SmartDashboard.putNumber("BR Relative Encoder", backRight.getRelativeEncoder());
    // SmartDashboard.putNumber("BL Relative Encoder", backLeft.getRelativeEncoder());
    // This method will be called once per scheduler run
  }
}
