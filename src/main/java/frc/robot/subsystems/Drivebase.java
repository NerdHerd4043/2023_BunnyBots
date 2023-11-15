// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;

import cowlib.SwerveModule;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.ModuleLocations;
import frc.robot.Constants.DriveConstants.SwerveModules;

public class Drivebase extends SubsystemBase {

  private SwerveModule frontLeft = new SwerveModule(SwerveModules.frontLeft);
  private SwerveModule frontRight = new SwerveModule(SwerveModules.frontRight);
  private SwerveModule backLeft = new SwerveModule(SwerveModules.backLeft);
  private SwerveModule backRight = new SwerveModule(SwerveModules.backRight);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    ModuleLocations.frontLeft,
    ModuleLocations.frontRight,
    ModuleLocations.backLeft,
    ModuleLocations.backRight
  );

  /** Creates a new Drivebase. */
  public Drivebase() {}

  public void drive(double speedX, double speedY, double rot) {
    this.drive(new ChassisSpeeds(speedX, speedY, rot));
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    this.frontLeft.drive(moduleStates[0]);
    this.frontRight.drive(moduleStates[1]);
    this.backLeft.drive(moduleStates[2]);
    this.backRight.drive(moduleStates[3]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
