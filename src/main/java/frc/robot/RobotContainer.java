// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DriveConstants.TargetConstants;
import frc.robot.commands.autoCommands.AutoDrive;
import frc.robot.commands.autoCommands.AutoShoot;
import frc.robot.commands.driveCommands.Drive;
import frc.robot.commands.driveCommands.ToggleLimelight;
import frc.robot.commands.driveCommands.TargetingMode;
import frc.robot.commands.shooterCommands.ShootBall;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Drivebase drivebase = new Drivebase();
  private final Flywheel flywheel = new Flywheel();
  private final Hood hood = new Hood();
  private final Indexer indexer = new Indexer();

  private final AHRS gyro = new AHRS();
  
  private final AutoDrive autoDrive = new AutoDrive(drivebase, gyro, 0.5, 2);
  private final AutoShoot autoShoot = new AutoShoot(drivebase, gyro, indexer, flywheel);
  SendableChooser<Command> commandChooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private static CommandXboxController driveStick = new CommandXboxController(0);

  private MedianFilter filter = new MedianFilter(DriveConstants.TargetConstants.medianFilter);

  
  NetworkTable limelightTable =
  NetworkTableInstance.getDefault().getTable("limelight");

  
  private final DoubleSupplier xPose =
  () -> filter.calculate(limelightTable.getEntry("tx").getDouble(0));
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    commandChooser.addOption("Simple Drive", autoDrive);
    commandChooser.addOption("Drive and Shoot", autoShoot);

    SmartDashboard.putData("Autos", commandChooser);
  
    drivebase.setDefaultCommand(
      new TargetingMode(
        drivebase,
        gyro,
        () -> deadband(-driveStick.getLeftY(), DriveConstants.deadband) * drivebase.getMaxVelocity() * 1.7,
        () -> deadband(-driveStick.getLeftX(), DriveConstants.deadband) * drivebase.getMaxVelocity() * 1.7,
        () -> deadband(driveStick.getRightX(), DriveConstants.deadband) * drivebase.getMaxAngleVelocity(),
        xPose)
        );

    hood.setDefaultCommand(
      new RunCommand(
        () -> hood.adjust(
          0.4 * (driveStick.getRightTriggerAxis() - driveStick.getLeftTriggerAxis())),
          hood)
    );

    // Configure the trigger bindings
    configureBindings();
  }

  private double deadband(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0;
    } else {
      return input;
    }
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void updateAlliance() {
    OperatorConstants.onBlueAlliance = DriverStation.getAlliance() == Alliance.Blue;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driveStick.a().onTrue(new RunCommand(() -> flywheel.flywheelSpeed(ShooterConstants.flywheelSpeed), flywheel));
    driveStick.b().onTrue(new RunCommand(() -> flywheel.flywheelSpeed(0), flywheel));
    driveStick.rightBumper().onTrue(new ShootBall(indexer, flywheel));
    driveStick.pov(0).onTrue(new InstantCommand(gyro::reset));
    driveStick.leftBumper().onTrue(new ToggleLimelight());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return commandChooser.getSelected();
  }
}
