// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.driveCommands.FindTarget;
import frc.robot.commands.driveCommands.ToggleLimelight;
import frc.robot.commands.shooterCommands.ShootBall;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot(Drivebase drivebase, AHRS gyro, Indexer indexer, Flywheel flywheel, Boolean onBlueAlliance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoDrive(drivebase, gyro, 1, 4),
      new ToggleLimelight(),
      new FindTarget(drivebase, gyro, () -> 0, () -> 0, () -> 0, onBlueAlliance),
      new ShootBall(indexer, flywheel),
      new WaitCommand(1),
      new ShootBall(indexer, flywheel),
      new WaitCommand(1),
      new ShootBall(indexer, flywheel),
      new WaitCommand(1),
      new ShootBall(indexer, flywheel),
      new WaitCommand(1),
      new ShootBall(indexer, flywheel),
      new ToggleLimelight()
    );
  }
}
