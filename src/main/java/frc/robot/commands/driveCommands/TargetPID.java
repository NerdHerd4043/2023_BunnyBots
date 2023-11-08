// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants.*;
import frc.robot.subsystems.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TargetPID extends PIDCommand {

  private final Drivebase drivebase;

  /** Creates a new TargetPID. */
  public TargetPID(Drivebase drivebase, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier xPose) {
    super(
        // The controller that the command will use
        new PIDController(TargetPIDvalues.p, TargetPIDvalues.i, TargetPIDvalues.d),
        // This should return the measurement
        xPose,
        // This should return the setpoint (can also be a constant)
        () -> TargetConstants.xCenter,
        // This uses the output
        output -> {
          drivebase.drive(speedX.getAsDouble(), speedY.getAsDouble(), output);
        });

    this.drivebase = drivebase;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
