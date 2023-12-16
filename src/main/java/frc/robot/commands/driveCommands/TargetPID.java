// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.function.DoubleSupplier;

import org.json.JSONObject;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants.*;
import frc.robot.subsystems.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TargetPID extends PIDCommand {

  private final Drivebase drivebase;
  private final AHRS gyro;

  /** Creates a new TargetPID. */
  public TargetPID(Drivebase drivebase, AHRS gyro, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier xPose) {
    super(
        // The controller that the command will use
        new PIDController(TargetPIDvalues.p, TargetPIDvalues.i, TargetPIDvalues.d),
        // This should return the measurement
        xPose,
        // This should return the setpoint (can also be a constant)
        () -> TargetConstants.xCenter,
        // This uses the output
        output -> {
          SmartDashboard.putNumber("Target PID Output", output);
          SmartDashboard.putNumber("Yaw", gyro.getYaw());
          drivebase.fieldOrientedDrive(speedX.getAsDouble(), speedY.getAsDouble(), -output/8.0, -gyro.getYaw());
        });

    this.drivebase = drivebase;
    this.gyro = gyro;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    String raw = NetworkTableInstance.getDefault().getTable("limelight").getEntry("json").getString("{}");
    var obj = new JSONObject(raw);
    var results = obj.optJSONObject("Results");
    var retro = results.optJSONArray("Retro");
    var length = retro.length();

    ArrayList<Double> arr = new ArrayList<Double>();
    double diff1;
    double diff2;
    boolean blueAlliance;

    for(int i = 0; i < length; i++){
      var item = retro.getJSONObject(i);
      var itemY = item.getDouble("ty");
      arr.add(itemY);
    }

    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 0) {
      return true;
    }
    else if(length >= 3){
      Collections.sort(arr);
      diff1 = Math.abs(arr.get(0) - arr.get(1));
      diff2 = Math.abs(arr.get(arr.size() - 2) - arr.get(arr.size() - 1));
      blueAlliance = diff1 > diff2;

      if(blueAlliance == OperatorConstants.onBlueAlliance) {
        return true;
      }
    }
    arr.clear();
    return false;
  }
}
