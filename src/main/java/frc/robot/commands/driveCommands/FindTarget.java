// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.function.DoubleSupplier;

import org.json.JSONObject;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drivebase;

public class FindTarget extends Command {

  private Drivebase drivebase;
  private final AHRS gyro;
  private final DoubleSupplier speedX;
  private final DoubleSupplier speedY;
  private final DoubleSupplier rot;

  /** Creates a new FindTarget. */
  public FindTarget(Drivebase drivebase, AHRS gyro, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier rot) {
    this.drivebase = drivebase;
    this.gyro = gyro;
    this.speedX = speedX;
    this.speedY = speedY;
    this.rot = rot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Yaw", gyro.getYaw());
    drivebase.fieldOrientedDrive(speedX.getAsDouble(), speedY.getAsDouble(), rot.getAsDouble(), -gyro.getYaw());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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

    if(length >= 3){
      Collections.sort(arr);
      diff1 = Math.abs(arr.get(0) - arr.get(1));
      diff2 = Math.abs(arr.get(arr.size() - 2) - arr.get(arr.size() - 1));
      blueAlliance = diff1 > diff2;

      if(blueAlliance != OperatorConstants.onBlueAlliance){
        return true;
      }
    }
    arr.clear();
    return false;
  }
}
