// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.function.DoubleSupplier;

import org.json.JSONObject;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class FindTarget extends CommandBase {

  private Drivebase drivebase;
  private final DoubleSupplier speedX;
  private final DoubleSupplier speedY;
  private final DoubleSupplier rot;
  private boolean onBlueAlliance;

  /** Creates a new FindTarget. */
  public FindTarget(Drivebase drivebase, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier rot, boolean onBlueAlliance) {
    this.drivebase = drivebase;
    this.speedX = speedX;
    this.speedY = speedY;
    this.rot = rot;
    this.onBlueAlliance = onBlueAlliance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.drive(speedX.getAsDouble(), speedY.getAsDouble(), rot.getAsDouble());
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
      diff1 = Math.abs(arr.get(0)- arr.get(1));
      diff2 = Math.abs(arr.get(arr.size() - 2) - arr.get(arr.size() - 1));
      blueAlliance = diff1 > diff2;

      if(blueAlliance != onBlueAlliance){
        return true;
      }
    }
    arr.clear();
    return false;
  }
}
