// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package cowlib;

/** Add your docs here. */
public class SwerveModuleConfig {
    public final int driveMotorId;
    public final int angleMotorId;
    public final int encoderId;

    public SwerveModuleConfig(int driveMotorId, int angleMotorId, int encoderId) {
        this.driveMotorId = driveMotorId;
        this.angleMotorId = angleMotorId;
        this.encoderId = encoderId;
    }
}
