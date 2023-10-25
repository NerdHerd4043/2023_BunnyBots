// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package cowlib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.jni.CANSparkMaxJNI;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class SwerveModule {

    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private PIDController pidController;

    public SwerveModule(int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new CANSparkMax(angleMotor, MotorType.kBrushless);
        this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);
        this.pidController = new PIDController(1, 0, 0);

        this.pidController.enableContinuousInput(0, 360);
    }
}
