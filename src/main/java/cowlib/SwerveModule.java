// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package cowlib;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.SwervePID;

/** Add your docs here. */
public class SwerveModule {
	private CANSparkMax angleMotor;
	private CANSparkMax speedMotor;
	private PIDController pidController;
	private CANCoder encoder;
	private boolean inverted;
	private double offset;

	public SwerveModule(int angleMotor, int speedMotor, int encoder, boolean drive_inverted) {
		this.angleMotor = new CANSparkMax(angleMotor, MotorType.kBrushless);
		this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);
		this.pidController = new PIDController(SwervePID.p, SwervePID.i, SwervePID.d);
		this.encoder = new CANCoder(encoder);
		this.inverted = drive_inverted;
		offset = 0;

		this.pidController.enableContinuousInput(-180, 180);
	}
	
	public SwerveModule(SwerveModuleConfig config, double maxVelocity, double maxVoltage) {
		this(config.angleMotorId, config.driveMotorId, config.encoderId, config.drive_inverted);
		angleMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
		speedMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
	}

	public void drive(double speed, double angle) {
		speedMotor.setVoltage(speed * (this.inverted ? -1 : 1));
		// angleMotor.set(MathUtil.clamp(pidController.calculate(encoder.getAbsolutePosition(), angle), -1, 1));
		angleMotor.setVoltage(1 - (pidController.calculate(this.getEncoder() + offset, angle + offset)));
	}

	public void drive(SwerveModuleState state) {
		this.drive(state.speedMetersPerSecond, state.angle.getDegrees());
	}

	public double getEncoder() {
		return encoder.getAbsolutePosition();
	}

	public double getRelativeEncoder() {
		return encoder.getAbsolutePosition() + offset;
	}

	public void resetEncoder() {
		offset = -getEncoder();
	}
}
