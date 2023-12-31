// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import cowlib.SwerveModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final class DriveConstants {
		public static final double deadband = 0.15;
		public static final int currentLimit = 30;

		public static final class SwervePID {
			public static final double p = 0.11;
			public static final double i = 0;
			public static final double d = 0.0015;
		}

		public static final class SwerveModules {
			public static final SwerveModuleConfig frontLeft = new SwerveModuleConfig(1, 11, 21, true, 0);
			public static final SwerveModuleConfig frontRight = new SwerveModuleConfig(2, 12, 22, true, 0);
			public static final SwerveModuleConfig backRight = new SwerveModuleConfig(3, 13, 23, false, 0);
			public static final SwerveModuleConfig backLeft = new SwerveModuleConfig(4, 14, 24, true, 0);
		}

		public static final class ModuleLocations {
			public static final double dist = 10.325;
			public static final Translation2d frontLeft = new Translation2d(-dist, dist);
			public static final Translation2d frontRight = new Translation2d(dist, dist);
			public static final Translation2d backLeft = new Translation2d(-dist, -dist);
			public static final Translation2d backRight = new Translation2d(dist, -dist);
		}

		public static final class TargetPIDvalues {
			public static final double p = 0.08;
			public static final double i = 0;
			public static final double d = 0.0043;
		}

		public static final class TargetConstants {
			public static final double xCenter = 0;
			public static final int medianFilter = 5;
			public static boolean targetMode = false;
		}
	}
	public static final class ShooterConstants {
		public static final int flywheel1ID = 5;
		public static final int flywheel2ID = 15;
		public static final int hoodID = 25;
		public static final int hoodEncoderID = 35;
		public static final double flywheelSpeed = 1;

		public static final class FlywheelPIDs {
			public static final double p = 0;
			public static final double i = 0;
			public static final double d = 0;
		}

		public static final class HoodPIDs {
			public static final double p = 0;
			public static final double i = 0;
			public static final double d = 0;
		}

		public static final class HoodPositions {
			public static final double retracted = 14.0; // TODO get retracted value
			public static final double extended = 36.0; // TODO get extended value
		}
	}
	public static final class IndexerConstants {
		public static final int indexMotorID = 6;
		public static final int indexMotorEncoderID = 16;

		public static final class IndexerPIDs {
			public static final double p = 0.0044;
			public static final double i = 0;
			public static final double d = 0.000055;
		}
	}
	public static final class OperatorConstants {
		public static Boolean onBlueAlliance = false;
		public static final int kDriverControllerPort = 0;
	}
}
