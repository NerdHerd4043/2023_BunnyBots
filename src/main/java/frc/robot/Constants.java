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
		public static final class SwervePID {
			public static final double p = 1;
			public static final double i = 0;
			public static final double d = 0.1;
		}

		public static final class SwerveModules {
			public static final SwerveModuleConfig frontLeft = new SwerveModuleConfig(0, 1, 2);
			public static final SwerveModuleConfig frontRight = new SwerveModuleConfig(3, 4, 5);
			public static final SwerveModuleConfig backLeft = new SwerveModuleConfig(6, 7, 8);
			public static final SwerveModuleConfig backRight = new SwerveModuleConfig(9, 10, 11);
		}

		public static final class ModuleLocations {
			public static final double dist = 10.325;
			public static final Translation2d frontLeft = new Translation2d(-dist, dist);
			public static final Translation2d frontRight = new Translation2d(dist, dist);
			public static final Translation2d backLeft = new Translation2d(-dist, -dist);
			public static final Translation2d backRight = new Translation2d(dist, -dist);
		}

		public static final class TargetPIDvalues {
			public static final double p = 0;
			public static final double i = 0;
			public static final double d = 0;
		}

		public static final class TargetConstants {
			public static final double xCenter = 0;
			public static final int medianFilter = 5;
		}
	}
	public static class ShooterConstants {
		public static final int flywheelID = 12;
		public static final int hoodID = 13;
		public static final int hoodEncoderID = 1;

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
	}
	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}
}
