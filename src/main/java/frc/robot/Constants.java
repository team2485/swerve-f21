// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final String CONFIGS_FILE = "/home/lvuser/constants.csv";
    public static final class OIConstants {
        public static final int kDriverPort = 0;
        public static final double kDriveDeadband = 0.05;
        public static final double kDriveSlewRate = 2; //units per second to limit rate to, inverse of how long it will take from 0 to 1
    }

    public static final class ModuleConstants {
        
        public static final int kFalconCPR = 2048; //pulses per rotation
        public static final double kWheelDiameterMeters = 0.1016; //meters
        public static final double kDriveGearRatio = 8.16; //motor turns/wheel turns
        public static final double kDriveDistMetersPerRevolution = (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;
        public static final double kDriveDistMetersPerPulse = 10 * kDriveDistMetersPerRevolution/ kFalconCPR;

        public static final double ksVoltsDrive = 0.477;
        public static final double kvVoltSecondsPerMeterDrive = 2.98;
        public static final double kaVoltSecondsSquaredPerMeterDrive = 0.147;

        public static final int kCANCoderCPR = 4096;
        public static final double kTurningAngleRadiansPerPulse = (2 * Math.PI)/kCANCoderCPR;
        public static final double kTurningGearRatio = 12.8;
        public static final double ksVoltsTurning = 0.722;
        public static final double kvVoltSecondsPerMeterTurning = 0.216;

        public static final double kPDrive = 3;
        public static final double kPTurning = 5;
		public static final double kMaxModuleAngularSpeedRadiansPerSecond = 4*Math.PI;
		public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4*Math.PI;

        
    }

    public static final class DriveConstants {
        public static final int PIGEON_PORT = 9;

        public static final int FL_DRIVE_TALON_PORT = 1;
        public static final int FL_ANGLE_TALON_PORT = 2;
        public static final int FL_CANCODER_PORT = 11;
        public static final Rotation2d FL_CANCODER_ZERO = Rotation2d.fromDegrees(-3.6);

        public static final int FR_DRIVE_TALON_PORT = 3;
        public static final int FR_ANGLE_TALON_PORT = 4;
        public static final int FR_CANCODER_PORT = 12;
        public static final Rotation2d FR_CANCODER_ZERO = Rotation2d.fromDegrees(-167.7);

        public static final int BR_DRIVE_TALON_PORT = 5;
        public static final int BR_ANGLE_TALON_PORT = 6;
        public static final int BR_CANCODER_PORT = 13;
        public static final Rotation2d BR_CANCODER_ZERO = Rotation2d.fromDegrees(56.0);

        public static final int BL_DRIVE_TALON_PORT = 7;
        public static final int BL_ANGLE_TALON_PORT = 8;
        public static final int BL_CANCODER_PORT = 14;
        public static final Rotation2d BL_CANCODER_ZERO = Rotation2d.fromDegrees(139.0);

        //teleop values
        public static final double kMaxSpeedMetersPerSecond = 1; //meters per second
        public static final double kMaxAngularSpeedRadiansPerSecond = 10*Math.PI; //radians per second

        public static final double WHEELBASE_LENGTH = 0.635; //meters
        public static final double WHEELBASE_WIDTH = 0.508; //meters
        
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(
                Units.inchesToMeters(WHEELBASE_LENGTH/2),
                Units.inchesToMeters(WHEELBASE_WIDTH/2)
            ), 
            new Translation2d(
                Units.inchesToMeters(WHEELBASE_LENGTH/2),
                Units.inchesToMeters(-WHEELBASE_WIDTH/2)
            ), 
            new Translation2d(
                Units.inchesToMeters(-WHEELBASE_LENGTH/2),
                Units.inchesToMeters(WHEELBASE_WIDTH/2)
            ), 
            new Translation2d(
                Units.inchesToMeters(-WHEELBASE_LENGTH/2),
                Units.inchesToMeters(-WHEELBASE_WIDTH/2)
            )  
        );
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 0.6;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
    }
	
}
