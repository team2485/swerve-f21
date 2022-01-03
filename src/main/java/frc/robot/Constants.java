// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import frc.team2485.WarlordsLib.IDManager;

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
    public static final String kRobotIdFile = "/home/lvuser/id.txt";

    public static final class OIConstants {
        public static final int kDriverPort = 0;
        public static final double kDriveDeadband = 0.08;
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

        public static final double kPDrive = 4;
        public static final double kPTurning = 5;
		public static final double kModuleMaxAngularSpeedRadiansPerSecond = 4*Math.PI;
		public static final double kModuleMaxModuleAngularAccelerationRadiansPerSecondSquared = 4*Math.PI;

        
    }

    public static final class DriveConstants {
        /**
         * Zeros found with bevel gears facing right. 
         * Offset is the negative of the zero. 
         */
        public static final int kPigeonPort = 9;

        public static final int kFLDriveTalonPort = 1;
        public static final int kFLAngleTalonPort = 2;
        public static final int kFLCANCoderPort = 11;
        public static final Rotation2d kFLCANCoderZero = Rotation2d.fromDegrees(-3.6);

        public static final int kFRDriveTalonPort = 3;
        public static final int kFRAngleTalonPort = 4;
        public static final int kFRCANCoderPort = 12;
        public static final Rotation2d kFRCANCoderZero = Rotation2d.fromDegrees(-167.7);

        public static final int kBRDriveTalonPort = 5;
        public static final int kBRAngleTalonPort = 6;
        public static final int kBRCANCoderPort = 13;
        public static final Rotation2d kBRCANCoderZero = Rotation2d.fromDegrees(56.0);

        public static final int kBLDriveTalonPort = 7;
        public static final int kBLAngleTalonPort = 8;
        public static final int kBLCANCoderPort = 14;
        public static final Rotation2d kBLCANCoderZero = Rotation2d.fromDegrees(139.0);

        //teleop values
        public static final double kTeleopMaxSpeedMetersPerSecond = 1; //meters per second
        public static final double kTeleopMaxAngularSpeedRadiansPerSecond = 10*Math.PI; //radians per second

        public static final double kWheelbaseLength = 0.635; //meters
        public static final double kWheelBaseWidth = 0.508; //meters
        
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(
                Units.inchesToMeters(kWheelbaseLength/2),
                Units.inchesToMeters(kWheelBaseWidth/2)
            ), 
            new Translation2d(
                Units.inchesToMeters(kWheelbaseLength/2),
                Units.inchesToMeters(-kWheelBaseWidth/2)
            ), 
            new Translation2d(
                Units.inchesToMeters(-kWheelbaseLength/2),
                Units.inchesToMeters(kWheelBaseWidth/2)
            ), 
            new Translation2d(
                Units.inchesToMeters(-kWheelbaseLength/2),
                Units.inchesToMeters(-kWheelBaseWidth/2)
            )  
        );
    }

    public static final class AutoConstants {
        public static final double kAutoMaxSpeedMetersPerSecond = 0.6;
        public static final double kAutoMaxAccelerationMetersPerSecondSquared = 0.5;
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
