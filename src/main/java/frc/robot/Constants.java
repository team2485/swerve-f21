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
    public static final class OIConstants {
        public static final int DRIVER_PORT = 0;
    }
    public static final class ModuleConstants {
        public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; //radians/second
        public static final double MAX_ANGULAR_ACCELERATION = 2 * Math.PI; //radians/second
        
        public static final int FALCON_CPR = 2048; //pulses per rotation
        public static final double WHEEL_DIAMETER = 0.15; //meters
        public static final double DRIVE_GEAR_RATIO = 8.16; //motor turns/wheel turns
        public static final double DRIVE_DIST_PER_MOTOR_REV = (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
        public static final double DRIVE_DIST_PER_PULSE = DRIVE_DIST_PER_MOTOR_REV / FALCON_CPR;

        public static final int CANCODER_CPR = 4096;
        public static final double TURNING_ANGLE_PER_PULSE = (2 * Math.PI)/CANCODER_CPR;
    }

    public static final class DriveConstants {
        public static final int PIGEON_PORT = 20;

        public static final int FL_DRIVE_TALON_PORT = 1;
        public static final int FL_ANGLE_TALON_PORT = 2;
        public static final int FL_CANCODER_PORT = 11;
        public static final Rotation2d FL_CANCODER_ZERO = Rotation2d.fromDegrees(176.3);

        public static final int FR_DRIVE_TALON_PORT = 3;
        public static final int FR_ANGLE_TALON_PORT = 4;
        public static final int FR_CANCODER_PORT = 12;
        public static final Rotation2d FR_CANCODER_ZERO = Rotation2d.fromDegrees(196.2);

        public static final int BR_DRIVE_TALON_PORT = 5;
        public static final int BR_ANGLE_TALON_PORT = 6;
        public static final int BR_CANCODER_PORT = 13;
        public static final Rotation2d BR_CANCODER_ZERO = Rotation2d.fromDegrees(55.7);

        public static final int BL_DRIVE_TALON_PORT = 7;
        public static final int BL_ANGLE_TALON_PORT = 8;
        public static final int BL_CANCODER_PORT = 14;
        public static final Rotation2d BL_CANCODER_ZERO = Rotation2d.fromDegrees(318.9);



        public static final double MAX_SPEED = 3; //meters per second
        public static final double MAX_ANGULAR_SPEED = Math.PI; //rots per sec
        public static final double WHEELBASE_LENGTH = 25;
        public static final double WHEELBASE_WIDTH = 20;
        
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
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

	
}
