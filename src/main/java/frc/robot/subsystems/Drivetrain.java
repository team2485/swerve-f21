package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import io.github.oblarg.oblog.Loggable;

public class Drivetrain extends SubsystemBase implements Loggable{

    private final PigeonIMU pigeon;
    
    private final SwerveModule[] modules;

    public Drivetrain() {
        pigeon = new PigeonIMU(DriveConstants.PIGEON_PORT);

        modules = new SwerveModule[] {
            new SwerveModule(DriveConstants.FL_DRIVE_TALON_PORT, DriveConstants.FL_ANGLE_TALON_PORT, DriveConstants.FL_CANCODER_PORT, DriveConstants.FL_CANCODER_ZERO, "FL"),
            new SwerveModule(DriveConstants.FR_DRIVE_TALON_PORT, DriveConstants.FR_ANGLE_TALON_PORT, DriveConstants.FR_CANCODER_PORT, DriveConstants.FR_CANCODER_ZERO, "FR"),
            new SwerveModule(DriveConstants.BL_DRIVE_TALON_PORT, DriveConstants.BL_ANGLE_TALON_PORT, DriveConstants.BL_CANCODER_PORT, DriveConstants.BL_CANCODER_ZERO, "BL"),
            new SwerveModule(DriveConstants.BR_DRIVE_TALON_PORT, DriveConstants.BR_ANGLE_TALON_PORT, DriveConstants.BR_CANCODER_PORT, DriveConstants.BR_CANCODER_ZERO, "BR")
        };
        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        for (SwerveModule m : modules) {
            m.addToShuffleboard();
        }

        tab.addNumber("Gyro heading", pigeon::getFusedHeading);

        
    }
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] states =
        DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
          fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(pigeon.getFusedHeading()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(states, DriveConstants.MAX_SPEED);
        for (int i = 0; i < states.length; i++) {
            SwerveModule module = modules[i];
            SwerveModuleState state = states[i];
            module.setDesiredState(state);
        }
    }
    
    
    
}
