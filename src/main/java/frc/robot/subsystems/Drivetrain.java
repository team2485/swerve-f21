package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.AutoConstants.*;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.ShuffleboardContainerWrapper;
import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase implements Loggable{

    private final PigeonIMU m_pigeon;
    
   // public final SwerveModule[] m_modules;
    public final SwerveModule m_frontLeftModule;
    public final SwerveModule m_frontRightModule;
    public final SwerveModule m_backLeftModule;
    public final SwerveModule m_backRightModule;

    private final SwerveDriveOdometry m_odometry;
    private final Field2d m_field = new Field2d();

    @Log
    private double desiredXSpeed;

    @Log
    private double desiredYSpeed;

    @Log
    private double desiredRotation;

    public Drivetrain() {
        m_pigeon = new PigeonIMU(kPigeonPort);

        m_frontLeftModule = new SwerveModule(kFLDriveTalonPort, kFLAngleTalonPort, kFLCANCoderPort, kFLCANCoderZero, "FL");
        m_frontRightModule = new SwerveModule(kFRDriveTalonPort, kFRAngleTalonPort, kFRCANCoderPort, kFRCANCoderZero, "FR");
        m_backLeftModule = new SwerveModule(kBLDriveTalonPort, kBLAngleTalonPort, kBLCANCoderPort, kBLCANCoderZero, "BL");
        m_backRightModule = new SwerveModule(kBRDriveTalonPort, kBRAngleTalonPort, kBRCANCoderPort, kBRCANCoderZero, "BR");
        
        m_odometry = new SwerveDriveOdometry(kDriveKinematics, Rotation2d.fromDegrees(m_pigeon.getFusedHeading()));

        SmartDashboard.putData("Field", m_field);

        this.zeroHeading();
    }


    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] states =
        kDriveKinematics.toSwerveModuleStates(
          fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_pigeon.getFusedHeading()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(states, kTeleopMaxSpeedMetersPerSecond);
        // for (int i = 0; i < states.length; i++) {
        //     SwerveModule module = m_modules[i];
        //     SwerveModuleState state = states[i];
        //     module.setDesiredState(state);
        // }
        m_frontLeftModule.setDesiredState(states[0]);
        m_frontRightModule.setDesiredState(states[1]);
        m_backLeftModule.setDesiredState(states[2]);
        m_backRightModule.setDesiredState(states[3]);

        this.desiredRotation = rot;
        this.desiredXSpeed = xSpeed;
        this.desiredYSpeed = ySpeed;
    }

    //used in autonomous
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, kAutoMaxSpeedMetersPerSecond);
        m_frontLeftModule.setDesiredState(desiredStates[0]);
        m_frontRightModule.setDesiredState(desiredStates[1]);
        m_backLeftModule.setDesiredState(desiredStates[2]);
        m_backRightModule.setDesiredState(desiredStates[3]);
    }
    
    // Resets the drive encoders to currently read a position of 0
    public void resetDriveEncoders(){
        m_frontLeftModule.resetDriveEncoder();
        m_backLeftModule.resetDriveEncoder();
        m_frontRightModule.resetDriveEncoder();
        m_backLeftModule.resetDriveEncoder();
    }

    @Log
    public double getHeading() {
        return m_pigeon.getFusedHeading();
    }

    // Zeroes the heading of the robot
    public void zeroHeading() {
        m_pigeon.setFusedHeading(0);
    }

    public Pose2d getPoseMeters(){
        return m_odometry.getPoseMeters();
    }
        
    public void resetOdometry(Pose2d pose){
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(m_pigeon.getFusedHeading()));
    }

    public void setDriveNeutralMode(NeutralMode mode) {
        m_frontLeftModule.getDriveMotor().setNeutralMode(mode); 
        m_frontRightModule.getDriveMotor().setNeutralMode(mode); 
        m_backLeftModule.getDriveMotor().setNeutralMode(mode); 
        m_backRightModule.getDriveMotor().setNeutralMode(mode); 
    }

    @Override
    public void periodic() {
        m_odometry.update(
            Rotation2d.fromDegrees(m_pigeon.getFusedHeading()),
            m_frontLeftModule.getState(),
            m_backRightModule.getState(),
            m_frontRightModule.getState(),
            m_backRightModule.getState());
        
        m_field.setRobotPose(this.getPoseMeters());
    }
}
