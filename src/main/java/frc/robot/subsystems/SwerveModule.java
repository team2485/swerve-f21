package frc.robot.subsystems;

import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.control.WL_ProfiledPIDController;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonFX;
import frc.team2485.WarlordsLib.motorcontrol.WL_TalonFX;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.WarlordsLib.sensors.TalonEncoder;
import frc.team2485.WarlordsLib.sensors.TalonEncoder.TalonEncoderType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule{
    private final PIDTalonFX driveMotor;
    private final PIDTalonFX angleMotor;

    private final CANCoder angleEncoder; 

    private final String moduleID;
    public SwerveModule( 
        int driveMotorID,
        int angleMotorID,
        int angleEncoderID, 
        Rotation2d offset, 
        String moduleID) {
        
        this.moduleID = moduleID;

        this.driveMotor = new PIDTalonFX(driveMotorID, ControlMode.Velocity);
        this.angleMotor = new PIDTalonFX(angleMotorID, ControlMode.Position);

        this.angleEncoder = new CANCoder(angleEncoderID);
        CANCoderConfiguration canCoderConfig = new CANCoderConfiguration();
        canCoderConfig.magnetOffsetDegrees = offset.getDegrees();
        angleEncoder.configAllSettings(canCoderConfig);

        TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();

        // Use the CANCoder as the remote sensor for the primary TalonFX PID
        angleMotorConfig.remoteFilter0.remoteSensorDeviceID = angleEncoderID;
        angleMotorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        angleMotorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        angleMotor.configAllSettings(angleMotorConfig);

        this.driveMotor.setDistancePerPulse(ModuleConstants.DRIVE_DIST_PER_PULSE);

        RobotConfigs.getInstance().addConfigurable(moduleID + "swerveAngleController", angleMotor);

        
        
    }

    public void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        tab.add(driveMotor);
        tab.addNumber("Encoder [" + moduleID + "] absolute position", angleEncoder::getAbsolutePosition);
        tab.add("Angle Motor [" + moduleID + "]", angleMotor);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getEncoderVelocity(), new Rotation2d(angleEncoder.getAbsolutePosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentRotation = Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());

        SwerveModuleState state = 
            SwerveModuleState.optimize(desiredState, currentRotation);
        
        Rotation2d rotationDelta = state.angle.minus(currentRotation);

        double deltaTicks = (rotationDelta.getDegrees() / 360) * ModuleConstants.CANCODER_CPR;
        double currentTicks = angleEncoder.getPosition() / angleEncoder.configGetFeedbackCoefficient();
        double desiredTicks = currentTicks + deltaTicks;

        angleMotor.runPID(desiredTicks);

        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / Constants.DriveConstants.MAX_SPEED);
        
    }

    
}
