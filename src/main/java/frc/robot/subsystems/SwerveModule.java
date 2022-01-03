package frc.robot.subsystems;

import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonFX;
import frc.team2485.WarlordsLib.motorcontrol.WL_TalonFX;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.WarlordsLib.sensors.TalonEncoder;
import frc.team2485.WarlordsLib.sensors.TalonEncoder.TalonEncoderType;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.ShuffleboardContainerWrapper;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule implements Loggable {

    private final WPI_TalonFX m_driveMotor;
    private final WPI_TalonFX m_turningMotor;
    private final CANCoder m_turningEncoder;

    private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPDrive, 0, 0);

    SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(ModuleConstants.ksVoltsDrive,
            ModuleConstants.kvVoltSecondsPerMeterDrive);

    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(ModuleConstants.kPTurning, 0,
            0.1, new TrapezoidProfile.Constraints(ModuleConstants.kModuleMaxAngularSpeedRadiansPerSecond,
                    ModuleConstants.kModuleMaxModuleAngularAccelerationRadiansPerSecondSquared));

    SimpleMotorFeedforward m_turningFeedforward = new SimpleMotorFeedforward(ModuleConstants.ksVoltsTurning,
            ModuleConstants.kvVoltSecondsPerMeterTurning);

    private final String m_moduleID;

    public SwerveModule(int driveMotorID, int turningMotorID, int turningEncoderID, Rotation2d zero, String moduleID) {

        this.m_moduleID = moduleID;

        this.m_driveMotor = new WPI_TalonFX(driveMotorID);
        this.m_turningMotor = new WPI_TalonFX(turningMotorID);

        this.m_turningEncoder = new CANCoder(turningEncoderID);

        m_turningEncoder.configMagnetOffset(-zero.getDegrees());
        m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public void addCustomLogging(ShuffleboardContainerWrapper container) {
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(this.getSpeedMetersPerSecond(), this.getHeading());
    }

    @Log(name = "Heading Degrees")
    private double getHeadingDegrees() {
        return m_turningEncoder.getAbsolutePosition();
    }

    private Rotation2d getHeading() {
        return Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition());
    }

    @Log(name = "Speed meters per second")
    private double getSpeedMetersPerSecond() {
        return m_driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveDistMetersPerPulse;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        double currentSpeedMetersPerSecond = this.getSpeedMetersPerSecond();
        Rotation2d currentHeading = this.getHeading();

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentHeading);

        final double driveOutputVolts = m_drivePIDController.calculate(currentSpeedMetersPerSecond,
                state.speedMetersPerSecond) + m_driveFeedforward.calculate(state.speedMetersPerSecond);

        final double turningOutputVolts = m_turningPIDController.calculate(currentHeading.getRadians(),
                state.angle.getRadians())
                + m_turningFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.setVoltage(driveOutputVolts);
        m_turningMotor.setVoltage(turningOutputVolts);

        SmartDashboard.putNumber("turn setpoint velocity", m_turningPIDController.getSetpoint().velocity);
        SmartDashboard.putNumber("drive output", driveOutputVolts);
        SmartDashboard.putNumber("turn output", turningOutputVolts);

    }

    public void resetDriveEncoder() {
        m_driveMotor.setSelectedSensorPosition(0);
    }

    public WPI_TalonFX getDriveMotor() {
        return m_driveMotor;
    }

    public WPI_TalonFX getAngleMotor() {
        return m_turningMotor;
    }

    @Override
    public String configureLogName() {
        return m_moduleID;
    }

    @Override
    public LayoutType configureLayoutType() {
        return BuiltInLayouts.kGrid;
    }

    @Override
    public int[] configureLayoutSize() {
        int[] size = {3,3};
        return size;
      }
    
}
