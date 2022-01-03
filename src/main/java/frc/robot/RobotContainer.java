// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveWithController;
import frc.robot.subsystems.Drivetrain;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import io.github.oblarg.oblog.annotations.Config;

public class RobotContainer {

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final WL_XboxController m_driver = new WL_XboxController(OIConstants.kDriverPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {


    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    this.configureDrivetrainCommands();
  }

  private void configureDrivetrainCommands() {
    m_drivetrain.setDefaultCommand(
      // new RunCommand(()-> {
      //   m_drivetrain.drive(
      //     -modifyAxis(m_driver.getY(Hand.kLeft)) * DriveConstants.kMaxSpeedMetersPerSecond,
      //     -modifyAxis(m_driver.getX(Hand.kLeft)) * DriveConstants.kMaxSpeedMetersPerSecond,
      //     -modifyAxis(m_driver.getX(Hand.kRight)) * DriveConstants.kMaxAngularSpeedRadiansPerSecond,
      //     true
      //   );}, m_drivetrain)
        new DriveWithController(m_driver, m_drivetrain)
      );

    m_driver.getJoystickButton(XboxController.Button.kX).whenPressed(
      new InstantCommand(m_drivetrain::zeroHeading)
    );
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
   // Create config for trajectory
    TrajectoryConfig configForward =
        new TrajectoryConfig(
                AutoConstants.kAutoMaxSpeedMetersPerSecond,
                AutoConstants.kAutoMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory forwardTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0.7, 0.3), new Translation2d(1.5, -0.3)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2, 0, new Rotation2d(0)),
            configForward);


    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveForward =
        new SwerveControllerCommand(
            forwardTrajectory,
            m_drivetrain::getPoseMeters, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drivetrain::setModuleStates,
            m_drivetrain);



    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.resetOdometry(forwardTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveForward.andThen(() -> m_drivetrain.drive(0, 0, 0, false)); 
   }

  public void configureDriveBreakMode() {
    m_drivetrain.setDriveNeutralMode(NeutralMode.Brake);
  }

  public void configureDriveCoastMode() {
    m_drivetrain.setDriveNeutralMode(NeutralMode.Coast);
  }

  public void testPeriodic() {

  }

}
