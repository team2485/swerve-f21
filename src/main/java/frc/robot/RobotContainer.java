// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.team2485.WarlordsLib.oi.WL_XboxController;

public class RobotContainer {

  private final Drivetrain drivetrain;
  private final WL_XboxController driver;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driver = new WL_XboxController(OIConstants.DRIVER_PORT);
    drivetrain = new Drivetrain();

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    this.configureDrivetrainCommands();
  }

  private void configureDrivetrainCommands() {
    drivetrain.setDefaultCommand(
      new RunCommand(()-> {
        drivetrain.drive(
          -driver.getY(Hand.kLeft) * DriveConstants.MAX_SPEED,
          -driver.getX(Hand.kLeft) * DriveConstants.MAX_SPEED,
          -driver.getX(Hand.kRight) * DriveConstants.MAX_ANGULAR_SPEED,
          driver.getBumper(Hand.kLeft)
        );}, drivetrain)
      );
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null; 
  }
}
