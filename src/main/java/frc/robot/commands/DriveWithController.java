// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;

public class DriveWithController extends CommandBase {
    private final WL_XboxController m_driver;
    private final Drivetrain m_drivetrain;

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(OIConstants.kDriveSlewRate);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(OIConstants.kDriveSlewRate);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(OIConstants.kDriveSlewRate);

    public DriveWithController(WL_XboxController driver, Drivetrain drivetrain) {
        this.m_driver = driver;
        this.m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final double xSpeed = 
            -m_xspeedLimiter.calculate(
                Deadband.deadband(m_driver.getY(Hand.kLeft), OIConstants.kDriveDeadband))
            * DriveConstants.kMaxSpeedMetersPerSecond;
        
        final double ySpeed = 
            -m_yspeedLimiter.calculate(
                Deadband.deadband(m_driver.getX(Hand.kLeft), OIConstants.kDriveDeadband))
            * DriveConstants.kMaxSpeedMetersPerSecond;
        
        final double rot = 
            -m_rotLimiter.calculate(
                Deadband.deadband(m_driver.getX(Hand.kRight), OIConstants.kDriveDeadband))
            * DriveConstants.kMaxAngularSpeedRadiansPerSecond;
        
        final boolean fieldRelative = !m_driver.getYButton();
        m_drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
