/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.Constants.PidConstants;

import frc.robot.subsystems.EncoderSubsystem;
import frc.robot.Constants.EncoderConstants;

public class DriveForwardCommand extends CommandBase {
  /**
   * Creates a new DriveForwardCommand.
   */
  private final Double m_speed;
  private final DriveTrain m_subsystem;
  private final EncoderSubsystem m_encoder;
  private final GyroSubsystem m_gyro;
  public double targetAngle; // target is starting direction
  private boolean magic = false;

  public DriveForwardCommand(final DriveTrain subsystem, final Double speed, final GyroSubsystem gyro, final EncoderSubsystem encoder) {

//    public DriveForwardCommand(final DriveTrain subsystem, final GyroSubsystem gyro, final Double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_speed = speed;
    m_gyro = gyro;
    m_encoder = encoder;
    addRequirements(m_subsystem, m_gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetAngle = m_gyro.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double deltaAngle,  turningValue;
    // targetAngle += 7.2 / 2; // drive in a circle every 2 seconds: 360 degree/ 50 update per sec / 2 sec
    deltaAngle = targetAngle - m_gyro.getAngle();
    turningValue = MathUtil.clamp(deltaAngle * PidConstants.kProportionalDriveStraight, -1.0, 1.0);
    SmartDashboard.putNumber("turn", turningValue);
    SmartDashboard.putNumber("delta", deltaAngle);
    m_subsystem.preussDrive(-m_speed, -turningValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_subsystem.preussDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
