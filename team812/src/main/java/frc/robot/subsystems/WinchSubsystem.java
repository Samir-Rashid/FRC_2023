/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class WinchSubsystem extends SubsystemBase {
  /**
   * Creates a new HookSubsystem.
   */
  private final WPI_TalonSRX m_winch = new WPI_TalonSRX(CANConstants.kHookMotor);

  public WinchSubsystem() {
    stop();
    m_winch.configFactoryDefault();
    m_winch.setNeutralMode(NeutralMode.Brake);
  }

  public void forward() {
    m_winch.set(1.0);
  }

  public void reverse() {
    m_winch.set(-1.0);
  }

  public void stop() {
    m_winch.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}