// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator implements AutoCloseable {
  
  private double m_ElevatorKp = 0.1;

  // Standard classes for controlling our elevator
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          m_ElevatorKp,
          Constants.kElevatorKi,
          Constants.kElevatorKd,
          new TrapezoidProfile.Constraints(2.45, 2.45));
  private final ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          Constants.kElevatorkS,
          Constants.kElevatorkG,
          Constants.kElevatorkV,
          Constants.kElevatorkA);

  private final CANSparkMax m_motor = new CANSparkMax(Constants.elevatorCanID, MotorType.kBrushless);

  /** Subsystem constructor. */
  public Elevator() {

    if (RobotBase.isSimulation()) {
    }

    if (RobotBase.isReal()) {
    }

    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(false);

    SmartDashboard.putNumber("m_kElevatorKp", m_ElevatorKp);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal) {

    m_ElevatorKp = SmartDashboard.getNumber("m_kElevatorKp", m_ElevatorKp);
    if (Constants.kElevatorKp_max < m_ElevatorKp)
    {
      m_ElevatorKp = Constants.kElevatorKp_max;
    }
    else if (m_ElevatorKp < 0)
    {
      m_ElevatorKp = 0;
    }
    m_controller.setP(m_ElevatorKp);

    m_controller.setGoal(goal);

    double pos = m_motor.getEncoder().getPosition();
    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(pos);
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
  
    SmartDashboard.putNumber("pos error", m_controller.getPositionError());
    SmartDashboard.putNumber("vel error", m_controller.getVelocityError());
    SmartDashboard.putNumber("pos", pos);
    SmartDashboard.putNumber("pidOutput", pidOutput);
    SmartDashboard.putNumber("feedforwardOutput", feedforwardOutput);
    feedforwardOutput = 0;

    m_motor.setVoltage(pidOutput + feedforwardOutput);
  }

  @Override
  public void close() {
    m_motor.setVoltage(0);
    m_motor.close();
  }
}
