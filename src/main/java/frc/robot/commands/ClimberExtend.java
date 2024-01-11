// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ClimberExtend extends Command {
  private final Climber m_climber;
  private double speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberExtend(Climber climber, double speed) {
    m_climber = climber;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_climber.BrokeLimit()) {
      m_climber.resetEncoders();
      m_climber.setZeroed();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = -RobotContainer.m_WeaponsGamepad.getRawAxis(1); // get value of joystick
    if(m_climber.LimitSwitch() && speed < 0) {
      speed = 0;
    }
    else if ((m_climber.getPosition() >= m_climber.getMaxExtensionTicks()) && speed > 0) {
      speed = 0;
    }

    m_climber.runMotor(-speed);

    if(m_climber.LimitSwitch()){
      m_climber.resetEncoders();
      m_climber.setZeroed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
