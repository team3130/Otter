// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.Shooter;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShuttleMovingSetpoint extends Command {
  private final Shooter shooter;
  private Timer timer = new Timer();
  double topStartingPoint = 0;
  double bottomStartingPoint = 0;

  double topRPSToIncrease = 0;
  double bottomRPSToIncrease = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param shooter  The subsystem used by this command.
   */
  public ShuttleMovingSetpoint(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setTryingToShuttle(true);
    shooter.setTryingToShootNotShuttle(false);

    timer.reset();
    timer.start();
    if (Constants.debugMode) {
      shooter.updatePIDValues();
    }
    shooter.configureVelocitySlots();

    topStartingPoint = shooter.getTopFlyVelocityRPS(); //momentum in the wheels creates a nonzero starting state
    bottomStartingPoint = shooter.getBottomFlyVelocityRPS();

    topRPSToIncrease = shooter.getTopShuttleVelocitySetpoint() - topStartingPoint; //ground left to cover
    bottomRPSToIncrease = shooter.getBottomShuttleVelocitySetpoint() - bottomStartingPoint;

  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if ((timer.get() / shooter.getMaxTime()) < 1) { // make sure you are not applying >100% of setpoint before incrementing
        shooter.setTopFlywheelVelocity( topStartingPoint + ((timer.get() / shooter.getMaxTime()) * topRPSToIncrease));
        shooter.setBottomFlywheelVelocity( bottomStartingPoint + ((timer.get() / shooter.getMaxTime()) * bottomRPSToIncrease));
        // setpoint = how fast you are going now + (% of time used * rps the controller has to cover to setpoint); make sure it is never over setpoint
      } else {
        shooter.setShuttleFlywheelVelocity(); //normal going straight to setpoint, assumes youve hit it previously
      }
      shooter.checkFlywheelsAtVelocitySetpoint(shooter.getTopShuttleVelocitySetpoint(), shooter.getBottomShuttleVelocitySetpoint());
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooters();
    timer.stop();
    shooter.setTryingToShuttle(false);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}