// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.Shooter;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.NewShooter;
//import frc.robot.subsystems.Shooter;

public class ShuttleMovingSetpoint extends Command {
  private final NewShooter shooter;
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
  public ShuttleMovingSetpoint(NewShooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setTryingToShuttle(true);
    shooter.setTryingToShoot(false);

    timer.reset();
    timer.start();
    if (Constants.debugMode) {
      shooter.updateShooterPID();
    }
    shooter.configureShooterConfigs();
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterWithMomentum(timer.get());
    shooter.checkShooterAtSetpoint();
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    timer.stop();
    shooter.setTryingToShuttle(false);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}