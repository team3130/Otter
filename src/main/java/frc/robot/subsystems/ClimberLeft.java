package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberLeft extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */

    private final DigitalInput limitSwitch;

    private final WPI_TalonSRX motor;

    private boolean ratchetDir = true;

    public ClimberLeft() {


        motor = new WPI_TalonSRX(Constants.Climber.kLMotor);
        motor.configFactoryDefault();
        motor.setInverted(false);

        limitSwitch = new DigitalInput(Constants.Climber.kRLimitSwitch);
    }

    public boolean brokeLeft() {
        return !limitSwitch.get();
    }

    public void setSpeedLeft(double speed) {
        if (!ratchetDir) {
            speed *= -1;
        }
        motor.set(ControlMode.PercentOutput, speed);
    }

    public void stopLeft() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    public void setRatchetDir(boolean Skibidi) {
        ratchetDir = Skibidi;
    }

    public boolean getRatchetDirection(){
        return ratchetDir;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("ClimberBrokeLeft", this::brokeLeft, null);
        builder.addBooleanProperty("LeftRatchetDirection", this::getRatchetDirection, this::setRatchetDir);
    }
}
