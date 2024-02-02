package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.Constants;


// Swerve module that reflects the actual swerve modules
public class SwerveModule implements Sendable {
    private final TalonFX steerMotor; // the steering motor
    private final TalonFX driveMotor; // the driving motor
    private final CANcoder absoluteEncoder; // the can encoder attached to the shaft
    private final PIDController turningPidController; // PID controller for steering
    private final double absoluteEncoderOffset; // the absolute encoder offset from where 0 is to where it thinks it is
    private final int side; // the side that the bot is on
    final VoltageOut steerMotorVoltRequest = new VoltageOut(0);
    final VoltageOut driveMotorVoltRequest = new VoltageOut(0);

    /**
     * Initializes a swerve module and its motors.
     * Initializes the steering PID controller.
     * @param side is reflective in {@link Constants}
     */
    public SwerveModule(int side) {
        steerMotor = new TalonFX(Constants.CAN.turningID[side]);
        driveMotor = new TalonFX(Constants.CAN.spinningID[side]);

        absoluteEncoder = new CANcoder(Constants.CAN.CANCoders[side]);
        turningPidController = new PIDController(Constants.Swerve.kP_Swerve[side], Constants.Swerve.kI_Swerve[side], Constants.Swerve.kD_Swerve[side]);

        steerMotor.getConfigurator().apply(new TalonFXConfiguration()); // config factory default
        steerMotor.setNeutralMode(NeutralModeValue.Brake); // Brake mode
        steerMotor.setInverted(true);

        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        driveMotor.setInverted(false);

        turningPidController.enableContinuousInput(-Math.PI, Math.PI); // wrap for circles
        turningPidController.setTolerance(0.0025, 0.05); // at position tolerance

        absoluteEncoderOffset = Constants.EncoderOffsets.kCANCoderOffsets[side];

        this.side = side;

        resetEncoders();

        String name = this.getClass().getSimpleName();
        name = name.substring(name.lastIndexOf('.') + 1);
        name += " " + side;
        SendableRegistry.addLW(this, name, name);
    }

    // returns the amount of distance the drive motor has travelled in meters
    public double getDrivePosition() {
        return driveMotor.getPosition().getValue() * Constants.Conversions.DriveRotToMeters;
    }

    // returns the position of the steering motor radians
    public double getTurningPosition() {
        return steerMotor.getPosition().getValue() * Constants.Conversions.SteerRotToRads;
    }

    // gets the velocity of the drive motor in m/s
    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValue() * Constants.Conversions.DriveRotToMeters;
    }

    // gets the speed at which the steering motor turns in radians per second
    public double getTurningVelocity() {
        return steerMotor.getVelocity().getValue() * Constants.Conversions.SteerRotToRads;
    }

    // gets the position of the steering wheel according to the absolute encoders
    public double getAbsoluteEncoderRad() {
        return Math.toRadians(absoluteEncoder.getAbsolutePosition().getValue() * 360);
    }

    // return the position of the steering wheel in degrees
    public double getAbsoluteEncoderDegrees() {
        return absoluteEncoder.getAbsolutePosition().getValue();
    }

    // updates steering PID controller
    // param p is the new kP value
    public void updatePValue(double p) {
        turningPidController.setP(p);
    }

    // updates the steering PID controller
    // param d the new kD value
    public void updateDValue(double d) {
        turningPidController.setD(d);
    }

    /**
     * Resets the relative encoders according the absolute encoder involving the offset
     */
    public void resetEncoders() {
        steerMotor.setPosition((getAbsoluteEncoderRad() - absoluteEncoderOffset) / Constants.Conversions.SteerRotToRads);
        // m_driveMotor.setSelectedSensorPosition(0);
    }

    /**
     * Whether the wheels are zeroed or not
     * @return custom at set-point logic for the PID controller
     */
    public boolean wheelsZeroed() {
        Rotation2d pos = new Rotation2d(getTurningPosition());
        return (pos.getDegrees() > 355 || pos.getDegrees() < 5) && getTurningVelocity() < 0.05;
    }

    /**
     * @return the current swerve module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /**
     * Default stop method to stop the motors
     */
    public void stop() {
        steerMotor.setControl(steerMotorVoltRequest.withOutput(0));
        driveMotor.setControl(driveMotorVoltRequest.withOutput(0));
    }

    /**
     * Set the desired swerve module state
     * @param state the state to set the swerve modules to
     */
    public void setDesiredState(SwerveModuleState state) {
        // dead-band
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // max turn is 90 degrees optimization
        state = SwerveModuleState.optimize(state, getState().angle);
        // percent output of the drive motor that the swerve controller wants you to go to by the physical max speed the bot can travel
        // TODO: underneath set control voltage output is not real
        // m_driveMotor.setControl(driveMotorVoltRequest.withOutput(12d* (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond)));
        driveMotor.setVoltage((10d* (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond)));
        // set the steering motor based off the output of the PID controller
        steerMotor.setVoltage(4d * turningPidController.calculate(Math.IEEEremainder(getTurningPosition(), Math.PI * 2), state.angle.getRadians()));
    }

    /**
     * Turns the motors to an angle
     * @param setpoint in radians
     */
    public void turnToAngle(double setpoint) {
        steerMotor.setVoltage(12d * turningPidController.calculate(Math.IEEEremainder(getTurningPosition(), Math.PI * 2), setpoint));
    }

    /**
     * Get the position of the swerve module
     * @return gets with turning position and velocity
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    /**
     * Whether the pid controller is at the set point.
     * @return whether pid is done
     */
    public boolean PIDisDone() {
        return turningPidController.atSetpoint();
    }

    /**
     * Gets the P value for steering motors
     * @return the P value
     */
    public double getPValue() {
        return turningPidController.getP();
    }

    /**
     * Gets the P value for steering motors
     * @return the P value
     */
    public double getDValue() {
        return turningPidController.getD();
    }

    /**
     * Setter for the P value
     * @param newP the new p value
     */
    public void setPValue(double newP) {
        turningPidController.setP(newP);
    }

    /**
     * Setter for derby derv
     * @param newD the new D value
     */
    public void setDValue(double newD) {
        turningPidController.setD(newD);
    }

    public void setIValue(double newI) {
        turningPidController.setI(newI);
    }

    public double getIValue() {
        return turningPidController.getI();
    }

    /**
     * The string representation of the swerve module
     * @return "Swerve module side: " + sideNumber: int
     */
    public String toString() {
        return "Swerve module side: " + side;
    }

    /**
     * Builds the sendable for shuffleboard
     * @param builder sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Module " + side);
/*        builder.addDoubleProperty("Drive position", this::getDrivePosition, null);
        builder.addDoubleProperty("Drive velocity", this::getDriveVelocity, null);*/
        builder.addDoubleProperty("Steer position", this::getSteerPositionWrapped, null);
/*        builder.addDoubleProperty("Steer velocity", this::getTurningVelocity, null);
        builder.addDoubleProperty("Steer relative", this::getRelativePositionDegrees, null);
        builder.addDoubleProperty("Absolute encoder position", this::getAbsoluteEncoderDegrees, null);*/
        builder.addDoubleProperty("Swerve P " + side, this::getPValue, this::setPValue);
        builder.addDoubleProperty("Swerve I " + side, this::getIValue, this::setIValue);
        builder.addDoubleProperty("Swerve D " + side, this::getDValue, this::setDValue);
    }

    public double getSteerPositionWrapped() {
        return Math.IEEEremainder(getRelDegrees(), 360);
    }

    public double getRelativePositionDegrees() {
        return Math.toDegrees(getTurningPosition());
    }

    public double getRelDegrees() {
        return Math.toDegrees(getTurningPosition());
    }
}
