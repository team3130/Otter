package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
    final VelocityVoltage driveMotorVeloVoltRequest = new VelocityVoltage(0).withSlot(0);

    Slot0Configs slot0Configs;
    private double slot0_kS = 0;
    private double slot0_kV = 0;
    private double slot0_kA = 0;
    private double slot0_kP = 0;
    private double slot0_kI = 0;
    private double slot0_kD = 0;


    //private double steeringVoltage = 4d;
    //private double drivingVoltage = 10d;

    /**
     * Initializes a swerve module and its motors.
     * Initializes the steering PID controller.
     * @param side is reflective in {@link Constants}
     */
    public SwerveModule(int side) {
        steerMotor = new TalonFX(Constants.Swerve.turningID[side]);
        driveMotor = new TalonFX(Constants.Swerve.spinningID[side]);

        absoluteEncoder = new CANcoder(Constants.Swerve.CANCoders[side]);
        turningPidController = new PIDController(Constants.Swerve.kP_Swerve[side], Constants.Swerve.kI_Swerve[side], Constants.Swerve.kD_Swerve[side]);

        steerMotor.getConfigurator().apply(new TalonFXConfiguration()); // config factory default
        steerMotor.setNeutralMode(NeutralModeValue.Brake); // Brake mode
        steerMotor.setInverted(true);

        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        driveMotor.setInverted(false);

        turningPidController.enableContinuousInput(-Math.PI, Math.PI); // wrap for circles
        turningPidController.setTolerance(0.0025, 0.05); // at position tolerance

        absoluteEncoderOffset = Constants.SwerveEncoderOffsets.kCANCoderOffsets[side];
        this.side =side;

        slot0Configs = new Slot0Configs();

        slot0Configs.kS = slot0_kS; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = slot0_kV; // 1/(rps) - A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = slot0_kP; // 1/rps - An error of 1 rps results in 0.11 V output
        slot0Configs.kI = slot0_kI; // 1/rot - output per unit of integrated error in velocity (output/rotation)
        slot0Configs.kD = slot0_kD; // output per unit of error derivative in velocity (output/ (rps/s))

        driveMotor.getConfigurator().apply(slot0Configs);

        driveMotor.getConfigurator().apply((new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).
                withSupplyCurrentThreshold(40).withSupplyTimeThreshold(0)), 0.01);

        steerMotor.getConfigurator().apply((new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).
                withSupplyCurrentThreshold(20).withSupplyTimeThreshold(0)), 0.01);

        driveMotor.getConfigurator().apply((new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).
                withStatorCurrentLimit(40).withSupplyTimeThreshold(0)), 0.01);

        steerMotor.getConfigurator().apply((new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).
                withStatorCurrentLimit(20).withSupplyTimeThreshold(0)), 0.01);


        resetEncoders();

        String name = this.getClass().getSimpleName();
        name = name.substring(name.lastIndexOf('.') + 1);
        name += " " + side;
        SendableRegistry.addLW(this, name, name);
    }

    // returns the amount of distance the drive motor has travelled in meters
    public double getDrivePosition() {
        //return driveMotor.getPosition().getValue() * Constants.Conversions.DriveRotToMeters;
        return driveMotor.getPosition().getValue() * Constants.SwerveConversions.driveRotToMeters;
    }

    // returns the position of the steering motor radians
    public Rotation2d getTurningPosition() {
        // return steerMotor.getPosition().getValue() * Constants.Conversions.SteerRotToRads;

        return new Rotation2d(steerMotor.getPosition().getValue() * Constants.SwerveConversions.steerRotToRads);
    }

    public double getTurningPositionRadians() {
        return steerMotor.getPosition().getValue() * Constants.SwerveConversions.steerRotToRads;
    }

    // gets the velocity of the drive motor in m/s
    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValue() * Constants.SwerveConversions.driveRotToMeters * 10d;
    }

    // gets the speed at which the steering motor turns in radians per second
    public double getTurningVelocity() {
        return steerMotor.getVelocity().getValue() * Constants.SwerveConversions.steerRotToRads * 10d;
    }

    // gets the position of the steering wheel according to the absolute encoders
    public double getAbsoluteEncoderRads() {
        return Math.toRadians(absoluteEncoder.getAbsolutePosition().getValue() * 360);// / (Math.PI *2);
    }

    public double getAbsoluteEncoderRotation() {
        return absoluteEncoder.getAbsolutePosition().getValue();
    }

    public void updatePValue(double p) {
        turningPidController.setP(p);
    }
    public void updateDValue(double d) {
        turningPidController.setD(d);
    }

    /**
     * Resets the relative encoders according the absolute encoder involving the offset
     */
    public void resetEncoders() {
        steerMotor.setPosition((getAbsoluteEncoderRads() - absoluteEncoderOffset) / Constants.SwerveConversions.steerRotToRads);
        //steerMotor.setPosition((getAbsoluteEncoderRad() - absoluteEncoderOffset) / Constants.Conversions.SteerRotToRads);
    }

    /**
     * Whether the wheels are zeroed or not
     * @return custom at set-point logic for the PID controller
     */
    public boolean wheelsZeroed() {
        Rotation2d pos = getTurningPosition();
        return (pos.getDegrees() > 355 || pos.getDegrees() < 5) && getTurningVelocity() < 0.05;
    }

    /**
     * @return the current swerve module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getTurningPosition());
    }

    // Default stop method to stop the motors
    public void stop() {
        steerMotor.setControl(steerMotorVoltRequest.withOutput(0));
        driveMotor.setControl(driveMotorVoltRequest.withOutput(0));
    }

    /**
     * Set the desired swerve module state
     * @param state the state to set the swerve modules to
     */
    public void setTeleopDesiredState(SwerveModuleState state) {
        // dead-band
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // max turn is 90 degrees optimization
        state = SwerveModuleState.optimize(state, getState().angle);
        // percent output of the drive motor that the swerve controller wants you to go to by the physical max speed the bot can travel
        // m_driveMotor.setControl(driveMotorVoltRequest.withOutput(12d* (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond)));
        driveMotor.setVoltage(Constants.Swerve.maxDriveVoltage * (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond));
        // set the steering motor based off the output of the PID controller
        steerMotor.setVoltage(Constants.Swerve.maxSteerVoltage * turningPidController.calculate(Math.IEEEremainder(getTurningPositionRadians(), Math.PI * 2), state.angle.getRadians()));
    }

    public void setAutonDesiredState(SwerveModuleState state) {
        // dead-band
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // max turn is 90 degrees optimization
        state = SwerveModuleState.optimize(state, getState().angle);
        // percent output of the drive motor that the swerve controller wants you to go to by the physical max speed the bot can travel
        // m_driveMotor.setControl(driveMotorVoltRequest.withOutput(12d* (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond)));
        driveMotor.setVoltage((10d* (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond)));
        // set the steering motor based off the output of the PID controller
        steerMotor.setVoltage(4d * turningPidController.calculate(Math.IEEEremainder(getTurningPositionRadians(), Math.PI * 2), state.angle.getRadians()));
    }

    public void setTeleopDesiredStateVelo(SwerveModuleState state){
        // dead-band
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        updatePIDValues(); //mostly for testing

        // max turn is 90 degrees optimization
        state = SwerveModuleState.optimize(state, getState().angle);
        // set velocity to speed from state for drive motor
        driveMotor.setControl(driveMotorVeloVoltRequest.withVelocity(state.speedMetersPerSecond));
        // set the steering motor based off the output of the PID controller
        steerMotor.setVoltage(Constants.Swerve.maxSteerVoltage * turningPidController.calculate(Math.IEEEremainder(getTurningPositionRadians(), Math.PI * 2), state.angle.getRadians()));
    }

    public void driveAtVelocity(double velocity){
        updatePIDValues();
        driveMotor.setControl(driveMotorVeloVoltRequest.withVelocity(velocity));
    }

    /**
     * Turns the motors to an angle
     * @param setpoint in radians
     */
    public void turnToAngle(double setpoint) {
        //steerMotor.setVoltage(12d * turningPidController.calculate(Math.IEEEremainder(getTurningPositionRadians(), Math.PI * 2), setpoint));
        steerMotor.setVoltage(12d * turningPidController.calculate(Math.IEEEremainder(getTurningPositionRadians(), Math.PI * 2), Math.toRadians(setpoint)));
    }


    /**
     * Get the position of the swerve module
     * @return gets with turning position and velocity
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getTurningPosition());
    }

    /**
     * Whether the pid controller is at the set point.
     * @return whether pid is done
     */
    public boolean turningPIDisDone() {
        return turningPidController.atSetpoint();
    }

    public void setTurningPValue(double newP) {
        turningPidController.setP(newP);
    }
    public void setTurningDValue(double newD) {
        turningPidController.setD(newD);
    }
    public void setTurningIValue(double newI) {
        turningPidController.setI(newI);
    }

    public double getTurningPValue() {
        return turningPidController.getP();
    }
    public double getTurningIValue() {
        return turningPidController.getI();
    }
    public double getTurningDValue() {
        return turningPidController.getD();
    }

    /*
    public double getSteeringVoltage() { return steeringVoltage; }
    public double getDrivingVoltage() { return drivingVoltage; }
    public void setSteeringVoltage(double volt) { this.steeringVoltage = volt; }
    public void setDrivingVoltage(double volt){this.drivingVoltage=volt;}


     */

    /**
     * The string representation of the swerve module
     * @return "Swerve module side: " + sideNumber: int
     */
    /*public String toString() {
        return "Swerve module side: " + (side+1);
    }*/
    public int getRealSide(){
        int real = side + 1;
        return real;
    }
    public double getSteerRotations(){
        return steerMotor.getPosition().getValue();
    }


    public double getSwerveSteeringVoltage() {
        return Constants.Swerve.maxSteerVoltage;
    }
    public void setSwerveSteeringVoltage(double lol) {
        Constants.Swerve.maxSteerVoltage = lol;
    }
    public double getSwerveDrivingVoltage() {
        return Constants.Swerve.maxDriveVoltage;
    }
    public void getSwerveDrivingVoltage(double lol) {
        Constants.Swerve.maxDriveVoltage = lol;
    }

    public double getModuleSteeringSupplyCurrent() { return steerMotor.getSupplyCurrent().getValue();}
    public double getModuleDrivingSupplyCurrent() { return driveMotor.getSupplyCurrent().getValue();}
    public void updatePIDValues() {
        slot0Configs.kS = slot0_kS; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = slot0_kV; // 1/(rps) - A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = slot0_kP; // 1/rps - An error of 1 rps results in 0.11 V output
        slot0Configs.kI = slot0_kI; // 1/rot - output per unit of integrated error in velocity (output/rotation)
        slot0Configs.kD = slot0_kD; // output per unit of error derivative in velocity (output/ (rps/s))
        driveMotor.getConfigurator().apply(slot0Configs);
    }

    public double getSlot0_kV() { return slot0_kV;}
    public double getSlot0_kA() { return slot0_kA;}
    public double getSlot0_kP() { return slot0_kP;}
    public double getSlot0_kI() { return slot0_kI;}
    public double getSlot0_kD() { return slot0_kD;}

    public void setSlot0_kV(double value) { slot0_kV = value;}
    public void setSlot0_kA(double value) { slot0_kA = value;}
    public void setSlot0_kP(double value) { slot0_kP = value;}
    public void setSlot0_kI(double value) { slot0_kI = value;}
    public void setSlot0_kD(double value) { slot0_kD = value;}


    /**
     * Builds the sendable for shuffleboard
     * @param builder sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        if (Constants.debugMode) {
            builder.setSmartDashboardType("Swerve Module " + (getRealSide()));
            builder.addDoubleProperty("Drive velocity", this::getDriveVelocity, null);
            builder.addDoubleProperty("Steer position", this::getSteerRotations, null);
            builder.addDoubleProperty("Drive position", this::getDrivePosition, null);
            builder.addDoubleProperty("Absolute encoder position", this::getAbsoluteEncoderRads, null);
            builder.addDoubleProperty("Constant Steering voltage", this::getSwerveSteeringVoltage, this::setSwerveSteeringVoltage);
            builder.addDoubleProperty("Constant Driving voltage", this::getSwerveDrivingVoltage, this::getSwerveDrivingVoltage);

            builder.addDoubleProperty("Steering Module Current Supply", this::getModuleSteeringSupplyCurrent, null);
            builder.addDoubleProperty("Driving Module Current Supply", this::getModuleDrivingSupplyCurrent, null);

            //    builder.addDoubleProperty("Steering Voltage", this::getSteeringVoltage, this::setSteeringVoltage);
            // builder.addDoubleProperty("Driving voltage", this:: getDrivingVoltage, this::setDrivingVoltage);
/*        builder.addDoubleProperty("Steer velocity", this::getTurningVelocity, null);
        builder.addDoubleProperty("Steer relative", this::getRelativePositionDegrees, null);
        */
            builder.addDoubleProperty("Swerve P " + getRealSide(), this::getTurningPValue, this::setTurningPValue);
            builder.addDoubleProperty("Swerve I " + getRealSide(), this::getTurningIValue, this::setTurningIValue);
            builder.addDoubleProperty("Swerve D " + getRealSide(), this::getTurningDValue, this::setTurningDValue);

            builder.addDoubleProperty("Velo V " + getRealSide(), this::getSlot0_kV, this::setSlot0_kV);
            builder.addDoubleProperty("Velo A " + getRealSide(), this::getSlot0_kA, this::setSlot0_kA);
            builder.addDoubleProperty("Velo P " + getRealSide(), this::getSlot0_kP, this::setSlot0_kP);
            builder.addDoubleProperty("Velo I " + getRealSide(), this::getSlot0_kI, this::setSlot0_kI);
            builder.addDoubleProperty("Velo D " + getRealSide(), this::getSlot0_kD, this::setSlot0_kD);
        }
    }

    public double getSteerPositionWrapped() {
        return Math.IEEEremainder(getRelDegrees(), 360);
    }

    public double getRelativePositionDegrees() {
        return Math.toDegrees(getTurningPositionRadians());
    }

    public double getRelDegrees() {
        return Math.toDegrees(getTurningPositionRadians());
    }
}
