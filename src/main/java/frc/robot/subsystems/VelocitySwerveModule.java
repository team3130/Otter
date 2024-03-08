package frc.robot.subsystems;

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
public class VelocitySwerveModule implements Sendable {
    private final TalonFX steerMotor; // the steering motor
    private final TalonFX driveMotor; // the driving motor
    private final CANcoder absoluteEncoder; // the can encoder attached to the shaft
    private final PIDController turningPidController; // PID controller for steering
    private final double absoluteEncoderOffset; // the absolute encoder offset from where 0 is to where it thinks it is
    private final int side; // the side that the bot is on
    final VoltageOut steerMotorVoltRequest = new VoltageOut(0);
    final VoltageOut driveMotorVoltRequest = new VoltageOut(0);

    final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0).withSlot(0);
    private double driveFeedForwardVolt = 0;
    Slot0Configs slot0Configs; // gains for drive velocity
    private double slot0_kS = 0; // DONT USE KS
    private double slot0_kV = 0.115; // OLD VALUE: 0.135;
    private double slot0_kP = 0.6; // OLD VALUE: 0.3;
    private double slot0_kI = 0; // OLD VALUE: 0
    private double slot0_kD = 0.2; // OLD VALUE: 0.01;

    //private double steeringVoltage = 4d;
    //private double drivingVoltage = 10d;

    /**
     * Initializes a swerve module and its motors.
     * Initializes the steering PID controller.
     * @param side is reflective in {@link Constants}
     */
    public VelocitySwerveModule(int side) {
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



        resetEncoders();

        String name = this.getClass().getSimpleName();
        name = name.substring(name.lastIndexOf('.') + 1);
        name += " " + side;
        SendableRegistry.addLW(this, name, name);
    }

    public void configureVelocitySlots() {
        slot0Configs.kS = slot0_kS; // Add 0.05 V output to overcome static friction

        slot0Configs.kV = slot0_kV; // 1/(rps) - A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = slot0_kP; // 1/rps - An error of 1 rps results in 0.11 V output
        slot0Configs.kI = slot0_kI; // 1/rot - output per unit of integrated error in velocity (output/rotation)
        slot0Configs.kD = slot0_kD; // output per unit of error derivative in velocity (output/ (rps/s))
    }

    public void updateVelocityPID() {
        driveMotor.getConfigurator().apply(slot0Configs);
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
        configureVelocitySlots();
        updateVelocityPID();
        driveMotor.setControl(driveVelocityRequest.withVelocity(state.speedMetersPerSecond/Constants.SwerveConversions.wheelCircumference).withFeedForward(driveFeedForwardVolt));
        //driveMotor.setVoltage(Constants.Swerve.maxDriveVoltage * (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond));
        // TODO: positional controller by phoenix eventually
        steerMotor.setVoltage(Constants.Swerve.maxSteerVoltage * turningPidController.calculate(Math.IEEEremainder(getTurningPositionRadians(), Math.PI * 2), state.angle.getRadians()));
    }



    // returns the amount of distance the drive motor has travelled in meters
    public double getDrivePosition() {
        return driveMotor.getPosition().getValue() * Constants.SwerveConversions.driveRotToMeters;
    }

    // returns the position of the steering motor radians
    public Rotation2d getTurningPosition() {
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

    /**
     * Resets the relative encoders according the absolute encoder involving the offset
     */
    public void resetEncoders() {
        steerMotor.setPosition((getAbsoluteEncoderRads() - absoluteEncoderOffset) / Constants.SwerveConversions.steerRotToRads);
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
    public boolean PIDisDone() {
        return turningPidController.atSetpoint();
    }

    public void setPValue(double newP) {
        turningPidController.setP(newP);
    }
    public void setDValue(double newD) {
        turningPidController.setD(newD);
    }
    public void setIValue(double newI) {
        turningPidController.setI(newI);
    }

    public double getPValue() {
        return turningPidController.getP();
    }
    public double getIValue() {
        return turningPidController.getI();
    }
    public double getDValue() {
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

    /**
     * Builds the sendable for shuffleboard
     * @param builder sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        if (Constants.debugMode) {
            builder.setSmartDashboardType("Swerve Module " + (getRealSide()));
            // builder.addDoubleProperty("Drive velocity", this::getDriveVelocity, null);
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
            builder.addDoubleProperty("Swerve P " + getRealSide(), this::getPValue, this::setPValue);
            builder.addDoubleProperty("Swerve I " + getRealSide(), this::getIValue, this::setIValue);
            builder.addDoubleProperty("Swerve D " + getRealSide(), this::getDValue, this::setDValue);
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
