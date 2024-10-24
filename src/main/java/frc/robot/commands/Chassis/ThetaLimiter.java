package frc.robot.commands.Chassis;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

public class ThetaLimiter implements Sendable {
    private double prevTime;
    private double posOmegaLimit;
    private Translation2d prevState;

    public ThetaLimiter(double limitConstant, Translation2d joyStick){
        posOmegaLimit = limitConstant;
        prevState = joyStick;
        prevTime = MathSharedStore.getTimestamp();

        String name = this.getClass().getSimpleName();
        name = name.substring(name.lastIndexOf('.') + 1);
        SendableRegistry.addLW(this, name, name);
    }

    public Translation2d calculate(Translation2d desiredState) {
        double magnitude = desiredState.getNorm();
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - prevTime;
        Translation2d ghostStick = desiredState;
        double minMagValue = posOmegaLimit / Math.PI;
        if (magnitude > minMagValue) {
            double allowedAngle = posOmegaLimit * elapsedTime / magnitude;
            double diffTheta = Math.abs(desiredState.getAngle().minus(prevState.getAngle()).getRadians());
            if (diffTheta > allowedAngle) {
                double t = allowedAngle / diffTheta;
                ghostStick = prevState.interpolate(desiredState, t);
            }
        }
        if (desiredState.getNorm() > prevState.getNorm()){

        }

        prevState = ghostStick;
        prevTime = currentTime;
        return prevState;
    }
    public double getPosOmegaLimit() {
        return posOmegaLimit;
    }

    public void setPosOmegaLimit(double posOmegaLimit) {
        this.posOmegaLimit = posOmegaLimit;
    }
    @Override public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("thetaLimiter");
        builder.addDoubleProperty("thetaLimit", this::getPosOmegaLimit, this::setPosOmegaLimit);
        }
}
