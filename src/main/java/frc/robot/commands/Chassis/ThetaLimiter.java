package frc.robot.commands.Chassis;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class ThetaLimiter {
    private double prevTime;
    private final double magvalue = .5;
    private double posOmegaLimit;
    private double negOmegaLimit;
    public Rotation2d ghostTheta;
    private Rotation2d prevTheta;

    public ThetaLimiter(double limitConstant, Rotation2d joyStick){
        posOmegaLimit = limitConstant;
        negOmegaLimit = -limitConstant;
        prevTheta = joyStick;
        prevTime = MathSharedStore.getTimestamp();
    }
    public Rotation2d calculate(Rotation2d desiredTheta, double magnitude) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - prevTime;
        ghostTheta = desiredTheta;
        if (magnitude > posOmegaLimit * elapsedTime / Math.PI) {
            double allowedAngle = posOmegaLimit * elapsedTime / magnitude;
            double diffTheta = Math.abs(desiredTheta.minus(prevTheta).getRadians());
            if (diffTheta > 0) {
                double t = allowedAngle / diffTheta;
                ghostTheta = prevTheta.interpolate(desiredTheta, t);
            }
        }
        prevTheta = ghostTheta;
        return ghostTheta;
    }
}
