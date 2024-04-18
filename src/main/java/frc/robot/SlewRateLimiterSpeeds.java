// Copyright (c) FIRST and ERRORS, FRC-3130
// Open Source Software; you can modify and/or share it under the terms of

package frc.robot;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;


/**
 * A class that limits the rate of change of an input 2d values. Useful for implementing
 * 2-dimensional output ramps.
 */
public class SlewRateLimiterSpeeds {
    private double linearAccRateLimit;
    private double linearDecRateLimit;
    private double omegaRateLimit;
    private Translation2d prevMove;
    private double prevOmega;
    private double prevTime;
    private double deltaNorm;
    private double deltaRotation;


    /**
     * Creates a new SlewRateLimiterSpeeds with the given linear and angular rate limits
     * and an initial value.
     *
     * @param linearAccelerationRateLimit The rate-of-change limit in the linear 2d space, in units per
     *                                    second. Must be positive.
     * @param linearDecelerationRateLimit The rate-of-change deceleration limit in the linear 2d space, in units per
     *                                    second. Must be positive.
     * @param omegaLimit                  The rate-of-change limit for the angular component, in units per
     *                                    second. Must be positive.
     * @param initialValue                The initial value of the input in Chassis Speeds.
     */
    public SlewRateLimiterSpeeds(double linearAccelerationRateLimit, double linearDecelerationRateLimit, double omegaLimit, ChassisSpeeds initialValue) {
        linearAccRateLimit = linearAccelerationRateLimit;
        linearDecRateLimit = linearDecelerationRateLimit;
        deltaNorm = 0;
        deltaRotation = 0;
        omegaRateLimit = omegaLimit;
        prevMove = new Translation2d(initialValue.vxMetersPerSecond, initialValue.vyMetersPerSecond);
        prevOmega = initialValue.omegaRadiansPerSecond;
        prevTime = MathSharedStore.getTimestamp();
    }


    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public ChassisSpeeds calculate(ChassisSpeeds input) {
        /** time **/
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - prevTime;

        /** norm (r of polar coords)**/
        double prevNorm = prevMove.getNorm();
        Translation2d inputMove = new Translation2d(input.vxMetersPerSecond, input.vyMetersPerSecond);
        double inputNorm = inputMove.getNorm();

        deltaNorm = inputNorm- prevNorm; //desired change in norm

        double allowedChangeInNorm = MathUtil.clamp(deltaNorm,  linearDecRateLimit*elapsedTime,  linearAccRateLimit*elapsedTime);

        double allowedNormToDesired = 0;
        if (Math.abs(deltaNorm) > 0) { //dont divide by zero
            allowedNormToDesired = allowedChangeInNorm / deltaNorm;  //if (1) go all the way to wanted if (0) stay at prev
        }

        /** angle (theta of polar coords)**/
        double prevAngle = prevMove.getAngle().getRadians();
        double inputAngle = inputMove.getAngle().getRadians();

        deltaRotation = inputAngle - prevAngle; //desired change in rotation

        double allowedChangeInRotation = MathUtil.clamp(deltaRotation,  -omegaRateLimit*elapsedTime, +omegaRateLimit*elapsedTime); //allowed change in rotation

        double allowedRotationToDesired = 0;
        if (Math.abs(deltaRotation) > 0) { //dont divide by 0
            allowedRotationToDesired = allowedChangeInRotation / deltaRotation;
        }

        double usedT = Math.min(allowedNormToDesired, allowedRotationToDesired); //both are within slice take "more careful" one

        prevMove = prevMove.interpolate(inputMove, usedT); //reassign prevMove, this is what gets used on chassis and is ready for next iteration

        /** omega (angular velocity) **/
        double inputOmega = input.omegaRadiansPerSecond;
        double deltaOmega = inputOmega - prevOmega;
        prevOmega = MathUtil.clamp(
                deltaOmega,
                -omegaRateLimit * elapsedTime, // acceleration * time = velocity
                omegaRateLimit * elapsedTime);
        prevTime = currentTime;

        return new ChassisSpeeds(prevMove.getX(), prevMove.getY(), prevOmega);
    }

    public double getLinearAccRateLimit() {
        return linearAccRateLimit;
    }

    public void setLinearAccRateLimit(double limit) {
        linearAccRateLimit = limit;
    }

    public double getLinearDecRateLimit() {
        return linearDecRateLimit;
    }

    public void setLinearDecRateLimit(double limit) {
        linearDecRateLimit = limit;
    }

    public double getOmega() {
        return omegaRateLimit;
    }

    public void setOmega(double limit) {
        omegaRateLimit = limit;
    }


    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(ChassisSpeeds value) {
        prevMove = new Translation2d(value.vxMetersPerSecond, value.vyMetersPerSecond);
        prevOmega = value.omegaRadiansPerSecond;
        prevTime = MathSharedStore.getTimestamp();
    }

    public void initSendable(SendableBuilder builder) {
        if (Constants.debugMode) {
            builder.setSmartDashboardType("Slew");
            builder.addDoubleProperty("linear acc limit", this::getLinearAccRateLimit, this::setLinearAccRateLimit);
            builder.addDoubleProperty("linear decc limit", this::getLinearDecRateLimit, this::setLinearDecRateLimit);
            builder.addDoubleProperty("omega acc limit", this::getOmega, this::setOmega);

        }
    }
}

