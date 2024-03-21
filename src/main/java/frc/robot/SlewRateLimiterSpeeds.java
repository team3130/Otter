// Copyright (c) FIRST and ERRORS, FRC-3130
// Open Source Software; you can modify and/or share it under the terms of

package frc.robot;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * A class that limits the rate of change of an input 2d values. Useful for implementing
 * 2-dimensional output ramps.
 */
public class SlewRateLimiterSpeeds {
    private double m_linearAccRateLimit;
    private double m_linearDecRateLimit;
    private double currentRateLimit;
    private double m_omegaRateLimit;
    private Translation2d m_prevMove;
    private double m_prevOmega;
    private double m_prevTime;
    private Translation2d prevJoystick;


    /**
     * Creates a new SlewRateLimiterSpeeds with the given linear and angular rate limits
     * and an initial value.
     *
     * @param linearAccelerationRateLimit The rate-of-change limit in the linear 2d space, in units per
     *     second. Must be positive.
     * @param omegaRateLimit The rate-of-change limit for the angular component, in units per
     *     second. Must be positive.
     * @param initialValue The initial value of the input in Chassis Speeds.
     */
    public SlewRateLimiterSpeeds(double linearAccelerationRateLimit, double linearDecelerationRateLimit, double omegaRateLimit, ChassisSpeeds initialValue) {
        m_linearAccRateLimit = linearAccelerationRateLimit;
        m_linearDecRateLimit = linearDecelerationRateLimit;
        currentRateLimit = m_linearAccRateLimit;

        prevJoystick = new Translation2d(0,0);
        m_omegaRateLimit = omegaRateLimit;
        m_prevMove = new Translation2d(initialValue.vxMetersPerSecond, initialValue.vyMetersPerSecond);
        m_prevOmega = initialValue.omegaRadiansPerSecond;
        m_prevTime = MathSharedStore.getTimestamp();
    }


    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public ChassisSpeeds calculate(ChassisSpeeds input, Translation2d joystick) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - m_prevTime;

        Translation2d inputMove = new Translation2d(input.vxMetersPerSecond, input.vyMetersPerSecond);
        double wantedNorm = inputMove.minus(m_prevMove).getNorm();

        if ( Math.abs(prevJoystick.getNorm()) > Math.abs(joystick.getNorm())){
            currentRateLimit = m_linearDecRateLimit;
        } else{
            currentRateLimit = m_linearAccRateLimit;
        }

        double allowedNorm = MathUtil.clamp(wantedNorm, 0, currentRateLimit * elapsedTime);

        m_prevMove = m_prevMove.interpolate(inputMove, allowedNorm/wantedNorm);
        m_prevOmega = MathUtil.clamp(
                input.omegaRadiansPerSecond - m_prevOmega,
                -m_omegaRateLimit * elapsedTime, // acceleration * time = velocity
                m_omegaRateLimit *elapsedTime);
        m_prevTime = currentTime;
        return new ChassisSpeeds(m_prevMove.getX(), m_prevMove.getY(), m_prevOmega);
    }
    public  double getCurrentRateLimit(){
        return currentRateLimit;
    }
    public double getM_linearAccRateLimit(){
        return m_linearAccRateLimit;
    }
    public void setM_linearAccRateLimit(double limit){
        m_linearAccRateLimit = limit;
    }
    public double getM_linearDecRateLimit(){
        return m_linearDecRateLimit;
    }
    public void setM_linearDecRateLimit(double limit){
        m_linearDecRateLimit = limit;
    }
    public double getOmega(){
        return m_omegaRateLimit;
    }
    public void setOmega(double limit){
        m_omegaRateLimit = limit;
    }



    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(ChassisSpeeds value) {
        m_prevMove = new Translation2d(value.vxMetersPerSecond, value.vyMetersPerSecond);
        m_prevOmega = value.omegaRadiansPerSecond;
        m_prevTime = MathSharedStore.getTimestamp();
    }

}

