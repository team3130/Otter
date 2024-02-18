package frc.robot.errlib;

// Copyright (c) FIRST and ERRORS, FRC-3130
// Open Source Software; you can modify and/or share it under the terms of

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A class that limits the rate of change of an input 2d values. Useful for implementing
 * 2-dimensional output ramps.
 */
public class SlewRateLimiterSpeeds {
  private final double m_linearRateLimit;
  private final double m_omegaRateLimit;
  private Translation2d m_prevMove;
  private double m_prevOmega;
  private double m_prevTime;

  /**
   * Creates a new SlewRateLimiterSpeeds with the given linear and angular rate limits
   * and an initial value.
   *
   * @param linearRateLimit The rate-of-change limit in the linear 2d space, in units per
   *     second. Must be positive.
   * @param omegaRateLimit The rate-of-change limit for the angular component, in units per
   *     second. Must be positive.
   * @param initialValue The initial value of the input in Chassis Speeds.
   */
  public SlewRateLimiterSpeeds(double linearRateLimit, double omegaRateLimit, ChassisSpeeds initialValue) {
    m_linearRateLimit = linearRateLimit;
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
  public ChassisSpeeds calculate(ChassisSpeeds input) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - m_prevTime;
    Translation2d inputMove = new Translation2d(input.vxMetersPerSecond, input.vyMetersPerSecond);
    double wantedNorm = inputMove.minus(m_prevMove).getNorm();
    double allowedNorm = MathUtil.clamp(wantedNorm, 0, m_linearRateLimit * elapsedTime);
    m_prevMove = m_prevMove.interpolate(inputMove, allowedNorm/wantedNorm);
    m_prevOmega = MathUtil.clamp(
        input.omegaRadiansPerSecond - m_prevOmega,
        -m_omegaRateLimit * elapsedTime,
        m_omegaRateLimit *elapsedTime);
    m_prevTime = currentTime;
    return new ChassisSpeeds(m_prevMove.getX(), m_prevMove.getY(), m_prevOmega);
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
