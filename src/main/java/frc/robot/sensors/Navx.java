package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Navx {
    private static Navx pInstance;
    private static AHRS navX;
    private static boolean navXPresent;
    private double angle = 0d;

    // TODO: private static double zeroPitch = Constants.Balance.defaultPitchZero;

    private Navx() {
        try {
            //Connect to navX Gyro on MXP port.
            navX = new AHRS(SPI.Port.kMXP);
            navXPresent = true;
        } catch (Exception ex) {
            //If connection fails log the error and fall back to encoder based angles.
            String str_error = "Error instantiating navX from MXP: " + ex.getLocalizedMessage();
            DriverStation.reportError(str_error, true);
            navXPresent = false;
        }
    }

    public static Navx GetInstance() {
        if (pInstance == null) pInstance = new Navx();
        return pInstance;
    }

    public static void resetNavX(){
        navX.reset();
        navX.zeroYaw();
    }

    /**
     * Returns the current angle of the Navx. If the Navx is not present, will return -1.
     *
     * @return angle in degrees
     */
    public static double getAngle() {
        if (navXPresent) {
            return Math.IEEEremainder((navX.getAngle() + 360) * (Constants.kNavxReversed ? -1.0 : 1.0), 720); // converts navx heading angle infinite values to -360 to 360 even tho thats stupid
        }
        return -1;
    }

    /**
     * get the angle in radians
     * @return the angle as a rotation 2d (in radians)
     */
    public static Rotation2d getRotation() {
        return (navXPresent) ? navX.getRotation2d() : new Rotation2d(-1);
    }

    /**
     * Returns the current yaw of the Navx. If the Navx is not present, will return NaN.
     *
     * @return angle in degrees (-180, 180)
     */
    public static double getYaw() {
        if (navXPresent) {
            return navX.getYaw();
        }
        return Double.NaN;
    }
    
    /**
     * Returns the current pitch of the Navx. If the Navx is not present, will return NaN.
     *
     * @return angle in degrees (-180, 180)
     */
    public static double getPitch() {
        if (navXPresent) {
            return navX.getPitch();
        }
        return Double.NaN;
    }

    /**
     * Returns the rotation rate of the Navx along the Y axis (Pitch). If the Navx is not present, will return NaN.
     *
     * @return angle in degrees (-180, 180)
     */
    public static double getPitchVelocity() {
        if (navXPresent) {
            return navX.getRawGyroX();
        }
        return Double.NaN;
    }

    /**
     * Returns the current roll of the Navx. If the Navx is not present, will return NaN.
     *
     * @return angle in degrees (-180, 180)
     */
    public static double getRoll() {
        if (navXPresent) {
            return navX.getRoll();
        }
        return Double.NaN;
    }

    /**
     * Returns the rotation rate of the Navx along the Y axis (Roll). If the Navx is not present, will return NaN.
     *
     * @return angle in degrees (-180, 180)
     */
    public static double getRollVelocity() {
        if (navXPresent) {
            return navX.getRawGyroY();
        }
        return Double.NaN;
    }

    /**
     * Returns the current rate of change of the robots heading
     *
     * <p> getRate() returns the rate of change of the angle the robot is facing,
     * with a return of negative one if the gyro isn't present on the robot,
     * as calculating the rate of change of the angle using encoders is not currently being done.
     *
     * @return the rate of change of the heading of the robot in degrees per second.
     */
    public static double getRate() {
        if (navXPresent) return navX.getRate() * (Constants.kNavxReversed ? -1.0 : 1.0);
        return -1;
    }

    /**
     * Returns the current heading of the Navx. Range is wrapped onto 0 to 360
     * If the Navx is not present, will return -1.
     *
     * @return angle in degrees
     */
    public static double getHeading() {
        return Math.IEEEremainder(getAngle(), 360);
    }

    public static boolean getNavxPresent() {
        return navXPresent;
    }

    // TODO: public static void setPitchZero(double val) { zeroPitch = val;}

    // TODO: public static double getZeroPitch() { return zeroPitch;}

    public static void outputToShuffleboard() {
        SmartDashboard.putNumber("NavX angle", getRotation().getDegrees());
        SmartDashboard.putNumber("NavX Yaw", getYaw());
        SmartDashboard.putNumber("NavX Pitch", getPitch());
        SmartDashboard.putNumber("NavX Roll", getRoll());
    }
}