package frc.robot.commands.Chassis;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

import java.util.function.DoubleSupplier;

public class ThetaLimiter implements Sendable {
    private double prevTime;
    private double posOmegaLimit;
    private Translation2d prevState;
    public double posMagLimit;
    public static final double massConstant = 65;
    public static final double maxLinearEnergyConstant = 8;
    public static final double maxRotationalEnergyConstant = 8;
    public static final double maxCentripetalAcceleration = 8;

    public ThetaLimiter(double limitConstant, double posMagLimit, Translation2d joyStick){
        posOmegaLimit = limitConstant;
        this.posMagLimit = posMagLimit;
        prevState = joyStick;
        prevTime = MathSharedStore.getTimestamp();

        String name = this.getClass().getSimpleName();
        name = name.substring(name.lastIndexOf('.') + 1);
        SendableRegistry.addLW(this, name, name);
    }

    public Translation2d calculateLinear(Translation2d desiredState) {
        
        ChassisSpeeds chassisSpeeds = new Chassis().getRobotRelativeSpeeds();
        double[] currentLinearSpeeds = {chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond};
        double[] desiredLinearSpeeds = {desiredState.getX(), desiredState.getY()};

        if(currentLinearSpeeds[0]/Math.abs(currentLinearSpeeds[0]) != desiredLinearSpeeds[0]/Math.abs(desiredLinearSpeeds[0])) {
            desiredLinearSpeeds[0] = 0;
        }
        if(currentLinearSpeeds[1]/Math.abs(currentLinearSpeeds[1]) != desiredLinearSpeeds[1]/Math.abs(desiredLinearSpeeds[1])) {
            desiredLinearSpeeds[1] = 0;
        }

        double projScalar = (currentLinearSpeeds[0]*desiredLinearSpeeds[0] + currentLinearSpeeds[1]*desiredLinearSpeeds[1])/(currentLinearSpeeds[0]*currentLinearSpeeds[0] + currentLinearSpeeds[1]*currentLinearSpeeds[1]);
        double[] projDesiredOnCurrent = {projScalar*currentLinearSpeeds[0], projScalar*currentLinearSpeeds[1]};
        double[] normDesiredOnCurrent = {currentLinearSpeeds[0] - projDesiredOnCurrent[0],  currentLinearSpeeds[1] - projDesiredOnCurrent[1]};
        double linearEnergyChange = massConstant/2 * (Math.pow(projDesiredOnCurrent[0], 2) + Math.pow(projDesiredOnCurrent[1], 2) - Math.pow(currentLinearSpeeds[0], 2) - Math.pow(currentLinearSpeeds[1], 2));
        double absProj = Math.sqrt(Math.pow(projDesiredOnCurrent[0], 2) + Math.pow(projDesiredOnCurrent[1], 2));
        double absNorm = Math.sqrt(Math.pow(normDesiredOnCurrent[0], 2) + Math.pow(normDesiredOnCurrent[1], 2));

        if(projDesiredOnCurrent[0] > currentLinearSpeeds[0]) {
            if(linearEnergyChange > maxLinearEnergyConstant) {
                double maxAbsProj = Math.sqrt(maxLinearEnergyConstant + massConstant/2 * (Math.pow(currentLinearSpeeds[0], 2) + Math.pow(currentLinearSpeeds[1], 2)));
                projDesiredOnCurrent[0] *= maxAbsProj/absProj;
                projDesiredOnCurrent[1] *= maxAbsProj/absProj;
            }
        }

        if(absNorm > maxCentripetalAcceleration) {
            normDesiredOnCurrent[0] *= maxCentripetalAcceleration/absNorm;
            normDesiredOnCurrent[1] *= maxCentripetalAcceleration/absNorm;
        }

        return new Translation2d(projDesiredOnCurrent[0] + normDesiredOnCurrent[0], projDesiredOnCurrent[1] + normDesiredOnCurrent[1]);
    }
    public double getPosOmegaLimit() {
        return posOmegaLimit;
    }

    public void setPosOmegaLimit(double posOmegaLimit) {
        this.posOmegaLimit = posOmegaLimit;
    }

    public double getPosMagLimit() {
        return posMagLimit;
    }

    public void setPosMagLimit(double posMagLimit) {
        this.posMagLimit = posMagLimit;
    }
    @Override public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("thetaLimiter");
        builder.addDoubleProperty("thetaLimit", this::getPosOmegaLimit, this::setPosOmegaLimit);
        builder.addDoubleProperty("posMagLimit", this::getPosMagLimit, this::setPosMagLimit);

    }
}
