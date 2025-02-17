import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.Chassis.PowerLimiter;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class PowerLimiterTest {
  /* @Test
    void linearTest() throws InterruptedException {
        PowerLimiter powerLimiter = new PowerLimiter(new Translation2d());
        var joystick = new Translation2d(1, 0);
        Translation2d ghostTurn = powerLimiter.calculateLinear(joystick, new double[]{0.4, 0});
        System.out.println("\n" + "X component:" + ghostTurn.getX());
        System.out.println("Y component:" + ghostTurn.getY());
        try {
            Thread.sleep(20);
        } catch (Exception ignored) {
        }
    }
} */

   /* @Test
    void rotationalTest() {
        PowerLimiter powerLimiter = new PowerLimiter(new Translation2d());
        var joystick = new Translation2d(0.8, 0.6);
        Translation2d ghostTurn = powerLimiter.calculateRotational(joystick, 0);
        System.out.println("\n" + "New Angle: " + ghostTurn.getAngle().getRadians());
        try {
            Thread.sleep(20);
        } catch (Exception e) {
        }
    }*/
}
