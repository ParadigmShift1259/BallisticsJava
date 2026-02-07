import static edu.wpi.first.units.Units.*; // library for units, (e.g. MetersPerSecondPerSecond)
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class App {
    public static void main(String[] args) throws Exception {
        System.out.println("Team 1259 Paradigm Shift Ballistics");
        Ballistics ballistic = new Ballistics();

//        Distance distToFrontRim = Meter.of(1.47);
        Distance distToFrontRim = Meter.of(2.466600);
        Distance distIntoCone = Meter.of(0.5334);
        Distance heightAboveRim = Meter.of(1.9812);   // How far above Hub to place the shot (includes height of hub)
        Distance landingHeight = Meter.of(1.7272);
        // Expected output
        // RPM 3097
        // Shot angle 60.81
        // Lanfing angle -39.52

        // Distance distToFrontRim = Meter.of(3.72);
        // Distance distIntoCone = Meter.of(0.66);
        // Distance heightAboveRim = Meter.of(0.38);
        // Distance landingHeight = Meter.of(1.72);

        System.out.println("Inputs");
        System.out.println("distToFrontRim " + distToFrontRim);
        System.out.println("distIntoCone " + distIntoCone);
        System.out.println("heightAboveRim " + heightAboveRim);
        System.out.println("landingHeight " + landingHeight);

        AngularVelocity rpms = ballistic.CalcInitRPMs(distToFrontRim, distIntoCone, heightAboveRim, landingHeight);
        System.out.println("Outputs");
        System.out.println("Init velocity " + ballistic.getInitVelocity());
        System.out.println("RPM " + rpms);
        System.out.println("Init angle " + ballistic.getInitAngle());
        System.out.println("Landing angle " + ballistic.getLandingAngle());
    }
}
