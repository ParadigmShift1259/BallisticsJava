import static edu.wpi.first.units.Units.*; // library for units, (e.g. MetersPerSecondPerSecond)
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class App {
    public static void main(String[] args) throws Exception {
        System.out.println("Team 1259 Paradigm Shift Ballistics");
        Ballistics ballistic = new Ballistics();

        Distance distToFrontRim = Meter.of(2.0);
        Distance distIntoCone = Meter.of(0.5);
        Distance heightAboveRim = Meter.of(1.0);
        Distance landingHeight = Meter.of(1.5);

        System.out.println("Inputs");
        System.out.println("distToFrontRim " + distToFrontRim.magnitude());
        System.out.println("distIntoCone " + distIntoCone.magnitude());
        System.out.println("heightAboveRim " + heightAboveRim.magnitude());
        System.out.println("landingHeight " + landingHeight.magnitude());

        AngularVelocity rpms = ballistic.CalcInitRPMs(distToFrontRim, distIntoCone, heightAboveRim, landingHeight);
        System.out.println("Outputs");
        System.out.println("Init velocity " + ballistic.getInitVelocity().magnitude());
        System.out.println("RPM " + rpms.magnitude());
        System.out.println("Init angle " + ballistic.getInitAngle().magnitude());
        System.out.println("Landing angle " + ballistic.getLandingAngle().magnitude());
    }
}
