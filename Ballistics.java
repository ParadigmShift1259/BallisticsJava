import static edu.wpi.first.units.Units.*; // library for units, (e.g. MetersPerSecondPerSecond)
import edu.wpi.first.units.measure.*; // library for types (e.g. LinearAcceleration)

public class Ballistics {
    private static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(9.81);
    private static final Mass flywheelMass = Pound.of(1.5);
    
    private static final Distance flywheelRadius = Inch.of(2.0);
    private static final double flywheelRotInertiaFrac = 1.0 / 2.0; // used to be scalar if this causes issues
    //private static final double flywheelRotInertiaFrac = 0.6659;  // based on the SDS brass hollow flywheel with MOI 4 [pound][square inches]
    //private static final MomentOfInertia flywheelRotInertia = KilogramSquareMeters.of(flywheelMass.in(Kilogram) * flywheelRotInertiaFrac * flywheelRadius.in(Meter) * flywheelRadius.in(Meter));

    private static final Mass fuelMass = Pound.of(0.5);
    private static final Distance fuelRadius = Inch.of(5.91 / 2);
    //private static final double fuelRotInertiaFrac = 2.0 / 3.0;  // 2/3 Mr^2 hollow sphere
    private static final double fuelRotInertiaFrac = 2.0 / 5.0;  // 2/5 Mr^2 solid sphere
    //private static final MomentOfInertia fuelRotInertia = fuelRotInertiaFrac * fuelMass * fuelRadius * fuelRadius;

    private static final double massRatio = flywheelMass.in(Kilogram) / fuelMass.in(Kilogram);
    //private static final double rotInertiaRatio = flywheelRotInertia.in(KilogramSquareMeters) / fuelRotInertia.in(KilogramSquareMeters);

    private static final Angle maxAngle = Degree.of(75.0);
    private static final Angle minAngle = Degree.of(30.3);

    //private static final Distance robotHeight = Foot.of(3.0);
    private static final Distance defaultTargetDist = Foot.of(2.5);

    //private static final Distance defaultTargetHeight = Foot.of(80.0);
    private static final Distance defaultHeightAboveHub = Foot.of(6.0).plus(Inch.of(6.0));

    private Time _timeOne = Seconds.of(0.0);
    private Time _timeTwo = Seconds.of(0.0);
    private Time _timeTotal = Seconds.of(0.0);

    private Distance _heightAboveHub = defaultHeightAboveHub;
    private Distance _heightRobot = Inch.of(30.0);
    private Distance _heightTarget = Foot.of(6.0).minus(Inch.of(4.0));
    private Distance _heightMax = Meter.of(16.0);

    private Distance _xInput = Meter.of(0.0);
    private Distance _xTarget = defaultTargetDist;

    private LinearVelocity _velXInit = MetersPerSecond.of(0.0);
    private LinearVelocity _velYInit = MetersPerSecond.of(0.0);
    private LinearVelocity _velInit = MetersPerSecond.of(0.0);

    private Angle _angleInit = Degree.of(0.0);
    private Angle _landingAngle = Degree.of(0.0);

    private AngularVelocity _rotVelInit = RadiansPerSecond.of(0.0);
    private AngularVelocity _rpmInit = RPM.of(0.0);

    boolean _bClampAngle = true;

    public Ballistics() {
    }

    Distance HubHeightToMaxHeight()
    {
        double xInput = _xInput.magnitude();
        double hTarg = (_heightTarget.minus(_heightRobot)).baseUnitMagnitude();
        double dist = xInput + _xTarget.magnitude();
        double hAbove = _heightAboveHub.magnitude() - _heightRobot.baseUnitMagnitude();
        double x = _xTarget.magnitude() * xInput * (dist);

        double aValue = (xInput * hTarg - dist * hAbove) / x;
        double bValue = (dist * dist * hAbove - xInput * xInput * hTarg) / x;

        //double aValue = (_xInput.magnitude() * (_heightTarget.baseUnitMagnitude() - _heightRobot.baseUnitMagnitude()) - (_xInput.magnitude() + _xTarget.magnitude()) * (_heightAboveHub.magnitude() - _heightRobot.baseUnitMagnitude())) / (_xTarget.magnitude() * _xInput.magnitude() * (_xInput.magnitude() + _xTarget.magnitude()));
        //double bValue = ((_xInput.magnitude() + _xTarget.magnitude()) * (_xInput.magnitude() + _xTarget.magnitude()) * (_heightAboveHub.magnitude() - _heightRobot.baseUnitMagnitude()) - _xInput.magnitude() * _xInput.magnitude() * (_heightTarget.baseUnitMagnitude() - _heightRobot.baseUnitMagnitude())) / (_xTarget.magnitude() * _xInput.magnitude() * (_xInput.magnitude() + _xTarget.magnitude()));

        _heightMax = Meter.of(bValue * bValue / (4.0 * aValue) + _heightRobot.baseUnitMagnitude());

        System.out.println("max height " + _heightMax.magnitude());

        return _heightMax;
    }

    Time CalcTimeOne()
    {
        // C++ Code: _timeOne = math::sqrt(2.0 * (_heightMax - _heightRobot) / gravity);
        // double hMax = _heightMax.baseUnitMagnitude();
        // double hBot = _heightRobot.baseUnitMagnitude();
        // double h = hMax - hBot;
        // double t1 = Math.sqrt(2.0 * h / gravity.magnitude());
        _timeOne = Seconds.of(Math.sqrt(2.0 * (_heightMax.magnitude() - _heightRobot.baseUnitMagnitude()) / gravity.magnitude()));
        
        System.out.println("time 1 " + _timeOne.magnitude());

        return _timeOne;
    }

    Time CalcTimeTwo()
    {
        // C++ Code: _timeTwo = math::sqrt(2.0 * (_heightMax - _heightTarget) / gravity);
        _timeTwo = Seconds.of(Math.sqrt(2.0 * (_heightMax.magnitude() - _heightTarget.baseUnitMagnitude()) /gravity.magnitude()));

        System.out.println("time 2 " + _timeTwo.magnitude());

        return _timeTwo;
    }

    Time CalcTotalTime()
    {

    /*
     *  C++ Code: 
     * 
        m_timeTotal = CalcTimeOne() + CalcTimeTwo();

        return m_timeTotal;
     */

        _timeTotal = CalcTimeOne().plus(CalcTimeTwo());

        System.out.println("total time " + _timeTotal.magnitude());

        return _timeTotal;
    }

    LinearVelocity CalcInitXVel()
    {
        /*
         * C++ Code:
         *   // Without drag, v(t) = v0
            // x(t) = v0 * t
            // init vx = "total x dist" over time
            m_velXInit = (m_xInput + m_xTarget) / CalcTotalTime();

            return m_velXInit;
         */
        _velXInit = MetersPerSecond.of((_xInput.magnitude() + _xTarget.magnitude()) / CalcTotalTime().magnitude());

        System.out.println("Init Vel X " + _velXInit.magnitude());

        return _velXInit;
    }

    LinearVelocity CalcInitYVel()
    {
        /*
         * C++ Code:
         *     // vy only affected by gravity
                // square root of 2gh where h is the highest point
                // Derived from h = 1/2 V0^2/g
                m_velYInit = math::sqrt(2.0 * gravity * (m_heightMax - m_heightRobot));

            return m_velYInit;
         */
        _velYInit = MetersPerSecond.of(Math.sqrt(2.0 * gravity.magnitude()* (_heightMax.magnitude() - _heightRobot.baseUnitMagnitude())));

        System.out.println("Init Vel Y " + _velYInit.magnitude());

        return _velYInit;
    }

    LinearVelocity CalcInitVel()
    {
        HubHeightToMaxHeight();

        CalcInitXVel();
        CalcInitYVel();
    
        _angleInit = Degree.of(Math.atan(_velYInit.magnitude() / _velXInit.magnitude()));   

        System.out.println("Init Angle " + _angleInit.magnitude());

        if (_bClampAngle && minAngle.magnitude() < maxAngle.magnitude())
        {
            _angleInit = Degree.of(clamp(_angleInit.magnitude(), minAngle.magnitude(), maxAngle.magnitude()));
            // If we clamp the angle, we need to recalc the vx and vy as inputs to CalcInitVelWithAngle()
            _velYInit = MetersPerSecond.of(_velInit.magnitude() * Math.sin(_angleInit.magnitude()));
            _velXInit = MetersPerSecond.of(_velInit.magnitude() * Math.cos(_angleInit.magnitude()));
            System.out.println("Init Angle Clamped " + _angleInit.magnitude());
            System.out.println("Init Vel X Clamped " + _velXInit.magnitude());
            System.out.println("Init Vel Y Clamped " + _velYInit.magnitude());
        }

        CalcInitVelWithAngle();

        LinearVelocity vyfinal = MetersPerSecond.of(_velYInit.magnitude() - gravity.magnitude() * _timeTotal.magnitude());
        LinearVelocity vxfinal = _velXInit; // No drag
        Angle beta = Radian.of(Math.atan(vyfinal.magnitude() / vxfinal.magnitude()));
        _landingAngle = beta;

        return _velInit;
    }

    private double clamp(double min, double max, double value) {
        double result = value;
        if (value < min){
            result = min;
        }
        if (value > max){
            result = max;
        }
        return result;
    }

    LinearVelocity CalcInitVelWithAngle() 
    {
        Distance totalXDist = _xInput.plus(_xTarget);
        Distance totalYDist = _heightTarget.minus(_heightRobot);

        _velInit = MetersPerSecond.of(Math.sqrt(gravity.magnitude() * totalXDist.magnitude() * totalXDist.magnitude() / (2.0 * (totalXDist.magnitude() * Math.tan(_angleInit.magnitude()) - totalYDist.magnitude()))) / Math.cos(_angleInit.magnitude()));

        System.out.println("Init Vel " + _velInit.magnitude());

        return _velInit;
    }

    Angle getInitAngle()
    {
        return _angleInit;
    }

    Angle getLandingAngle()
    {
        return _landingAngle;
    }

    LinearVelocity getInitVelocity() {
        return _velInit;
    }

    AngularVelocity CalcInitRPMs(  Distance distance        // Floor distance (to front of cone?)
                                 , Distance targetDist      // Target distance within cone
                                 , Distance heightAboveHub  // Hub Height to max height
                                 , Distance targetHeight    // Height at end point within cone
                                )
    {
        _xInput = distance;
        _xTarget = targetDist;
        _heightTarget = targetHeight;
        _heightAboveHub = heightAboveHub;

        if (_xTarget.magnitude() == 0.0)
        {
            //_xTarget = Distance(0.000000001);    // Dividing by this, use 1nm to avoid INF and/or NAN
            _xTarget = Meters.of(0.001);    // Dividing by this, use 1mm to avoid INF and/or NAN
        }

        CalcInitVel();

        // C++ Code:
        // m_rotVelInit = radian_t(1.0) * m_velInit / m_flywheelRadius * (2.0 + (fuelRotInertiaFrac + 1.0) / (flywheelRotInertiaFrac * m_massRatio));
        // *** TODO: I AM REALLY UNSURE IF THIS CONVERSION IS CORRECT IN TERMS OF THE MATH. NEED TO DOUBLE CHECK
        _rotVelInit = RadiansPerSecond.of(Radian.of(1.0).magnitude() * _velInit.magnitude() / flywheelRadius.magnitude() * (2.0 + (fuelRotInertiaFrac + 1.0) / (flywheelRotInertiaFrac * massRatio)));
        _rpmInit = _rotVelInit;

        return _rpmInit;
    }

    AngularVelocity QuadraticFormula(double a, double b, double c, boolean subtract)
    {
        AngularVelocity outPut = RadiansPerSecond.of(0.0);

        if (subtract == false)
            outPut = RadiansPerSecond.of((-1.0 * b + Math.sqrt(b * b - 4 * a * c)) / (2 * a));
        else
            outPut = RadiansPerSecond.of((-1.0 * b - Math.sqrt(b * b - 4 * a * c)) / (2 * a));

        return outPut;
    }
}
