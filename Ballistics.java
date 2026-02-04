import static edu.wpi.first.units.Units.*; // library for units, (e.g. MetersPerSecondPerSecond)

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.*; // library for types (e.g. LinearAcceleration)

public class Ballistics {
    
    private static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(9.81);
    private static final Mass flywheelMass = Pound.of(2.8);
    
    private static final Distance flywheelRadius = Inch.of(2.0);
    private static final double flywheelRotInertiaFrac = 1.0 / 2.0; // used to be scalar if this causes issues

    private static final MomentOfInertia flywheelRotInertia = KilogramSquareMeters.of(flywheelMass.in(Kilogram) * flywheelRotInertiaFrac * flywheelRadius.in(Meter) * flywheelRadius.in(Meter));

    private static final Mass cargoMass = Ounce.of(9.5);
    private static final Distance cargoRadius = Inch.of(4.75);
    private static final double cargoRotInertiaFrac = 2.0 / 3.0; // used to be scalar
    private static final MomentOfInertia cargoRotInertia = KilogramSquareMeters.of(cargoRotInertiaFrac * cargoMass.in(Kilogram) * cargoRadius.in(Meter) * cargoRadius.in(Meter));

    private static final double massRatio = flywheelMass.in(Kilogram) / cargoMass.in(Kilogram);
    private static final double rotInertiaRatio = flywheelRotInertia.in(KilogramSquareMeters) / cargoRotInertia.in(KilogramSquareMeters);

    private static final Angle maxAngle = Degree.of(60.0);
    private static final Angle minAngle = Degree.of(33.3);

    private static final Distance robotHeight = Foot.of(3.0);
    private static final Distance defaultTargetDist = Foot.of(2.5);

    private static final Distance defaultTargetHeight = Foot.of(80.0);
    private static final Distance defaultHeightAboveHub = Foot.of(9.2);

    private Time _timeOne = Seconds.of(0.0);
    private Time _timeTwo = Seconds.of(0.0);
    private Time _timeTotal = Seconds.of(0.0);

    private Distance _heightAboveHub = defaultHeightAboveHub;
    private Distance _heightRobot = Foot.of(3.0);
    private Distance _heightTarget = Foot.of(8.67);
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
        //System.out.print("Hello.");
        _heightRobot = Foot.of(3.0);
        _heightTarget = Foot.of(8.67);
    }

    Distance HubHeightToMaxHeight()
    {
        double t1 = (_heightTarget.minus(_heightRobot)).magnitude();
        double t2 = _xInput.magnitude() + _xTarget.magnitude();
        double t3 = _heightAboveHub.magnitude() - _heightRobot.magnitude();
        double t4 = _xTarget.magnitude() * _xInput.magnitude() * (_xInput.magnitude() + _xTarget.magnitude());

        double aValue = (_xInput.magnitude() * (_heightTarget.magnitude() - _heightRobot.magnitude()) - (_xInput.magnitude() + _xTarget.magnitude()) * (_heightAboveHub.magnitude() - _heightRobot.magnitude())) / (_xTarget.magnitude() * _xInput.magnitude() * (_xInput.magnitude() + _xTarget.magnitude()));
        double bValue = ((_xInput.magnitude() + _xTarget.magnitude()) * (_xInput.magnitude() + _xTarget.magnitude()) * (_heightAboveHub.magnitude() - _heightRobot.magnitude()) - _xInput.magnitude() * _xInput.magnitude() * (_heightTarget.magnitude() - _heightRobot.magnitude())) / (_xTarget.magnitude() * _xInput.magnitude() * (_xInput.magnitude() + _xTarget.magnitude()));

        _heightMax = Meter.of(-1.0 * bValue * bValue / (4.0 * aValue) + _heightRobot.magnitude());

        // frc::SmartDashboard::PutNumber("HubHeightToMaxHeight", units::foot_t(_heightMax).to<double>());

        return _heightMax;
    }

    Time CalcTimeOne()
    {
        // C++ Code: _timeOne = math::sqrt(2.0 * (_heightMax - _heightRobot) / gravity);
        _timeOne = Seconds.of(Math.sqrt(2.0 * (_heightMax.magnitude() - _heightRobot.magnitude()) / gravity.magnitude()));
        return _timeOne;
    }

    Time CalcTimeTwo()
    {
        // C++ Code: _timeTwo = math::sqrt(2.0 * (_heightMax - _heightTarget) / gravity);
        _timeTwo = Seconds.of(Math.sqrt(2.0 * (_heightMax.magnitude() - _heightTarget.magnitude()) /gravity.magnitude()));
        return _timeTwo;
    }

    Time CalcTotalTime()
    {

    /*
     *  C++ Code: 
     * 
        m_timeTotal = CalcTimeOne() + CalcTimeTwo();

        // Estimate the landing angle
        // final vy = v0 - gt
        meters_per_second_t vyfinal = m_velYInit - gravity * m_timeTotal;
        meters_per_second_t vxfinal = m_velXInit; // No drag
        radian_t beta = units::math::atan(vyfinal / vxfinal);
        m_landingAngle = beta;
        qDebug("m_landingAngle %.3f CalcTotalTime()", m_landingAngle);

        return m_timeTotal;
     */

        _timeTotal = CalcTimeOne().plus(CalcTimeTwo());

        // TODO: Why is this calculation in this method? Surely it should be its own method, as calculating the landing angle is a byproduct of this method
        LinearVelocity vyfinal = MetersPerSecond.of(_velYInit.magnitude() - gravity.magnitude() * _timeTotal.magnitude());
        LinearVelocity vxfinal = _velXInit; // No drag
        Angle beta = Radian.of(Math.atan(vyfinal.magnitude() / vxfinal.magnitude()));
        _landingAngle = beta;
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
        _velYInit = MetersPerSecond.of(Math.sqrt(2.0 * gravity.magnitude()* (_heightMax.magnitude() - _heightRobot.magnitude())));

        return _velYInit;
    }

    LinearVelocity CalcInitVel()
    {
        HubHeightToMaxHeight();

        CalcInitXVel();
        CalcInitYVel();
    
        _angleInit = Radian.of(Math.atan(_velYInit.magnitude() / _velXInit.magnitude()));   

        if (_bClampAngle)
        {
            _angleInit = Degree.of(clamp(_angleInit.magnitude(), minAngle.magnitude(), maxAngle.magnitude()));
        }

        CalcInitVelWithAngle();

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
        return _velInit;
    }

    Angle GetInitAngle()
    {
        return _angleInit;
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
        _rotVelInit = RadiansPerSecond.of(Radian.of(1.0).magnitude() * _velInit.magnitude() / flywheelRadius.magnitude() * (2.0 + (cargoRotInertiaFrac + 1.0) / (flywheelRotInertiaFrac * massRatio)));
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
