import static edu.wpi.first.units.Units.*; // library for units, (e.g. MetersPerSecondPerSecond)

import javax.lang.model.util.ElementScanner14;

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

    }

    Distance HubHeightToMaxHeight()
    {
        double t1 = (_heightTarget.minus(_heightRobot);
        double t2 = _xInput + _xTarget;
        double t3 = _heightAboveHub - _heightRobot;
        double t4 = _xTarget * _xInput * (_xInput + _xTarget);

        double aValue = (_xInput * (_heightTarget - _heightRobot) - (_xInput + _xTarget) * (_heightAboveHub - _heightRobot)) / (_xTarget * _xInput * (_xInput + _xTarget));
        double bValue = ((_xInput + _xTarget) * (_xInput + _xTarget) * (_heightAboveHub - _heightRobot) - _xInput * _xInput * (_heightTarget - _heightRobot)) / (_xTarget * _xInput * (_xInput + _xTarget));

        _heightMax = -1.0 * bValue * bValue / (4.0 * aValue) + _heightRobot;

        // frc::SmartDashboard::PutNumber("HubHeightToMaxHeight", units::foot_t(_heightMax).to<double>());

        return _heightMax;
    }

    second_t CalcTimeOne()
    {
    _timeOne = math::sqrt(2.0 * (_heightMax - _heightRobot) / gravity);

    return _timeOne;
    }

    second_t CalcTimeTwo()
    {
    _timeTwo = math::sqrt(2.0 * (_heightMax - _heightTarget) / gravity);

    return _timeTwo;
    }

    second_t CalcTotalTime()
    {
    _timeTotal = CalcTimeOne() + CalcTimeTwo();
 
    Velocity vyfinal = _velYInit - gravity * _timeTotal;
    Velocity vxfinal = _velXInit; // No drag
    radian_t beta = units::math::atan(vyfinal / vxfinal);
    _landingAngle = beta;

    return _timeTotal;
    }

    Velocity CalcInitXVel()
    {
    _velXInit = (_xInput + _xTarget) / CalcTotalTime();

    return _velXInit;
    }

    Velocity CalcInitYVel()
    {
    _velYInit = math::sqrt(2.0 * gravity * (_heightMax - _heightRobot));

    return _velYInit;
    }

    Velocity CalcInitVel()
    {
    // _heightAboveHub = foot_t(frc::SmartDashboard::GetNumber("HeightAboveHub", 0.0));
    // _heightTarget = foot_t(frc::SmartDashboard::GetNumber("TargetHeight", 0.0));
    // _heightRobot = foot_t(frc::SmartDashboard::GetNumber("RobotHeight", 0.0));
    // _xInput = foot_t(frc::SmartDashboard::GetNumber("FloorHubDistance", 0.0));
    // _xTarget = foot_t(frc::SmartDashboard::GetNumber("TargetXDistance", 0.0));

    //_heightAboveHub = foot_t(_heightAboveHubEntry.GetDouble(10.0));
    // _heightTarget = foot_t(_heightTargetEntry.GetDouble(0.0));
    // _heightRobot = foot_t(_heightRobotEntry.GetDouble(0.0));
    // _xInput = foot_t(_xFloorDistanceEntry.GetDouble(0.0));
    // _xTarget = foot_t(_xTargetDistanceEntry.GetDouble(0.0));
        
    HubHeightToMaxHeight();

    CalcInitXVel();
    CalcInitYVel();
    
    _angleInit = math::atan(_velYInit / _velXInit);
    //printf("Angle Before Clamp %.3f\n", _angleInit.to<double>());
    if (_bClampAngle)
    {
        _angleInit = degree_t(std::clamp(_angleInit.to<double>(), minAngle.to<double>(), maxAngle.to<double>()));
    }
    // printf("Angle After Clamp %.3f\n", _angleInit);

    CalcInitVelWithAngle();
    // printf("InitVel %.3f\n", _velInit.to<double>());
    // printf("InitAngle %.3f\n", _angleInit.to<double>());

    #ifndef WINDOWS_BUILD
    _initVelEntry.SetDouble(_velInit.to<double>());
    _initAngleEntry.SetDouble(_angleInit.to<double>());
    #endif

    // frc::SmartDashboard::PutNumber("InitVel", units::feet_per_second_t(_velInit).to<double>());
    // frc::SmartDashboard::PutNumber("InitAngle", _angleInit.to<double>());

    return _velInit;
    }

    Velocity CalcInitVelWithAngle() {
    Distance totalXDist = _xInput + _xTarget;
    Distance totalYDist = _heightTarget - _heightRobot;

    _velInit = math::sqrt(gravity * totalXDist * totalXDist / (2.0 * (totalXDist * math::tan(_angleInit) - totalYDist))) / math::cos(_angleInit);
    return _velInit;
    }

    degree_t GetInitAngle()
    {
    return _angleInit;
    }

    revolutions_per_minute_t CalcInitRPMs(  Distance distance        // Floor distance (to front of cone?)
                                                        , Distance targetDist      // Target distance within cone
                                                        , Distance heightAboveHub  // Hub Height to max height
                                                        , Distance targetHeight    // Height at end point within cone
                                                    )
    {
    _xInput = distance;
    _xTarget = targetDist;
    _heightTarget = targetHeight;
    _heightAboveHub = heightAboveHub;
    if (_xTarget.to<double>() == 0.0)
    {
        //_xTarget = Distance(0.000000001);    // Dividing by this, use 1nm to avoid INF and/or NAN
        _xTarget = Distance(0.001);    // Dividing by this, use 1mm to avoid INF and/or NAN
    }

    CalcInitVel();

    _rotVelInit = radian_t(1.0) * _velInit / flywheelRadius * (2.0 + (cargoRotInertiaFrac + 1.0) / (flywheelRotInertiaFrac * massRatio));
    _rpmInit = _rotVelInit;

    #ifndef WINDOWS_BUILD
    _initRpmEntry.SetDouble(_rpmInit.to<double>());
    _setpointEntry.SetDouble(_rpmInit.to<double>() / FlywheelConstants::kGearRatio);
    // frc::SmartDashboard::PutNumber("InitRPM", _rpmInit.to<double>());
    #endif

    return _rpmInit;
    }

    radians_per_second_t QuadraticFormula(double a, double b, double c, bool subtract)
    {
    auto outPut = radians_per_second_t(0.0);
    
    if (subtract == false)
        outPut = radians_per_second_t((-1.0 * b + sqrt(b * b - 4 * a * c)) / (2 * a));
    else
        outPut = radians_per_second_t((-1.0 * b - sqrt(b * b - 4 * a * c)) / (2 * a));

    return outPut;
    }
}
