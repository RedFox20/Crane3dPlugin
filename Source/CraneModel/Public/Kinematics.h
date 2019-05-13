// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#pragma once
#include <cmath>

namespace crane3d
{
    //////////////////////////////////////////////////////////////////////
    // Basic physics relations

    template<class T> struct Unit
    {
        double Value = 0.0;
        static const Unit Zero;
        Unit operator+(Unit b) const { return { Value + b.Value }; }
        Unit operator-(Unit b) const { return { Value - b.Value }; }
        bool operator>(Unit b) const { return Value > b.Value; }
        bool operator<(Unit b) const { return Value < b.Value; }
        Unit operator+(double x) const { return { Value + x }; }
        Unit operator-(double x) const { return { Value - x }; }
        Unit operator*(double x) const { return { Value * x }; }
        Unit operator/(double x) const { return { Value / x }; }
        bool operator>(double b) const { return Value > b; }
        bool operator<(double b) const { return Value < b; }
        bool operator==(double b) const { return Value == b; }
        bool operator!=(double b) const { return Value != b; }
        double operator/(Unit b) const { return Value / b.Value; }
        Unit operator-() const { return { -Value }; }
    };
    struct _Force {};
    struct _Mass  {};
    struct _Accel {};
    using Force = Unit<_Force>;
    using Mass  = Unit<_Mass>;
    using Accel = Unit<_Accel>;

    inline double sign(double x) { return x > 0 ? 1.0 : (x < 0 ? -1.0 : 0.0); }
    template<class T> inline double sign(Unit<T> x) { return sign(x.Value); }
    template<class T> inline Unit<T> abs(Unit<T> x) { return { std::abs(x.Value) }; }
    template<class T> inline Unit<T> operator*(double x, Unit<T> u) { return { x * u.Value }; }
    template<class T> inline bool operator>(double x, Unit<T> u) { return x > u.Value; }

    // F = ma
    inline Force operator*(Mass m, Accel a) { return { m.Value * a.Value }; }
    inline Force operator*(Accel a, Mass m) { return { m.Value * a.Value }; }
    // a = F/m
    inline Accel operator/(Force F, Mass m) { return { F.Value / m.Value }; }

    constexpr Force operator""_N (long double newtons) { return { double(newtons) }; }
    constexpr Mass  operator""_kg(long double kilos)   { return { double(kilos)   }; }
    constexpr Accel operator""_ms2(long double accel)  { return { double(accel)   }; }

    constexpr Force operator""_N (unsigned long long newtons) { return { double(newtons) }; }
    constexpr Mass  operator""_kg(unsigned long long kilos)   { return { double(kilos)   }; }
    constexpr Accel operator""_ms2(unsigned long long accel)  { return { double(accel)   }; }

    //////////////////////////////////////////////////////////////////////

    // compute new velocity using Euler's method:
    // v' = v + a*dt
    inline double integrate_euler_velocity(double v0, Accel a, double dt)
    {
        return v0 + a.Value*dt;
    }

    // compute new position using Euler's method:
    // x' = x + v'*dt
    inline double integrate_euler_pos(double x, double v1, double dt)
    {
        return x + v1*dt;
    }

    // integrate position using Velocity Verlet method:
    // x' = x + v*dt + (a*dt^2)/2
    inline double integrate_verlet_pos(double x, double v, Accel a, double dt)
    {
        return x + v*dt + a.Value*dt*dt*0.5;
    }

    // integrate velocity using Velocity Verlet method:
    // v' = v + (a0+a1)*0.5*dt
    inline double integrate_verlet_vel(double v, Accel a0, Accel a1, double dt)
    {
        return v + (a0.Value + a1.Value)*0.5*dt;
    }
    
    // calculate average velocity from position change
    // v_avg = dx / dt
    inline double average_velocity(double x1, double x2, double dt)
    {
        return (x2 - x1) / dt;
    }

    //////////////////////////////////////////////////////////////////////

    inline double clamp(double x, double min, double max)
    {
        if (x <= min) return min;
        if (x >= max) return max;
        return x;
    }

    // dampens values that are very close to 0.0
    inline double dampen(double x)
    {
        return std::abs(x) < 0.001 ? 0.0 : x;
    }

    inline Force dampen(Force force)
    {
        return { std::abs(force.Value) < 0.001 ? 0.0 : force.Value };
    }

    inline bool inside_limits(double x, double min, double max)
    {
        return (min+0.001) < x && x < (max-0.001);
    }

    //////////////////////////////////////////////////////////////////////
}
