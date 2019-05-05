// Copyright (c) 2019 - Jorma Rebane Crane3D
// Distributed under MIT License
#pragma once

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
        Unit operator+(double x) const { return { Value + x }; }
        Unit operator-(double x) const { return { Value - x }; }
        Unit operator*(double x) const { return { Value * x }; }
        Unit operator/(double x) const { return { Value / x }; }
        bool operator>(double b) const { return Value > b; }
        bool operator<(double b) const { return Value < b; }
        bool operator==(double b) const { return Value == b; }
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

    struct Vec3d
    {
        double X = 0.0, Y = 0.0, Z = 0.0;

        Vec3d operator+(const Vec3d& v) const { return { X + v.X, Y + v.Y, Z + v.Z }; }
        Vec3d operator-(const Vec3d& v) const { return { X - v.X, Y - v.Y, Z - v.Z }; }
        Vec3d operator*(const Vec3d& v) const { return { X * v.X, Y * v.Y, Z * v.Z }; }
        Vec3d operator/(const Vec3d& v) const { return { X / v.X, Y / v.Y, Z / v.Z }; }
        Vec3d operator*(double v) const { return { X * v, Y * v, Z * v }; }
        Vec3d operator/(double v) const { return { X / v, Y / v, Z / v }; }
    };

    inline Vec3d operator*(double a, const Vec3d& b) { return { a*b.X, a*b.Y, a*b.Z }; }

    //////////////////////////////////////////////////////////////////////

    // compute new velocity:
    // v' = v + a*dt
    inline double integrate_velocity(double v0, Accel a, double dt)
    {
        return v0 + a.Value*dt;
    }

    /**
     * Compute a new position using "Velocity Verlet" integration
     * https://en.wikipedia.org/wiki/Verlet_integration#Velocity_Verlet
     * 1st form: x' = x + (2*v + a*dt)*dt*0.5  [new vel calculated before pos]
     * 2nd form: x' = x + v*dt + (a*dt^2)*0.5  [pos calculated before new vel]
     * @param x Previous position
     * @param v Previous velocity
     * @param a Current acceleration [assume const]
     * @param dt DeltaTime / time step [eg: 0.01]
     * @return New position x'
     */
    inline double integrate_pos(double x, double v, Accel a, double dt)
    {
        // 1st form using average velocity, assuming constant acc
        double vNew = v + a.Value*dt;
        double vAvg = (v + vNew)*0.5;
        return x + vAvg*dt;
    }

    struct Body
    {
        Vec3d pos { 0.0, 0.0, 0.0 };
        Vec3d vel { 2.0, 0.0, 0.0 }; // 2m/s along x-axis
        Vec3d acc { 0.0, 0.0, 0.0 }; // no acceleration at first
        double mass = 1.0; // 1kg
        double drag = 0.1; // rho*C*Area - simplified drag for this example

        /**
         * Update pos and vel using "Velocity Verlet" integration
         * @param dt DeltaTime / time step [eg: 0.01]
         */
        void update(double dt)
        {
            Vec3d new_pos = pos + vel*dt + acc*(dt*dt*0.5);
            Vec3d new_acc = apply_forces(); // only needed if acceleration is not constant
            Vec3d new_vel = vel + (acc+new_acc)*(dt*0.5);
            pos = new_pos;
            vel = new_vel;
            acc = new_acc;
        }

        Vec3d apply_forces() const
        {
            Vec3d grav_acc = Vec3d{0.0, 0.0, -9.81 }; // 9.81m/s^2 down in the Z-axis
            Vec3d drag_force = 0.5 * drag * (vel*vel); // D = 0.5 * (rho * C * Area * vel^2)
            Vec3d drag_acc = drag_force / mass; // a = F/m
            return grav_acc - drag_acc;
        }
    };

    /**
     * Compute new position using "Velocity Verlet" integration
     * 1st form: x' = x + (2*v + a*dt)*dt*0.5  [new vel calculated before pos]
     * 2nd form: x' = x + v*dt + (a*dt^2)*0.5  [pos calculated before new vel]
     * @param pos Previous position
     * @param vel Previous velocity
     * @param acc Current acceleration [assume const]
     * @param dt DeltaTime / time step [eg: 0.01]
     * @return New position x'
     */
    inline Vec3d integrate_pos(Vec3d pos, Vec3d vel, Vec3d acc, double dt)
    {
        // 1st form using average velocity
        Vec3d new_vel = vel + acc*dt;
        Vec3d avg_vel = (vel + new_vel)*0.5;
        return pos + avg_vel*dt;
    }

    // avg velocity = (x2 - x1) / (t2 - t1)
    inline double average_velocity(double x1, double x2, double dt)
    {
        return (x2 - x1) / dt;
    }

    //////////////////////////////////////////////////////////////////////
}
