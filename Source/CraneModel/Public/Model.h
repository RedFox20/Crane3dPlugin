// Copyright (c) 2019 - Jorma Rebane 3DCrane UE4
#pragma once
#include <string> // std::string

namespace crane3d
{
    /**
     * Allows switching between different crane model dynamics
     */
    enum class ModelType
    {
        // The most basic and foolproof crane model
        Linear,

        // Non-linear model with constant pendulum length with 2 control forces.
        // LiftLine (Fwind) is ignored
        NonLinearConstLine,


        // Non-linear fully dynamic model with all 3 forces
        NonLinearComplete,

        // Original non-linear fully dynamic model with all 3 forces and refined friction formulae
        NonLinearOriginal,
    };

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

    struct Component
    {
        Mass Mass = 1_kg;
        double LimitMin = 0.0;
        double LimitMax = 0.0;
        double Pos = 0.0;
        double Vel = 0.0;
        Accel Acc = 0_ms2; // actual acceleration

        Force Applied; // applied force
        Force Friction; // friction
        Force Fnet; // net force
        Accel NetAcc; // net driving acceleration

        // friction coefficient for Steel-Steel (depends highly on type of steel)
        // https://hypertextbook.com/facts/2005/steel.shtml
        double CoeffStatic = 0.8; // static coeff, dry surface
        double CoeffKinetic = 0.7; // kinetic coeff, dry surface

        Component() = default;
        Component(double limitMin, double limitMax) : LimitMin{limitMin}, LimitMax{limitMax}
        {
        }

        // Update pos and vel using "Velocity Verlet" integration
        void Update(Accel new_acc, double dt);
        
        // Apply driving forces
        void ApplyForce(Force applied, Accel g);

        // Prevent applying force when against frame
        Force ClampForceByPosLimits(Force force) const;
    };

    /**
     * Output state of the model
     */
    struct ModelState
    {
        double Alfa = 0.0; // α pendulum measured alfa angle
        double Beta = 0.0; // β pendulum measured beta angle

        double RailOffset = 0.0; // Xw distance of the rail with the cart from the center of the construction frame
        double CartOffset = 0.0; // Yw distance of the cart from the center of the rail
        double LiftLine   = 0.0; // R lift-line length

        // Payload 3D coordinates
        double PayloadX = 0.0;
        double PayloadY = 0.0;
        double PayloadZ = 0.0;

        void Print() const;
    };

    // Coordinate system of the Crane model
    // X: outermost movement of the rail, considered as forward
    // Y: left-right movement of the cart
    // Z: up-down movement of the payload
    class Model
    {
    public:
        /**
         * NOTE: These are the customization parameters of the model
         */
        // Which model to use? Linear is simple and foolproof		
        ModelType Type = ModelType::Linear;
        Mass Mpayload = 1.000_kg; // Mc mass of the payload
        Mass Mcart    = 1.155_kg; // Mw mass of the cart
        Mass Mrail    = 2.200_kg; // Ms mass of the moving rail
        double G { 9.81 };  // gravity constant, 9.81m/s^2
        Accel g = 9.81_ms2; // gravity constant, 9.81m/s^2

        Component Rail { -0.30, +0.30 }; // rail component
        Component Cart { -0.35, +0.35 }; // cart component
        Component Line { +0.05, +0.90 }; // lift-line winding component
        Component CAlfa { -0.05, +0.05 }; // Alfa component
        Component CBeta { -0.05, +0.05 }; // Beta component

        double RailFriction = 100.0; // Tx rail friction
        double CartFriction = 82.0;  // Ty cart friction
        double WindingFriction = 75.0;  // Tr liftline winding friction 

    private:

        double X = 0.0; // distance of the rail with the cart from the center of the construction frame
        double Y = 0.0; // distance of the cart from the center of the rail;
        double R = 0.5; // length of the lift-line
        double Alfa = 0.0; // α angle between y axis (cart moving left-right) and the lift-line
        double Beta = 0.0; // β angle between negative direction on the z axis and the projection
                           // of the lift-line onto the xz plane
        
        // only used for basic linear model
        double Δα = 0.0, Δα_vel = 0.0;
        double Δβ = 0.0, Δβ_vel = 0.0;

        // velocity time derivatives
        double X_vel = 0.0; // rail X velocity
        double Y_vel = 0.0; // cart Y velocity
        double R_vel = 0.0; // payload Z velocity
        double Alfa_vel = 0.0;
        double Beta_vel = 0.0;

        // x1..x10 as per 3DCrane mathematical model description
        double u1, u2, u3; // driving acceleration of cart, rail, wind
        double T1, T2, T3; // friction accel of cart, rail, wind
        double N1, N2, N3; // net acceleration of cart, rail, wind

        double ADrcart, ADrrail, ADrwind; // driving accel of cart, rail, wind
        double AFrcart, AFrrail, AFrwind; // friction accel of cart, rail, wind
        double μ1, μ2; // coefficient of friction: payload/cart ratio;  payload/railcart ratio

        // simulation time sink for running correct number of iterations every update
        double SimulationTimeSink = 0.0;
        int64_t DiscreteStepCounter = 0;

        // for debugging:
        double DbgFixedTimeStep = 0.0;
        double DbgAvgIterations = 1.0; // for debugging

    public:

        Model();

        /**
         * Updates the model using a fixed time step
         * @param fixedTime Size of the fixed time step. For example 0.01
         * @param deltaTime Time since last update
         * @param Frail force driving the rail with cart (Fx)
         * @param Fcart force driving the cart along the rail (Fy)
         * @param Fwind force winding the lift-line (Fr)
         * @return New state of the crane model
         */
        ModelState UpdateFixed(double fixedTime, double deltaTime, Force Frail, Force Fcart, Force Fwind);

        /**
         * Updates the model using deltaTime as the time step. This can be unstable if deltaTime varies.
         * @param deltaTime Time since last update
         * @param Frail force driving the rail with cart (Fx)
         * @param Fcart force driving the cart along the rail (Fy)
         * @param Fwind force winding the lift-line (Fr)
         * @return New state of the crane model
         */
        ModelState Update(double deltaTime, Force Frail, Force Fcart, Force Fwind);

        /**
         * @return Current state of the crane:
         *  distance of the rail, cart, length of lift-line and swing angles of the payload
         */
        ModelState GetState() const;

        std::wstring GetStateDebugText() const;

    private:


        void PrepareBasicRelations(Force Frail, Force Fcart, Force Fwind);
        
        // ------------------
        
        void BasicLinearModel(double dt, Force Frail, Force Fcart, Force Fwind);

        // ------------------

        void NonLinearConstLine(double dt, Force Frail, Force Fcart, Force Fwind);
        void NonLinearCompleteModel(double dt, Force Frail, Force Fcart, Force Fwind);
        void NonLinearOriginalModel(double dt, Force Frail, Force Fcart, Force Fwind);

        // ------------------
    };

}
