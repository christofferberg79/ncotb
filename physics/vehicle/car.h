#pragma once

#include "world.h"

#include <array>
#include <vector>

namespace ncotb {

/// Dimensions, masses and tuning of a Car. The defaults are an RC monster truck
/// at about 1:10 scale. In the car's frame x points forward, y up and z right.
struct CarSpec {
    // Chassis (the battery and motor are in it).
    Vec3 chassisHalfExtents{0.21, 0.03, 0.07};
    Real chassisMass = 2.2;
    // Where the wheels are: axles at x = +-wheelbase / 2, wheel centres at
    // z = +-track / 2, armDrop below the chassis centre.
    Real wheelbase = 0.32;
    Real track = 0.34;
    Real armDrop = 0.02;
    // Wheels: spheres for collision (drawn as tyres wheelWidth wide).
    Real wheelRadius = 0.085;
    Real wheelWidth = 0.07;
    Real wheelMass = 0.15;
    Real wheelFriction = 1.0;
    // Double wishbones: per wheel a lower arm hinged to the chassis at
    // z = +-armPivot and to an upright at z = +-armOuter, and above it, armGap
    // higher, a camber link (a rod) between the same z positions. Equal and
    // parallel, so the upright moves up and down without tilting. The lower arm
    // is lowerArm below the wheel centre.
    Real armPivot = 0.05;
    Real armOuter = 0.125;
    Real lowerArm = 0.012;
    Real armGap = 0.045;
    Real armMass = 0.08;
    Real uprightMass = 0.06;
    Real armTravel = 0.45; ///< hinge limit, radians either way
    // Front steering knuckles turn on the upright about a vertical kingpin at z = +-kingpin.
    Real kingpin = 0.14;
    Real knuckleMass = 0.06;
    // Shocks: from a tower on the chassis (shockTop above its centre, z = +-shockTopZ)
    // to the arm at z = +-shockArm. Tuned by the ride's natural frequency and
    // damping ratio; the rest length is chosen so the car stands level with the
    // arms horizontal.
    Real shockTop = 0.07;
    Real shockTopZ = 0.04;
    Real shockArm = 0.11;
    Real suspensionFrequency = 2.5; ///< Hz, of the chassis bouncing on the springs
    Real suspensionDamping = 0.6;   ///< damping ratio
    // Drive: an electric motor on every wheel (4WD), torque falling linearly to
    // zero at top speed.
    Real motorTorque = 0.45; ///< N m per wheel at standstill
    Real topSpeed = 12;      ///< m/s
    Real coastDrag = 0.0005; ///< N m s: the motors' drag when coasting
    Real brakeTorque = 0.6;  ///< N m per wheel
    // Steering servo.
    Real maxSteer = 0.45; ///< radians, at the centre line (Ackermann sets each wheel)
    Real steerRate = 6;   ///< rad/s, servo speed
    Real steerTorque = 2; ///< N m
};

/// A four-wheel car built from rigid bodies, joints and springs, like an RC
/// monster truck: each wheel turns on an upright held by a lower arm and an
/// upper camber link (a double wishbone), and a spring and damper (a "shock")
/// from the chassis to the lower arm carries the load. The front wheels turn
/// on steering knuckles on their uprights.
///
///   chassis --hinge (x)--> lower arm --hinge (x)--> upright <--rod-- chassis
///   upright --hinge (y, steering)--> knuckle --hinge (axle)--> wheel     (front)
///   upright --hinge (axle)--> wheel                                       (rear)
///   chassis --spring + damper--> lower arm
///
/// Arm and link form a parallelogram, so a sideways force on the tyre pulls
/// along them and doesn't act on the spring (single swing arms "jack": the
/// tyre's grip then carries part of the weight, and the solver may pick any
/// split). A rod (one constraint) rather than a hinged upper arm leaves each
/// corner with exactly one way to move and no redundant constraints, which the
/// iterative solver needs to settle (with four hinges it took ~10x the iterations).
///
/// All parts share a collision group, so they don't collide with each other.
class Car {
public:
    enum Corner { FrontLeft, FrontRight, RearLeft, RearRight };

    Car(World& world, const CarSpec& spec, const Vec3& position, Real heading, int collisionGroup = 77);

    /// Driver input for the next step: throttle in [-1, 1] (negative: reverse),
    /// steer in [-1, 1] (positive: left), brake in [0, 1]. Call before each step.
    void drive(Real throttle, Real steer, Real brake);

    /// Puts the car back on its wheels, standing at position (chassis centre
    /// height wheelRadius + armDrop above it), facing heading, at rest.
    void place(const Vec3& position, Real heading);
    /// Lifts the car 0.3 m and sets it upright, facing the way it was going.
    void flip();

    /// Applies changed suspension settings (frequency, damping) to the springs.
    void retune();
    CarSpec& spec() { return spec_; }
    const CarSpec& spec() const { return spec_; }

    RigidBody& chassis() const { return *chassis_; }
    RigidBody& wheel(int corner) const { return *wheels_[corner]; }
    const std::array<RigidBody*, 4>& wheels() const { return wheels_; }
    const std::array<RigidBody*, 4>& arms() const { return arms_; } ///< the lower arms
    const std::array<RigidBody*, 4>& uprights() const { return uprights_; }
    const std::array<RigidBody*, 2>& knuckles() const { return knuckles_; }
    HingeJoint& steering(int side) const { return *steer_[side]; }
    /// The axle direction of a wheel (world space, pointing to the car's left).
    Vec3 axle(int corner) const;

    Vec3 forward() const; ///< the chassis' forward axis
    Real speed() const;   ///< chassis velocity along forward()
    Real heading() const; ///< yaw angle of forward() (0 along +x, positive to the left)
    /// Wheel spin relative to its holder (rad/s, positive rolling forward).
    Real wheelSpin(int corner) const;
    /// How far a wheel moved up from its resting position (m, along the arm's arc).
    Real suspensionTravel(int corner) const;
    /// True if every part is still in the world (the user may have deleted one).
    bool intact(const World& world) const;
    /// Largest distance between a joint's two anchors (m): how far the joints drift apart.
    Real jointError() const;

private:
    struct Pose {
        Vec3 x; ///< in the car's frame
    };
    void build(int collisionGroup);
    /// The spring rate, damping and rest length of a corner's shock.
    void shockTuning(int corner, Real& stiffness, Real& damping, Real& restLength) const;
    Vec3 local(const Vec3& p) const; ///< car frame -> world, at the placed pose

    World* world_;
    CarSpec spec_;
    Vec3 origin_;
    Quat orientation_;
    RigidBody* chassis_ = nullptr;
    std::array<RigidBody*, 4> arms_{}, uprights_{}, wheels_{};
    std::array<RigidBody*, 2> knuckles_{};
    std::array<HingeJoint*, 4> armHinge_{}, axleHinge_{};
    std::vector<Joint*> joints_; ///< all of the car's joints
    std::array<HingeJoint*, 2> steer_{};
    std::array<Pose, 4> armPose_{}, uprightPose_{}, wheelPose_{};
    std::array<Pose, 2> knucklePose_{};
};

} // namespace ncotb
