#include "vehicle/car.h"

#include <algorithm>
#include <cmath>

namespace ncotb {

namespace {

bool isFront(int corner) { return corner < 2; }
/// -1 on the left (z < 0), +1 on the right.
Real side(int corner) { return corner % 2 == 0 ? -1 : 1; }

// Box sizes of the small parts (half extents).
const Vec3 kKnuckleHalf{0.01, 0.02, 0.008};
constexpr Real kUprightHalfZ = 0.006;

/// The points that define one corner, in the car's frame.
struct CornerGeometry {
    Vec3 lowerPivot, upperPivot; ///< arm hinges on the chassis
    Vec3 lowerOuter, upperOuter; ///< arm hinges on the upright
    Vec3 hub;                    ///< wheel centre
    Vec3 kingpin;                ///< steering hinge (front)
    Vec3 shockTop, shockArm;     ///< shock ends on the chassis and on the lower arm
};

CornerGeometry geometry(const CarSpec& s, int c) {
    const Real x = (isFront(c) ? 1 : -1) * s.wheelbase / 2, z = side(c);
    const Real yHub = -s.armDrop, yLow = yHub - s.lowerArm, yUp = yLow + s.armGap;
    return {{x, yLow, z * s.armPivot},        {x, yUp, z * s.armPivot},   {x, yLow, z * s.armOuter},
            {x, yUp, z * s.armOuter},         {x, yHub, z * s.track / 2}, {x, yHub, z * s.kingpin},
            {x, s.shockTop, z * s.shockTopZ}, {x, yLow, z * s.shockArm}};
}

/// Velocity of a point turning with unit rate about the x axis through pivot
/// (car frame): how far it moves per radian of arm rotation.
Vec3 swing(const Vec3& p, const Vec3& pivot) { return cross(Vec3{1, 0, 0}, p - pivot); }

} // namespace

Car::Car(World& world, const CarSpec& spec, const Vec3& position, Real heading, int collisionGroup)
    : world_(&world),
      spec_(spec) {
    origin_ = position + Vec3{0, spec.wheelRadius + spec.armDrop, 0};
    orientation_ = Quat::fromAxisAngle({0, 1, 0}, heading);
    computePoses();
    build(collisionGroup);
}

void Car::computePoses() {
    for (int c = 0; c < 4; ++c) {
        const CornerGeometry g = geometry(spec_, c);
        const Real z = side(c);
        armPose_[c].x = (g.lowerPivot + g.lowerOuter) / 2;
        uprightPose_[c].x = (g.lowerOuter + g.upperOuter) / 2 + Vec3{0, 0, z * kUprightHalfZ};
        wheelPose_[c].x = g.hub;
        if (isFront(c)) knucklePose_[c].x = g.kingpin + Vec3{0, 0, z * kKnuckleHalf.z};
    }
}

Vec3 Car::local(const Vec3& p) const { return origin_ + rotate(orientation_, p); }

void Car::build(int collisionGroup) {
    World& w = *world_;
    const CarSpec& s = spec_;
    auto add = [&](const Shape& shape, Real mass, const Vec3& at) {
        RigidBody& b = w.addBody(shape, mass, local(at), orientation_);
        b.collisionGroup = collisionGroup;
        return &b;
    };
    auto hinge = [&](RigidBody& a, RigidBody& b, const Vec3& at, const Vec3& axis) {
        HingeJoint* j = &w.addHinge(a, b, local(at), rotate(orientation_, axis));
        joints_.push_back(j);
        return j;
    };
    chassis_ = add(Shape::box(s.chassisHalfExtents), s.chassisMass, {});

    const Real halfLength = (s.armOuter - s.armPivot) / 2;
    for (int c = 0; c < 4; ++c) {
        const CornerGeometry g = geometry(s, c);
        arms_[c] = add(Shape::box({0.012, 0.008, halfLength}), s.armMass, armPose_[c].x);
        uprights_[c] =
            add(Shape::box({0.012, s.armGap / 2 + 0.008, kUprightHalfZ}), s.uprightMass, uprightPose_[c].x);

        armHinge_[c] = hinge(*chassis_, *arms_[c], g.lowerPivot, {1, 0, 0});
        armHinge_[c]->setLimits(-s.armTravel, s.armTravel);
        hinge(*arms_[c], *uprights_[c], g.lowerOuter, {1, 0, 0});
        joints_.push_back(
            &w.addDistanceJoint(*chassis_, *uprights_[c], local(g.upperPivot), local(g.upperOuter)));

        RigidBody* holder = uprights_[c];
        if (isFront(c)) {
            const int k = c; // FrontLeft, FrontRight are 0 and 1
            knuckles_[k] = add(Shape::box(kKnuckleHalf), s.knuckleMass, knucklePose_[k].x);
            steer_[k] = hinge(*uprights_[c], *knuckles_[k], g.kingpin, {0, 1, 0});
            steer_[k]->setLimits(-(s.maxSteer + 0.2), s.maxSteer + 0.2);
            holder = knuckles_[k];
        }
        wheels_[c] = add(Shape::sphere(s.wheelRadius), s.wheelMass, g.hub);
        wheels_[c]->friction = s.wheelFriction;
        wheels_[c]->restitution = 0.1;
        // The axle points to the car's left, so positive spin rolls forward.
        axleHinge_[c] = hinge(*holder, *wheels_[c], g.hub, {0, 0, -1});
    }
    for (int c = 0; c < 4; ++c) {
        Real k, damping, rest;
        shockTuning(c, k, damping, rest);
        const CornerGeometry g = geometry(s, c);
        w.addSpring(*chassis_, local(g.shockTop), *arms_[c], local(g.shockArm), k, damping, rest);
    }
}

void Car::shockTuning(int c, Real& stiffness, Real& damping, Real& restLength) const {
    // Worked out in the car's frame at the design pose (arms horizontal, car
    // level), by virtual work: per radian the lower arm turns, every force does
    // work f . (how far its point moves), and the shock must cancel the total.
    const CarSpec& s = spec_;
    // Tuned for Earth, like a real truck: on the Moon it rides high and bouncy.
    const Real g = kEarthGravity;
    const CornerGeometry geo = geometry(s, c);

    // Static wheel loads: the knuckles put a little more weight on the front.
    const Real corner = s.armMass + s.uprightMass + s.wheelMass;
    const Real total = s.chassisMass + 4 * corner + 2 * s.knuckleMass;
    const Real xCentre = 2 * s.knuckleMass * (s.wheelbase / 2) / total;
    const Real share = isFront(c) ? 0.5 + xCentre / s.wheelbase : 0.5 - xCentre / s.wheelbase;
    const Real load = total * g * share / 2;

    // The upright and everything on it move like the lower arm's outer hinge.
    const Vec3 upright = swing(geo.lowerOuter, geo.lowerPivot);
    Real work = dot(Vec3{0, load, 0}, upright); // the ground pushing up on the tyre
    const Real carried = s.uprightMass + s.wheelMass + (isFront(c) ? s.knuckleMass : 0);
    work += dot(Vec3{0, -carried * g, 0}, upright);
    work += dot(Vec3{0, -s.armMass * g, 0}, swing(armPose_[c].x, geo.lowerPivot));
    const Vec3 dir = normalize(geo.shockArm - geo.shockTop); // the shock pushes the arm this way
    const Real leverShock = dot(dir, swing(geo.shockArm, geo.lowerPivot));
    const Real force = -work / leverShock;

    // Wheel rate from the ride frequency of a quarter of the chassis, turned into
    // the spring rate through the lever ratio (the shock moves leverShock per
    // radian, the wheel leverWheel).
    const Real leverWheel = s.armOuter - s.armPivot;
    const Real quarter = s.chassisMass / 4;
    const Real omega = 2 * M_PI * s.suspensionFrequency;
    const Real wheelRate = quarter * omega * omega;
    const Real ratio2 = (leverWheel / leverShock) * (leverWheel / leverShock);
    stiffness = wheelRate * ratio2;
    damping = 2 * s.suspensionDamping * std::sqrt(wheelRate * quarter) * ratio2;
    restLength = length(geo.shockArm - geo.shockTop) + force / stiffness; // compressed by force / k
}

void Car::retune() {
    for (int c = 0; c < 4; ++c) {
        Spring* sp = world_->findSpring(*chassis_, *arms_[c]);
        if (!sp) continue;
        shockTuning(c, sp->stiffness, sp->damping, sp->restLength);
    }
}

void Car::place(const Vec3& position, Real heading) {
    origin_ = position + Vec3{0, spec_.wheelRadius + spec_.armDrop, 0};
    orientation_ = Quat::fromAxisAngle({0, 1, 0}, heading);
    auto put = [&](RigidBody& b, const Vec3& at) {
        b.x = local(at);
        b.q = orientation_;
        b.P = {};
        b.L = {};
        b.updateDerived();
        b.wake();
    };
    put(*chassis_, {});
    for (int c = 0; c < 4; ++c) {
        put(*arms_[c], armPose_[c].x);
        put(*uprights_[c], uprightPose_[c].x);
        put(*wheels_[c], wheelPose_[c].x);
    }
    for (int k = 0; k < 2; ++k)
        put(*knuckles_[k], knucklePose_[k].x);
}

void Car::flip() {
    Vec3 f = forward();
    Real heading = std::hypot(f.x, f.z) > 1e-6 ? std::atan2(-f.z, f.x) : 0;
    place(chassis_->x - Vec3{0, spec_.wheelRadius + spec_.armDrop, 0} + Vec3{0, 0.3, 0}, heading);
}

void Car::drive(Real throttle, Real steer, Real brake) {
    const CarSpec& s = spec_;
    throttle = std::clamp(throttle, Real(-1), Real(1));
    brake = std::clamp(brake, Real(0), Real(1));
    const Real maxSpin = s.topSpeed / s.wheelRadius;
    for (int c = 0; c < 4; ++c) {
        HingeJoint& axleJoint = *axleHinge_[c];
        if (brake > 0) {
            // Brake: hold the wheel's spin at zero relative to its holder, with
            // limited torque (an implicit constraint, so it can't overshoot).
            axleJoint.setMotor(0, brake * s.brakeTorque);
            continue;
        }
        axleJoint.disableMotor();
        // Motor: full torque at standstill, falling to zero at top speed. The
        // reaction goes to the chassis, where the motor is.
        const Real spin = wheelSpin(c);
        Real torque = 0;
        if (throttle != 0) {
            const Real sense = throttle > 0 ? 1 : -1;
            torque = s.motorTorque * throttle * std::clamp(1 - sense * spin / maxSpin, Real(0), Real(1));
        }
        torque -= s.coastDrag * spin;
        const Vec3 n = axleJoint.axis();
        wheels_[c]->torque += n * torque;
        chassis_->torque -= n * torque;
    }

    // Steering servos with Ackermann geometry: each front wheel points at the
    // same turning centre on the line of the rear axle, so the inner one turns more.
    const Real centre = std::clamp(steer, Real(-1), Real(1)) * s.maxSteer;
    for (int k = 0; k < 2; ++k) {
        const Real z = (k == 0 ? -1 : 1) * s.kingpin;
        Real target = 0;
        if (std::abs(centre) > 1e-6) target = std::atan(s.wheelbase / (s.wheelbase / std::tan(centre) + z));
        // Turning the wheel wakes a parked truck (the servo alone wouldn't).
        if (std::abs(target - steer_[k]->angle()) > 0.01) knuckles_[k]->wake();
        Real rate = std::clamp(30 * (target - steer_[k]->angle()), -s.steerRate, s.steerRate);
        steer_[k]->setMotor(rate, s.steerTorque);
    }
}

Vec3 Car::axle(int corner) const { return axleHinge_[corner]->axis(); }

Vec3 Car::forward() const { return chassis_->R().column(0); }

Real Car::speed() const { return dot(chassis_->v(), forward()); }

Real Car::heading() const {
    Vec3 f = forward();
    return std::atan2(-f.z, f.x);
}

Real Car::wheelSpin(int corner) const {
    const HingeJoint& j = *axleHinge_[corner];
    return dot(j.bodyB().omega() - j.bodyA().omega(), j.axis());
}

Real Car::suspensionTravel(int corner) const {
    const Real leverWheel = spec_.armOuter - spec_.armPivot;
    return -side(corner) * armHinge_[corner]->angle() * leverWheel;
}

bool Car::intact(const World& world) const {
    auto present = [&](const RigidBody* b) {
        return std::any_of(world.bodies().begin(), world.bodies().end(),
                           [&](const auto& p) { return p.get() == b; });
    };
    if (!present(chassis_)) return false;
    for (int c = 0; c < 4; ++c)
        if (!present(arms_[c]) || !present(uprights_[c]) || !present(wheels_[c])) return false;
    return present(knuckles_[0]) && present(knuckles_[1]);
}

Real Car::jointError() const {
    Real worst = 0;
    for (const Joint* j : joints_)
        worst = std::max(worst, j->error());
    return worst;
}

} // namespace ncotb
