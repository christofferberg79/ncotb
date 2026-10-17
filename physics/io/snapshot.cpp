#include "io/snapshot.h"

#include "vehicle/car.h"
#include "world.h"

#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <utility>

namespace ncotb {

// ------------------------------------------------------------------ writer and reader

SnapshotWriter::SnapshotWriter(std::ostream& out) : out_(out) {}

SnapshotWriter& SnapshotWriter::key(const char* keyword) {
    end();
    out_ << keyword;
    open_ = true;
    return *this;
}

SnapshotWriter& SnapshotWriter::operator<<(Real v) {
    char buf[40];
    std::snprintf(buf, sizeof buf, " %.17g", v); // enough digits to read back the same double
    out_ << buf;
    return *this;
}

SnapshotWriter& SnapshotWriter::operator<<(int v) {
    out_ << ' ' << v;
    return *this;
}

SnapshotWriter& SnapshotWriter::operator<<(const Vec3& v) { return *this << v.x << v.y << v.z; }
SnapshotWriter& SnapshotWriter::operator<<(const Quat& q) { return *this << q.s << q.v; }

SnapshotWriter& SnapshotWriter::word(const std::string& w) {
    out_ << ' ' << w;
    return *this;
}

void SnapshotWriter::comment(const std::string& text) {
    end();
    out_ << "# " << text << '\n';
}

void SnapshotWriter::end() {
    if (open_) out_ << '\n';
    open_ = false;
}

SnapshotReader::SnapshotReader(std::istream& in) : in_(in) {}

void SnapshotReader::fail(const std::string& message) {
    if (error_.empty()) error_ = "line " + std::to_string(line_) + ": " + message;
}

namespace {
/// The tokens of one line, without its comment.
std::vector<std::string> tokenize(const std::string& line) {
    std::istringstream ss(line.substr(0, line.find('#')));
    std::vector<std::string> tokens;
    for (std::string t; ss >> t;)
        tokens.push_back(t);
    return tokens;
}
} // namespace

bool SnapshotReader::nextToken(std::string& token) {
    if (failed()) return false;
    if (pos_ >= tokens_.size()) {
        fail("missing value after '" + (tokens_.empty() ? std::string() : tokens_[0]) + "'");
        return false;
    }
    token = tokens_[pos_++];
    return true;
}

std::string SnapshotReader::peek() {
    if (failed()) return "";
    // Look ahead without consuming: remember the stream position.
    auto at = in_.tellg();
    int line = line_;
    std::string text, keyword;
    while (std::getline(in_, text)) {
        auto t = tokenize(text);
        if (!t.empty()) {
            keyword = t[0];
            break;
        }
    }
    in_.clear();
    in_.seekg(at);
    line_ = line;
    return keyword;
}

bool SnapshotReader::expect(const char* keyword) {
    if (failed()) return false;
    if (pos_ < tokens_.size()) {
        fail("unexpected '" + tokens_[pos_] + "' after '" + tokens_[0] + "'");
        return false;
    }
    std::string text;
    tokens_.clear();
    while (tokens_.empty()) {
        if (!std::getline(in_, text)) {
            fail(std::string("unexpected end, expected '") + keyword + "'");
            return false;
        }
        ++line_;
        tokens_ = tokenize(text);
    }
    pos_ = 1;
    if (tokens_[0] != keyword) {
        fail("expected '" + std::string(keyword) + "', found '" + tokens_[0] + "'");
        return false;
    }
    return true;
}

Real SnapshotReader::real() {
    std::string t;
    if (!nextToken(t)) return 0;
    char* end = nullptr;
    Real v = std::strtod(t.c_str(), &end);
    if (end == t.c_str() || *end != '\0') fail("not a number: '" + t + "'");
    return v;
}

int SnapshotReader::integer() {
    std::string t;
    if (!nextToken(t)) return 0;
    char* end = nullptr;
    long v = std::strtol(t.c_str(), &end, 10);
    if (end == t.c_str() || *end != '\0') fail("not an integer: '" + t + "'");
    return int(v);
}

Vec3 SnapshotReader::vec3() {
    Real x = real(), y = real(), z = real();
    return {x, y, z};
}

Quat SnapshotReader::quat() {
    Real s = real();
    return {s, vec3()};
}

std::string SnapshotReader::word() {
    std::string t;
    nextToken(t);
    return t;
}

// ------------------------------------------------------------------ the world

int bodyIndex(const World& world, const RigidBody* b) {
    const auto& bodies = world.bodies();
    for (size_t i = 0; i < bodies.size(); ++i)
        if (bodies[i].get() == b) return int(i);
    return -1; // the ground anchor
}

/// Friend of World, the joints and Car: reads and writes their private state.
struct SnapshotAccess {
    static RigidBody* body(World& w, SnapshotReader& in) {
        int i = in.integer();
        if (i == -1) return &w.ground();
        if (i < 0 || i >= int(w.bodies().size())) {
            in.fail("no body " + std::to_string(i));
            return &w.ground();
        }
        return w.bodies()[size_t(i)].get();
    }
    /// Joints that are saved (all but mouse joints), in the world's order.
    static std::vector<const Joint*> savedJoints(const World& w) {
        std::vector<const Joint*> list;
        for (const auto& j : w.joints())
            if (!dynamic_cast<const MouseJoint*>(j.get())) list.push_back(j.get());
        return list;
    }
    static int jointIndex(const World& w, const Joint* j) {
        auto list = savedJoints(w);
        for (size_t i = 0; i < list.size(); ++i)
            if (list[i] == j) return int(i);
        return -1;
    }

    static void saveJoint(const World& w, const Joint& j, SnapshotWriter& out) {
        const int a = bodyIndex(w, &j.bodyA()), b = bodyIndex(w, &j.bodyB());
        if (auto* d = dynamic_cast<const DistanceJoint*>(&j)) {
            out.key("joint").word("distance") << a << b;
            out.key("anchors") << d->localA_ << d->localB_;
            out.key("length") << d->length_;
            out.key("impulse") << d->impulse_;
        } else if (auto* s = dynamic_cast<const SliderJoint*>(&j)) {
            out.key("joint").word("slider") << a << b;
            out.key("anchors") << s->localA_ << s->localB_;
            out.key("axis") << s->localAxis_;
            out.key("relative") << s->relativeQ0_;
            out.key("limits") << int(s->limits_) << s->lower_ << s->upper_;
            out.key("motor") << int(s->motor_) << s->motorSpeed_ << s->maxMotorForce_;
            out.key("impulses") << s->perp_[0].impulse << s->perp_[1].impulse << s->axial_.impulse
                                << s->angularImpulse_ << s->lowerImpulse_ << s->upperImpulse_
                                << s->motorImpulse_;
        } else if (auto* h = dynamic_cast<const HingeJoint*>(&j)) {
            out.key("joint").word("hinge") << a << b;
            out.key("anchors") << h->localA_ << h->localB_;
            out.key("axes") << h->localAxisA_ << h->localAxisB_;
            out.key("references") << h->localRefA_ << h->localRefB_;
            out.key("limits") << int(h->limits_) << h->lower_ << h->upper_;
            out.key("motor") << int(h->motor_) << h->motorSpeed_ << h->maxMotorTorque_;
            out.key("impulses") << h->impulse_ << h->alignImpulse_[0] << h->alignImpulse_[1]
                                << h->lowerImpulse_ << h->upperImpulse_ << h->motorImpulse_;
        } else if (auto* ball = dynamic_cast<const BallJoint*>(&j)) {
            out.key("joint").word("ball") << a << b;
            out.key("anchors") << ball->localA_ << ball->localB_;
            out.key("impulse") << ball->impulse_;
        }
    }

    static bool loadJoint(World& w, SnapshotReader& in) {
        if (!in.expect("joint")) return false;
        const std::string type = in.word();
        RigidBody* a = body(w, in);
        RigidBody* b = body(w, in);
        if (in.failed()) return false;
        // Create it at the bodies' poses, then put back the exact saved frames.
        in.expect("anchors");
        const Vec3 la = in.vec3(), lb = in.vec3();
        const Vec3 at = a->x + a->R() * la;
        if (type == "distance") {
            DistanceJoint& d = w.addDistanceJoint(*a, *b, at, b->x + b->R() * lb);
            d.localA_ = la;
            d.localB_ = lb;
            in.expect("length");
            d.length_ = in.real();
            in.expect("impulse");
            d.impulse_ = in.real();
        } else if (type == "slider") {
            in.expect("axis");
            const Vec3 axis = in.vec3();
            SliderJoint& s = w.addSlider(*a, *b, at, a->R() * axis);
            s.localA_ = la;
            s.localB_ = lb;
            s.localAxis_ = axis;
            in.expect("relative");
            s.relativeQ0_ = in.quat();
            in.expect("limits");
            s.limits_ = in.integer() != 0;
            s.lower_ = in.real();
            s.upper_ = in.real();
            in.expect("motor");
            s.motor_ = in.integer() != 0;
            s.motorSpeed_ = in.real();
            s.maxMotorForce_ = in.real();
            in.expect("impulses");
            s.perp_[0].impulse = in.real();
            s.perp_[1].impulse = in.real();
            s.axial_.impulse = in.real();
            s.angularImpulse_ = in.vec3();
            s.lowerImpulse_ = in.real();
            s.upperImpulse_ = in.real();
            s.motorImpulse_ = in.real();
        } else if (type == "hinge") {
            in.expect("axes");
            const Vec3 axisA = in.vec3(), axisB = in.vec3();
            HingeJoint& h = w.addHinge(*a, *b, at, a->R() * axisA);
            h.localA_ = la;
            h.localB_ = lb;
            h.localAxisA_ = axisA;
            h.localAxisB_ = axisB;
            in.expect("references");
            h.localRefA_ = in.vec3();
            h.localRefB_ = in.vec3();
            in.expect("limits");
            h.limits_ = in.integer() != 0;
            h.lower_ = in.real();
            h.upper_ = in.real();
            in.expect("motor");
            h.motor_ = in.integer() != 0;
            h.motorSpeed_ = in.real();
            h.maxMotorTorque_ = in.real();
            in.expect("impulses");
            h.impulse_ = in.vec3();
            h.alignImpulse_[0] = in.real();
            h.alignImpulse_[1] = in.real();
            h.lowerImpulse_ = in.real();
            h.upperImpulse_ = in.real();
            h.motorImpulse_ = in.real();
        } else if (type == "ball") {
            BallJoint& ball = w.addBallJoint(*a, *b, at);
            ball.localA_ = la;
            ball.localB_ = lb;
            in.expect("impulse");
            ball.impulse_ = in.vec3();
        } else {
            in.fail("unknown joint type '" + type + "'");
        }
        return !in.failed();
    }

    static void saveWorld(const World& w, SnapshotWriter& out) {
        out.key("world");
        out.key("time") << w.time_;
        out.key("gravity") << w.gravity;
        out.key("integrator") << int(w.integrator);
        out.key("substeps") << w.substeps;
        out.key("resolveContacts") << int(w.resolveContacts);
        out.key("contactMargin") << w.contactMargin;
        const SolverSettings& s = w.solver.settings;
        out.key("solver") << int(s.type) << s.iterations << int(s.warmStart) << s.baumgarte << s.slop
                          << s.maxPushVelocity << int(s.relax) << s.restitutionThreshold << int(s.friction)
                          << s.staggers << s.lcpMaxRows << s.jointMaxCorrection;
        out.comment("solver: type iterations warmStart baumgarte slop maxPush relax restitutionThreshold "
                    "friction staggers lcpMaxRows jointMaxCorrection");
        out.key("sleeping") << int(w.sleeping) << w.sleepSpeed << w.sleepSpin << w.sleepAfter;
        out.key("projection") << int(s.jointProjection) << s.positionIterations << s.projection;

        out.key("bodies") << int(w.bodies_.size());
        for (const auto& bp : w.bodies_) {
            const RigidBody& b = *bp;
            const Shape& sh = b.shape();
            switch (sh.type) {
            case Shape::Type::Sphere: out.key("body").word("sphere") << sh.radius; break;
            case Shape::Type::Box: out.key("body").word("box") << sh.halfExtents; break;
            case Shape::Type::Plane: out.key("body").word("plane"); break;
            case Shape::Type::Heightfield: {
                const Heightfield& hf = *sh.terrain;
                out.key("body").word("heightfield") << hf.nx() << hf.nz() << hf.cellSize();
                for (int k = 0; k < hf.nz(); ++k) {
                    out.key("row");
                    for (int i = 0; i < hf.nx(); ++i)
                        out << hf.heightAt(i, k);
                }
                break;
            }
            }
            out.key("mass") << b.mass();
            out.key("pose") << b.x << b.q;
            out.key("momentum") << b.P << b.L;
            out.key("material") << b.restitution << b.friction << b.collisionGroup;
            out.key("userData").word(std::to_string(b.userData));
            out.key("sleep") << int(b.asleep) << b.sleepTime;
        }

        auto joints = savedJoints(w);
        out.key("joints") << int(joints.size());
        for (const Joint* j : joints)
            saveJoint(w, *j, out);

        out.key("springs") << int(w.springs_.size());
        for (const Spring& sp : w.springs_) {
            out.key("spring") << bodyIndex(w, sp.a) << bodyIndex(w, sp.b) << sp.localA << sp.localB
                              << sp.restLength << sp.stiffness << sp.damping;
        }

        out.key("cache") << int(w.cache_.size());
        for (const World::CachedPoint& c : w.cache_)
            out.key("point") << bodyIndex(w, c.a) << bodyIndex(w, c.b) << c.localA << c.impulse
                             << c.tangentImpulse;

        // The frozen contacts of sleeping bodies (the step doesn't find them again).
        std::vector<const ContactManifold*> frozen;
        for (const ContactManifold& m : w.contacts_)
            if (!m.a->isAwake() && !m.b->isAwake()) frozen.push_back(&m);
        out.key("frozen") << int(frozen.size());
        for (const ContactManifold* m : frozen) {
            out.key("manifold") << bodyIndex(w, m->a) << bodyIndex(w, m->b) << m->normal << m->count;
            for (int i = 0; i < m->count; ++i) {
                const ContactPoint& p = m->points[i];
                out.key("contact") << p.position << p.depth << p.normalImpulse << p.tangentImpulse;
            }
        }
        out.key("end");
        out.end();
    }

    static bool loadWorld(World& w, SnapshotReader& in) {
        w.clear();
        in.expect("world");
        in.expect("time");
        const Real time = in.real();
        in.expect("gravity");
        w.gravity = in.vec3();
        in.expect("integrator");
        w.integrator = IntegratorType(in.integer());
        in.expect("substeps");
        w.substeps = in.integer();
        in.expect("resolveContacts");
        w.resolveContacts = in.integer() != 0;
        in.expect("contactMargin");
        w.contactMargin = in.real();
        SolverSettings& s = w.solver.settings;
        in.expect("solver");
        s.type = SolverType(in.integer());
        s.iterations = in.integer();
        s.warmStart = in.integer() != 0;
        s.baumgarte = in.real();
        s.slop = in.real();
        s.maxPushVelocity = in.real();
        s.relax = in.integer() != 0;
        s.restitutionThreshold = in.real();
        s.friction = in.integer() != 0;
        s.staggers = in.integer();
        s.lcpMaxRows = in.integer();
        s.jointMaxCorrection = in.real();
        if (in.peek() == "sleeping") {
            in.expect("sleeping");
            w.sleeping = in.integer() != 0;
            w.sleepSpeed = in.real();
            w.sleepSpin = in.real();
            w.sleepAfter = in.real();
        }
        if (in.peek() == "projection") { // older snapshots: Baumgarte for joints
            in.expect("projection");
            s.jointProjection = in.integer() != 0;
            s.positionIterations = in.integer();
            s.projection = in.real();
        } else {
            s.jointProjection = false;
        }

        in.expect("bodies");
        const int n = in.integer();
        for (int i = 0; i < n && !in.failed(); ++i) {
            in.expect("body");
            const std::string type = in.word();
            Shape shape;
            if (type == "sphere") {
                shape = Shape::sphere(in.real());
            } else if (type == "box") {
                shape = Shape::box(in.vec3());
            } else if (type == "plane") {
                shape = Shape::plane();
            } else if (type == "heightfield") {
                const int nx = in.integer(), nz = in.integer();
                const Real cell = in.real();
                if (nx < 2 || nz < 2 || nx > 100000 || nz > 100000) {
                    in.fail("bad heightfield size");
                    break;
                }
                std::vector<Real> heights(size_t(nx) * size_t(nz));
                for (int k = 0; k < nz && !in.failed(); ++k) {
                    in.expect("row");
                    for (int c = 0; c < nx; ++c)
                        heights[size_t(k) * size_t(nx) + size_t(c)] = in.real();
                }
                if (in.failed()) break;
                shape =
                    Shape::heightfield(std::make_shared<const Heightfield>(nx, nz, cell, std::move(heights)));
            } else {
                in.fail("unknown shape '" + type + "'");
                break;
            }
            in.expect("mass");
            RigidBody& b = w.addBody(shape, in.real());
            in.expect("pose");
            b.x = in.vec3();
            b.q = in.quat();
            in.expect("momentum");
            b.P = in.vec3();
            b.L = in.vec3();
            in.expect("material");
            b.restitution = in.real();
            b.friction = in.real();
            b.collisionGroup = in.integer();
            in.expect("userData");
            b.userData = std::uintptr_t(std::strtoull(in.word().c_str(), nullptr, 10));
            if (in.peek() == "sleep") { // not in snapshots from before sleeping
                in.expect("sleep");
                b.asleep = in.integer() != 0;
                b.sleepTime = in.real();
            }
            b.updateDerived();
        }

        in.expect("joints");
        const int nj = in.integer();
        for (int i = 0; i < nj && !in.failed(); ++i)
            loadJoint(w, in);

        in.expect("springs");
        const int ns = in.integer();
        for (int i = 0; i < ns && !in.failed(); ++i) {
            in.expect("spring");
            RigidBody* a = body(w, in);
            RigidBody* b = body(w, in);
            Spring sp{a, b, in.vec3(), in.vec3(), 0, 0, 0};
            sp.restLength = in.real();
            sp.stiffness = in.real();
            sp.damping = in.real();
            w.springs_.push_back(sp);
        }

        in.expect("cache");
        const int nc = in.integer();
        for (int i = 0; i < nc && !in.failed(); ++i) {
            in.expect("point");
            World::CachedPoint c{};
            c.a = body(w, in);
            c.b = body(w, in);
            c.localA = in.vec3();
            c.impulse = in.real();
            c.tangentImpulse = in.vec3();
            w.cache_.push_back(c);
        }
        if (in.peek() == "frozen") { // not in snapshots from before sleeping
            in.expect("frozen");
            const int nf = in.integer();
            for (int i = 0; i < nf && !in.failed(); ++i) {
                in.expect("manifold");
                ContactManifold m;
                m.a = body(w, in);
                m.b = body(w, in);
                m.normal = in.vec3();
                m.count = in.integer();
                if (m.count < 0 || m.count > ContactManifold::kMaxPoints) {
                    in.fail("bad contact count");
                    break;
                }
                for (int k = 0; k < m.count; ++k) {
                    in.expect("contact");
                    ContactPoint& p = m.points[k];
                    p.position = in.vec3();
                    p.depth = in.real();
                    p.normalImpulse = in.real();
                    p.tangentImpulse = in.vec3();
                }
                w.contacts_.push_back(m);
            }
            w.solvedContacts_ = 0;
        }
        in.expect("end");
        w.time_ = time;
        return !in.failed();
    }

    // -------------------------------------------------------------- cars

    /// The car's numeric settings, by name.
    static std::vector<std::pair<const char*, Real CarSpec::*>> specFields() {
        return {{"chassisMass", &CarSpec::chassisMass},
                {"wheelbase", &CarSpec::wheelbase},
                {"track", &CarSpec::track},
                {"armDrop", &CarSpec::armDrop},
                {"wheelRadius", &CarSpec::wheelRadius},
                {"wheelWidth", &CarSpec::wheelWidth},
                {"wheelMass", &CarSpec::wheelMass},
                {"wheelFriction", &CarSpec::wheelFriction},
                {"armPivot", &CarSpec::armPivot},
                {"armOuter", &CarSpec::armOuter},
                {"lowerArm", &CarSpec::lowerArm},
                {"armGap", &CarSpec::armGap},
                {"armMass", &CarSpec::armMass},
                {"uprightMass", &CarSpec::uprightMass},
                {"armTravel", &CarSpec::armTravel},
                {"kingpin", &CarSpec::kingpin},
                {"knuckleMass", &CarSpec::knuckleMass},
                {"shockTop", &CarSpec::shockTop},
                {"shockTopZ", &CarSpec::shockTopZ},
                {"shockArm", &CarSpec::shockArm},
                {"suspensionFrequency", &CarSpec::suspensionFrequency},
                {"suspensionDamping", &CarSpec::suspensionDamping},
                {"motorTorque", &CarSpec::motorTorque},
                {"topSpeed", &CarSpec::topSpeed},
                {"coastDrag", &CarSpec::coastDrag},
                {"brakeTorque", &CarSpec::brakeTorque},
                {"dragBrake", &CarSpec::dragBrake},
                {"maxSteer", &CarSpec::maxSteer},
                {"steerRate", &CarSpec::steerRate},
                {"steerTorque", &CarSpec::steerTorque}};
    }

    static void saveCar(const Car& car, const World& w, SnapshotWriter& out) {
        out.key("car");
        out.key("chassisHalfExtents") << car.spec_.chassisHalfExtents;
        for (auto [name, field] : specFields())
            out.key(name) << car.spec_.*field;
        out.key("frame") << car.origin_ << car.orientation_;
        auto bodies = [&](const char* key, const auto& list) {
            out.key(key);
            for (const RigidBody* b : list)
                out << bodyIndex(w, b);
        };
        auto joints = [&](const char* key, const auto& list) {
            out.key(key);
            for (const Joint* j : list)
                out << jointIndex(w, j);
        };
        out.key("chassis") << bodyIndex(w, car.chassis_);
        bodies("arms", car.arms_);
        bodies("uprights", car.uprights_);
        bodies("knuckles", car.knuckles_);
        bodies("wheels", car.wheels_);
        joints("armHinges", car.armHinge_);
        joints("axleHinges", car.axleHinge_);
        joints("steering", car.steer_);
        out.key("carJoints") << int(car.joints_.size());
        for (const Joint* j : car.joints_)
            out << jointIndex(w, j);
        out.key("endCar");
        out.end();
    }

    static std::unique_ptr<Car> loadCar(World& w, SnapshotReader& in) {
        in.expect("car");
        CarSpec spec;
        in.expect("chassisHalfExtents");
        spec.chassisHalfExtents = in.vec3();
        for (auto [name, field] : specFields()) {
            if (in.peek() != name) continue; // added since the snapshot was saved: keep the default
            in.expect(name);
            spec.*field = in.real();
        }
        auto car = std::unique_ptr<Car>(new Car(w, spec));
        in.expect("frame");
        car->origin_ = in.vec3();
        car->orientation_ = in.quat();
        auto savedBody = [&]() -> RigidBody* {
            int i = in.integer();
            if (i < 0 || i >= int(w.bodies().size())) {
                in.fail("no body " + std::to_string(i));
                return nullptr;
            }
            return w.bodies()[size_t(i)].get();
        };
        auto savedJoint = [&]() -> Joint* {
            auto list = savedJoints(w);
            int i = in.integer();
            if (i < 0 || i >= int(list.size())) {
                in.fail("no joint " + std::to_string(i));
                return nullptr;
            }
            return const_cast<Joint*>(list[size_t(i)]);
        };
        auto hinge = [&]() {
            auto* h = dynamic_cast<HingeJoint*>(savedJoint());
            if (!h && !in.failed()) in.fail("not a hinge");
            return h;
        };
        in.expect("chassis");
        car->chassis_ = savedBody();
        in.expect("arms");
        for (auto& b : car->arms_)
            b = savedBody();
        in.expect("uprights");
        for (auto& b : car->uprights_)
            b = savedBody();
        in.expect("knuckles");
        for (auto& b : car->knuckles_)
            b = savedBody();
        in.expect("wheels");
        for (auto& b : car->wheels_)
            b = savedBody();
        in.expect("armHinges");
        for (auto& j : car->armHinge_)
            j = hinge();
        in.expect("axleHinges");
        for (auto& j : car->axleHinge_)
            j = hinge();
        in.expect("steering");
        for (auto& j : car->steer_)
            j = hinge();
        in.expect("carJoints");
        const int n = in.integer();
        for (int i = 0; i < n && !in.failed(); ++i)
            car->joints_.push_back(savedJoint());
        in.expect("endCar");
        if (in.failed()) return nullptr;
        return car;
    }
};

void saveWorld(const World& world, SnapshotWriter& out) { SnapshotAccess::saveWorld(world, out); }
bool loadWorld(World& world, SnapshotReader& in) { return SnapshotAccess::loadWorld(world, in); }
void saveCar(const Car& car, const World& world, SnapshotWriter& out) {
    SnapshotAccess::saveCar(car, world, out);
}
std::unique_ptr<Car> loadCar(World& world, SnapshotReader& in) { return SnapshotAccess::loadCar(world, in); }

} // namespace ncotb
