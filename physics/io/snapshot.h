#pragma once

#include "math/quat.h"

#include <istream>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

namespace ncotb {

class World;
class Car;
class RigidBody;

/// Snapshots: everything that decides how a World steps on, as readable text,
/// so a loaded world steps exactly (bit for bit) like the original. Bodies
/// (shape, mass, pose, momentum, material), joints with their limits, motors
/// and accumulated impulses, springs, the contact warm-start cache, the
/// settings and the time. Numbers are written with 17 significant digits,
/// which round-trips doubles exactly.
///
/// Not saved: mouse joints (a grab is input, not state), and the current
/// contacts, which the next step finds again (detectCollisions() shows them).
///
/// The format is line based: a keyword, then values; '#' starts a comment.
/// Saving and loading go in the same fixed order, and loading is strict: any
/// mismatch stops with an error naming the line.

/// Writes values and keywords (one line per item) for saving.
class SnapshotWriter {
public:
    explicit SnapshotWriter(std::ostream& out);
    /// Starts a line with a keyword; values follow.
    SnapshotWriter& key(const char* keyword);
    SnapshotWriter& operator<<(Real v);
    SnapshotWriter& operator<<(int v);
    SnapshotWriter& operator<<(const Vec3& v);
    SnapshotWriter& operator<<(const Quat& q);
    SnapshotWriter& word(const std::string& w); ///< a single token (no spaces)
    void comment(const std::string& text);
    void end(); ///< ends the current line

private:
    std::ostream& out_;
    bool open_ = false;
};

/// Reads what a SnapshotWriter wrote. On a mismatch, failed() turns true and
/// error() says where; later reads return zeros, so callers can check once.
class SnapshotReader {
public:
    explicit SnapshotReader(std::istream& in);
    /// Expects the next line to start with keyword.
    bool expect(const char* keyword);
    /// The keyword of the next line, without consuming it ("" at the end).
    std::string peek();
    Real real();
    int integer();
    Vec3 vec3();
    Quat quat();
    std::string word();
    bool failed() const { return !error_.empty(); }
    const std::string& error() const { return error_; }
    void fail(const std::string& message);

private:
    bool nextToken(std::string& token);
    bool readAhead();
    std::istream& in_;
    std::vector<std::string> tokens_; ///< the rest of the current line
    size_t pos_ = 0;
    int line_ = 0;
    /// The next line with tokens, once peek() has read it. Peeking reads ahead
    /// rather than seeking back: tellg/seekg on a text-mode stream with CRLF
    /// line ends lands in the wrong place on Windows (MinGW).
    std::vector<std::string> ahead_;
    bool hasAhead_ = false;
    int linesRead_ = 0;
    int aheadLine_ = 0;
    std::string error_;
};

/// Writes the world's complete state.
void saveWorld(const World& world, SnapshotWriter& out);
/// Replaces the world's contents with a saved state. False (and the reader's
/// error) if the snapshot doesn't parse; the world may then be partly loaded.
bool loadWorld(World& world, SnapshotReader& in);

/// Writes a car: its spec and which bodies and joints of the world are its parts.
void saveCar(const Car& car, const World& world, SnapshotWriter& out);
/// Restores a car onto the parts of a world loaded from the same snapshot.
std::unique_ptr<Car> loadCar(World& world, SnapshotReader& in);

/// Index of body b in world.bodies(), or -1 for the world's ground anchor.
int bodyIndex(const World& world, const RigidBody* b);

} // namespace ncotb
