// ============================================================================
//  car_sim.h  -  soft-body (mass-spring / "beam") car physics for crash tests
//
//  The car is a lattice of mass nodes connected by beams. Every beam is an
//  elasto-plastic spring: it behaves elastically until it is compressed or
//  stretched beyond its yield strain, then it deforms permanently (its rest
//  length changes) and absorbs energy. Beams that are stretched too far tear.
//  This is the same basic idea that is used by BeamNG.drive.
//
//  Units: metres, kilograms, seconds, Newtons. Y is up, the car drives +X.
//  No graphics code here, so it can be tested headless (tools/physics_test).
// ============================================================================
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

// ---- tuning knobs (can be overridden with -D for experiments) -------------
#ifndef CS_EA_BASE
#define CS_EA_BASE 3.0e5   // axial stiffness (EA) of ordinary body beams, N
#endif
#ifndef CS_YC
#define CS_YC 0.030        // compressive yield strain of body beams
#endif
#ifndef CS_YT
#define CS_YT 0.035        // tensile yield strain of body beams
#endif
#ifndef CS_ZETA
#define CS_ZETA 0.10       // beam damping ratio
#endif
#ifndef CS_FRONT_MUL
#define CS_FRONT_MUL 1.0   // stiffness multiplier front crumple zone
#endif
#ifndef CS_CAGE_MUL
#define CS_CAGE_MUL 3.2
#endif

namespace sim {

// ----------------------------------------------------------------- math ----
struct V3 {
    double x = 0, y = 0, z = 0;
    V3() {}
    V3(double X, double Y, double Z) : x(X), y(Y), z(Z) {}
    V3 operator+(const V3& o) const { return V3(x + o.x, y + o.y, z + o.z); }
    V3 operator-(const V3& o) const { return V3(x - o.x, y - o.y, z - o.z); }
    V3 operator*(double s) const { return V3(x * s, y * s, z * s); }
    V3 operator/(double s) const { return V3(x / s, y / s, z / s); }
    V3& operator+=(const V3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    V3& operator-=(const V3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    V3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
};
inline double dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(const V3& a, const V3& b) {
    return V3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
inline double len(const V3& a) { return std::sqrt(dot(a, a)); }
inline V3 norm(const V3& a) {
    double l = len(a);
    return l > 1e-12 ? a * (1.0 / l) : V3(0, 0, 0);
}
template <class T> inline T clampv(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ------------------------------------------------------------ structures ----
enum QuadKind { Q_PAINT = 0, Q_GLASS, Q_UNDER, Q_HEAD, Q_TAIL, Q_TRIM };

struct Node {
    V3 p, v, f;
    double m = 1.0;
    int gi = 0, gj = 0, gk = 0;  // lattice indices
    double dmg = 0.0;            // 0..1 damage (for rendering)
};

struct Beam {
    int a = 0, b = 0;
    double L0 = 1, rest = 1;  // original / current rest length
    double k = 1, c = 0;      // stiffness N/m, damping Ns/m
    double yc = 0.02;         // compressive yield strain
    double yt = 0.035;        // tensile yield strain
    double brk = 0.35;        // tearing strain
    bool broken = false;
    bool cage = false;        // passenger cell
};

struct Quad {
    int n[4];
    int eb[4];  // beam indices of the 4 edges (for hiding torn panels)
    int kind;
    int flip = 1;  // +1/-1: makes cross(diag1, diag2) point outwards
};

struct Wheel {
    int hub = 0;
    bool front = false, driven = false;
    int side = 1;  // -1 left, +1 right
    double omega = 0, angle = 0;
    double steer = 0;
    double cy = 0.32;  // wheel centre height (for rendering)
    double load = 0;
    bool ground = false;
};

struct Obstacle {
    int type = 0;     // 0 = box, 1 = vertical pole
    V3 c;             // centre (box) / base centre (pole)
    V3 h;             // half extents (box)
    double radius = 0.15, height = 3.0;  // pole
};

struct Controls {
    double throttle = 0;   // -1..1 (negative = reverse)
    double brake = 0;      // 0..1
    double steer = 0;      // -1..1 (positive = right)
    double handbrake = 0;  // 0..1
};

// ------------------------------------------------------------------ sim ----
class CarSim {
public:
    // lattice dimensions
    static const int NX = 13, NY = 5, NZ = 3;

    std::vector<Node> nodes;
    std::vector<Beam> beams;
    std::vector<Quad> quads;
    std::vector<Obstacle> obstacles;
    Wheel wheels[4];
    Controls ctl;

    // tunables
    double carMass = 1350.0;
    double tireGrip = 1.0;  // multiplier (0 = frictionless sled)
    double timeSinceImpact = -1.0;
    bool impacted = false;

    // telemetry
    double simTime = 0;
    V3 com, vcom;
    double speedKmh = 0;
    double curG = 0, peakG = 0;
    double plasticWork = 0;    // J absorbed by permanent deformation
    double crushFront = 0;     // m
    double crushWidth = 0;     // m
    double damagedPct = 0;     // % beams permanently deformed / torn
    int brokenBeams = 0;
    double impactSpeedKmh = 0;
    double ke0 = 0;  // kinetic energy at impact start (J)

    int idx(int i, int j, int k) const { return (i * NY + j) * NZ + k; }

    CarSim() {
        buildCar();
        spawn(V3(0, 0, 0), 0.0, V3(0, 0, 0));
    }

    // -------------------------------------------------------- scenarios ----
    // 1 full frontal wall, 2 offset deformable-style barrier (40%),
    // 3 front pole, 4 side pole (sled), 5 free driving
    void loadScenario(int id, double kmh) {
        obstacles.clear();
        tireGrip = 1.0;
        const double runup = 7.0;
        double v = kmh / 3.6;
        auto wall = [&](double zc, double hz) {
            Obstacle o;
            o.type = 0;
            o.h = V3(1.5, 1.6, hz);
            o.c = V3(o.h.x, o.h.y, zc);
            obstacles.push_back(o);
        };
        switch (id) {
        default:
        case 1:  // full width frontal
            wall(0.0, 4.0);
            spawn(V3(-(2.1 + runup), 0, 0), 0.0, V3(v, 0, 0));
            break;
        case 2:  // 40 % offset: barrier covers only the left part of the car
            wall(-0.17 - 2.0, 2.0);
            spawn(V3(-(2.1 + runup), 0, 0), 0.0, V3(v, 0, 0));
            break;
        case 3: {  // narrow rigid pole in front of the car centre
            Obstacle o;
            o.type = 1;
            o.c = V3(0, 0, 0);
            o.radius = 0.16;
            o.height = 4.0;
            obstacles.push_back(o);
            spawn(V3(-(2.1 + runup), 0, 0), 0.0, V3(v, 0, 0));
            break;
        }
        case 4: {  // side impact on a pole: car is turned 90 deg and slides sideways
            Obstacle o;
            o.type = 1;
            o.c = V3(0, 0, 0);
            o.radius = 0.16;
            o.height = 4.0;
            obstacles.push_back(o);
            spawn(V3(-(0.85 + 0.16 + runup), 0, 0), 1.5707963267948966, V3(v, 0, 0));
            tireGrip = 0.0;
            break;
        }
        case 5: {  // free driving playground
            wall(0.0, 6.0);
            obstacles.back().c.x = 70.0 + obstacles.back().h.x;
            for (int i = 0; i < 5; ++i) {
                Obstacle o;
                o.type = 1;
                o.c = V3(20.0 + i * 8.0, 0, (i % 2 ? 3.0 : -3.0));
                o.radius = 0.2;
                o.height = 3.0;
                obstacles.push_back(o);
            }
            Obstacle b;
            b.type = 0;
            b.h = V3(1.2, 0.5, 1.2);
            b.c = V3(45.0, 0.5, -1.0);
            obstacles.push_back(b);
            spawn(V3(-6.0, 0, 0), 0.0, V3(0, 0, 0));
            break;
        }
        }
    }

    // ------------------------------------------------------------- spawn ----
    void spawn(const V3& pos, double yaw, const V3& vel) {
        const double cy = std::cos(yaw), sy = std::sin(yaw);
        for (size_t n = 0; n < nodes.size(); ++n) {
            const V3& l = local[n];
            nodes[n].p = V3(l.x * cy + l.z * sy + pos.x, l.y + pos.y, -l.x * sy + l.z * cy + pos.z);
            nodes[n].v = vel;
            nodes[n].f = V3();
            nodes[n].dmg = 0;
        }
        for (auto& b : beams) {
            b.rest = b.L0;
            b.broken = false;
        }
        for (auto& w : wheels) {
            w.omega = (len(V3(vel.x, 0, vel.z)) * (std::fabs(yaw) < 0.1 ? 1.0 : 0.0)) / R_WHEEL;
            w.angle = 0;
            w.steer = 0;
            w.cy = R_WHEEL;
            w.load = 0;
        }
        ctl = Controls();
        timeSinceImpact = -1.0;
        impacted = false;
        simTime = 0;
        peakG = curG = 0;
        plasticWork = 0;
        crushFront = crushWidth = damagedPct = 0;
        brokenBeams = 0;
        impactSpeedKmh = 0;
        ke0 = 0;
        gFilt = V3();
        updateCom();
        vcomPrev = vcom;
        speedKmh = len(vel) * 3.6;
        initFrontLen = frontLen();
        initWidth = 0;
    }

    // -------------------------------------------------------- chassis frame --
    void chassisFrame(V3& fwd, V3& right, V3& up) const {
        V3 a, b, l, r;
        for (int j = 0; j < NY; ++j)
            for (int k = 0; k < NZ; ++k) {
                a += nodes[idx(8, j, k)].p;
                b += nodes[idx(3, j, k)].p;
            }
        for (int i = 3; i <= 8; ++i)
            for (int k = 0; k < NZ; ++k) {
                l += nodes[idx(i, 0, k)].p;
                r += nodes[idx(i, NY - 1, k)].p;
            }
        fwd = norm(a - b);
        V3 rt = norm(r - l);
        up = norm(cross(rt, fwd));
        right = cross(fwd, up);
    }

    double speedForward() const {
        V3 f, r, u;
        chassisFrame(f, r, u);
        return dot(vcom, f);
    }

    // ----------------------------------------------------------- stepping ----
    void step(double dt) {
        const double g = 9.81;
        const size_t NN = nodes.size();
        simTime += dt;

        // gravity + aerodynamic drag
        const double kd = 0.48 / double(NN);
        for (auto& n : nodes) {
            double sp = len(n.v);
            n.f = V3(0, -g * n.m, 0) - n.v * (kd * sp);
        }

        // beams (elasto-plastic)
        for (auto& b : beams) {
            if (b.broken) continue;
            Node& A = nodes[b.a];
            Node& B = nodes[b.b];
            V3 d = B.p - A.p;
            double L = len(d);
            if (L < 1e-9) continue;
            V3 u = d * (1.0 / L);
            double ext = L - b.rest;
            double strain = ext / b.rest;

            if (strain < -b.yc) {  // compressive yield -> permanent shortening
                double nr = L / (1.0 - b.yc);
                double minRest = b.L0 * MIN_REST;
                if (nr < minRest) nr = minRest;
                if (nr < b.rest) {
                    plasticWork += std::fabs(b.k * ext) * (b.rest - nr);
                    b.rest = nr;
                    ext = L - b.rest;
                }
            } else if (strain > b.yt) {  // tensile yield -> permanent stretching
                if (strain > b.brk) {
                    b.broken = true;
                    ++brokenBeams;
                    continue;
                }
                double nr = L / (1.0 + b.yt);
                plasticWork += std::fabs(b.k * ext) * (nr - b.rest);
                b.rest = nr;
                ext = L - b.rest;
            }
            double relv = dot(B.v - A.v, u);
            double F = b.k * ext + b.c * relv;
            V3 fv = u * F;
            A.f += fv;
            B.f -= fv;
        }

        // wheels: suspension + tyres
        wheelForces(dt);

        // integrate (semi-implicit Euler)
        for (auto& n : nodes) {
            n.v += n.f * (dt / n.m);
            n.p += n.v * dt;
        }

        // collisions
        bool hit = false;
        for (auto& n : nodes) {
            collideGround(n);
            for (const auto& o : obstacles)
                if (collideObstacle(n, o)) hit = true;
        }
        if (hit && !impacted) {
            impacted = true;
            timeSinceImpact = 0;
            impactSpeedKmh = len(vcom) * 3.6;
            ke0 = 0.5 * carMass * dot(vcom, vcom);
        }
        if (impacted) timeSinceImpact += dt;

        // telemetry: centre-of-mass deceleration, low-pass filtered (~100 Hz)
        updateCom();
        V3 acc = (vcom - vcomPrev) / dt;
        vcomPrev = vcom;
        acc.y = 0;  // horizontal only
        double alpha = dt / (dt + 1.0 / (2.0 * 3.14159265 * 100.0));
        gFilt += (acc - gFilt) * alpha;
        curG = len(gFilt) / g;
        if (impacted && timeSinceImpact < 1.0) peakG = std::max(peakG, curG);
        speedKmh = len(vcom) * 3.6;
    }

    // statistics that don't need to run every physics step
    void updateStats() {
        for (auto& n : nodes) n.dmg = 0;
        int dam = 0;
        for (const auto& b : beams) {
            double d = b.broken ? 1.0 : std::max(0.0, (b.L0 - b.rest) / b.L0);
            if (d > 0.03) ++dam;
            Node& A = nodes[b.a];
            Node& B = nodes[b.b];
            A.dmg = std::max(A.dmg, d);
            B.dmg = std::max(B.dmg, d);
        }
        damagedPct = 100.0 * dam / double(beams.size());
        crushFront = std::max(0.0, initFrontLen - frontLen());
        crushWidth = std::max(0.0, -widthNow());
    }

    double kineticEnergy() const { return 0.5 * carMass * dot(vcom, vcom); }

private:
    static constexpr double R_WHEEL = 0.32;
    static constexpr double MAX_EXT = 0.12;
    static constexpr double K_SUSP = 42000.0;
    static constexpr double C_SUSP = 3200.0;
    static constexpr double WHEEL_I = 2.2;
    static constexpr double MIN_REST = 0.30;

    std::vector<V3> local;
    V3 vcomPrev, gFilt;
    double initFrontLen = 1.4, initWidth = 1.7;

    void updateCom() {
        V3 c, v;
        double M = 0;
        for (const auto& n : nodes) {
            c += n.p * n.m;
            v += n.v * n.m;
            M += n.m;
        }
        com = c / M;
        vcom = v / M;
    }
    double frontLen() const {
        V3 a, b;
        for (int j = 0; j < NY; ++j)
            for (int k = 0; k < NZ; ++k) {
                a += nodes[idx(NX - 1, j, k)].p;
                b += nodes[idx(8, j, k)].p;
            }
        return len(a - b) / double(NY * NZ);
    }
    // deepest lateral intrusion between the two sides of the passenger cell
    double widthNow() const {
        double best = 0;
        for (int i = 3; i <= 8; ++i)
            for (int k = 0; k < NZ; ++k) {
                int a = idx(i, 0, k), b = idx(i, NY - 1, k);
                double w0 = std::fabs(local[b].z - local[a].z);
                double w = len(nodes[b].p - nodes[a].p);
                best = std::max(best, w0 - w);
            }
        return -best;  // negative = narrowing (see updateStats)
    }

    // ---------------------------------------------------------- build car ----
    void buildCar() {
        const double dx = 0.35, x0 = -2.1;
        const double zs[NY] = {-0.85, -0.425, 0.0, 0.425, 0.85};
        const double topH[NX] = {0.98, 1.00, 1.04, 1.30, 1.44, 1.46, 1.46, 1.44, 1.24, 0.98, 0.93, 0.88, 0.80};
        const double midH[NX] = {0.78, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.75, 0.70, 0.62};
        const double sB[NX] = {0.90, 0.96, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.96, 0.90};
        const double sM[NX] = {0.94, 0.98, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.98, 0.94};
        const double sT[NX] = {0.88, 0.90, 0.92, 0.80, 0.78, 0.78, 0.78, 0.78, 0.80, 0.90, 0.92, 0.92, 0.90};
        const double yFloor = 0.36;

        nodes.assign(NX * NY * NZ, Node());
        local.assign(nodes.size(), V3());
        double wsum = 0;
        std::vector<double> w(nodes.size(), 1.0);
        for (int i = 0; i < NX; ++i)
            for (int j = 0; j < NY; ++j)
                for (int k = 0; k < NZ; ++k) {
                    int id = idx(i, j, k);
                    Node& n = nodes[id];
                    n.gi = i; n.gj = j; n.gk = k;
                    double y = (k == 0) ? yFloor : (k == 1 ? midH[i] : topH[i]);
                    double s = (k == 0) ? sB[i] : (k == 1 ? sM[i] : sT[i]);
                    local[id] = V3(x0 + i * dx, y, zs[j] * s);
                    double wt = 1.0;
                    if (k == 0) wt *= 1.3;
                    if (k == 2) wt *= 0.6;
                    if (i == 9 || i == 10) { if (k <= 1 && j >= 1 && j <= 3) wt *= 3.5; }  // engine block
                    if (i == 11 && k <= 1 && j >= 1 && j <= 3) wt *= 1.5;  // radiator / pump
                    if (i == NX - 1) wt *= 0.35;  // bumper skin is light
                    if (i == NX - 2) wt *= 0.6;
                    if (i == 0) wt *= 0.4;
                    if (i <= 2 && k == 0) wt *= 1.2;
                    if (k == 0 && (j == 0 || j == NY - 1) && (i == 2 || i == 10)) wt *= 4.0;  // hubs
                    w[id] = wt;
                    wsum += wt;
                }
        for (size_t n = 0; n < nodes.size(); ++n) {
            nodes[n].m = carMass * w[n] / wsum;
            nodes[n].p = local[n];
        }

        // beams: every node to its neighbours in a 3x3x3 neighbourhood (13 directions)
        std::unordered_map<uint64_t, int> emap;
        auto key = [](int a, int b) { return (uint64_t)std::min(a, b) * 100000ull + (uint64_t)std::max(a, b); };
        for (int i = 0; i < NX; ++i)
            for (int j = 0; j < NY; ++j)
                for (int k = 0; k < NZ; ++k)
                    for (int di = -1; di <= 1; ++di)
                        for (int dj = -1; dj <= 1; ++dj)
                            for (int dk = -1; dk <= 1; ++dk) {
                                if (!(di > 0 || (di == 0 && dj > 0) || (di == 0 && dj == 0 && dk > 0))) continue;
                                int ni = i + di, nj = j + dj, nk = k + dk;
                                if (ni < 0 || ni >= NX || nj < 0 || nj >= NY || nk < 0 || nk >= NZ) continue;
                                Beam b;
                                b.a = idx(i, j, k);
                                b.b = idx(ni, nj, nk);
                                b.L0 = b.rest = len(local[b.b] - local[b.a]);
                                int lo = std::min(i, ni), hi = std::max(i, ni);
                                double EA = CS_EA_BASE, yc = CS_YC, yt = CS_YT, brk = 0.35;
                                if (lo >= 3 && hi <= 8) {  // passenger cell
                                    EA *= CS_CAGE_MUL; yc = 0.05; yt = 0.07; brk = 0.6; b.cage = true;
                                } else if (hi >= 9 && lo >= 9) {  // front crumple zone: soft nose, stiffer towards the firewall
                                    static const double grade[4] = {0.90, 0.65, 0.45, 0.35};
                                    EA *= CS_FRONT_MUL * grade[lo - 9];
                                } else if (hi <= 2) {  // rear crumple zone
                                    EA *= 0.9;
                                } else {
                                    EA *= 1.6;
                                }
                                b.k = EA / b.L0;
                                double ma = nodes[b.a].m, mb = nodes[b.b].m;
                                double mred = ma * mb / (ma + mb);
                                b.c = 2.0 * CS_ZETA * std::sqrt(b.k * mred);
                                b.yc = yc; b.yt = yt; b.brk = brk;
                                emap[key(b.a, b.b)] = (int)beams.size();
                                beams.push_back(b);
                            }

        // render faces (outer skin of the lattice)
        auto addQuad = [&](int a, int b, int c, int d, int kind) {
            Quad q;
            q.n[0] = a; q.n[1] = b; q.n[2] = c; q.n[3] = d;
            q.kind = kind;
            {
                V3 nr = cross(local[c] - local[a], local[d] - local[b]);
                V3 ctr = (local[a] + local[b] + local[c] + local[d]) * 0.25 - V3(0, 0.75, 0);
                q.flip = dot(nr, ctr) >= 0 ? 1 : -1;
            }
            for (int e = 0; e < 4; ++e) {
                auto it = emap.find(key(q.n[e], q.n[(e + 1) % 4]));
                q.eb[e] = it == emap.end() ? -1 : it->second;
            }
            quads.push_back(q);
        };
        for (int jj : {0, NY - 1})  // sides
            for (int i = 0; i < NX - 1; ++i)
                for (int k = 0; k < NZ - 1; ++k) {
                    int kind = (k == 1 && (i == 4 || i == 6)) ? Q_GLASS : Q_PAINT;
                    addQuad(idx(i, jj, k), idx(i + 1, jj, k), idx(i + 1, jj, k + 1), idx(i, jj, k + 1), kind);
                }
        for (int i = 0; i < NX - 1; ++i)  // underbody and top
            for (int j = 0; j < NY - 1; ++j) {
                addQuad(idx(i, j, 0), idx(i + 1, j, 0), idx(i + 1, j + 1, 0), idx(i, j + 1, 0), Q_UNDER);
                bool glass = (j == 1 || j == 2) && (i == 2 || i == 3 || i == 7 || i == 8);
                addQuad(idx(i, j, NZ - 1), idx(i + 1, j, NZ - 1), idx(i + 1, j + 1, NZ - 1), idx(i, j + 1, NZ - 1),
                        glass ? Q_GLASS : Q_PAINT);
            }
        for (int ii : {0, NX - 1})  // front and back
            for (int j = 0; j < NY - 1; ++j)
                for (int k = 0; k < NZ - 1; ++k) {
                    int kind;
                    if (k == 0) kind = Q_TRIM;
                    else if (j == 0 || j == NY - 2) kind = (ii == 0) ? Q_TAIL : Q_HEAD;
                    else kind = Q_PAINT;
                    addQuad(idx(ii, j, k), idx(ii, j + 1, k), idx(ii, j + 1, k + 1), idx(ii, j, k + 1), kind);
                }

        // wheels
        int hubs[4] = {idx(10, 0, 0), idx(10, NY - 1, 0), idx(2, 0, 0), idx(2, NY - 1, 0)};
        for (int i = 0; i < 4; ++i) {
            wheels[i].hub = hubs[i];
            wheels[i].front = i < 2;
            wheels[i].driven = i >= 2;
            wheels[i].side = (i % 2 == 0) ? -1 : 1;
        }
    }

    // ----------------------------------------------------- wheels/tyres ----
    void wheelForces(double dt) {
        V3 fwd, right, up;
        chassisFrame(fwd, right, up);
        V3 Y(0, 1, 0);
        V3 fh = norm(V3(fwd.x, 0, fwd.z));
        V3 rh = cross(fh, Y);
        double speed = std::fabs(dot(vcom, fh));
        double maxSteer = 0.55 / (1.0 + speed * 0.10);
        double steerAng = ctl.steer * maxSteer;
        const double mu = 1.15;

        for (auto& w : wheels) {
            Node& h = nodes[w.hub];
            double s = w.front ? steerAng : 0.0;
            w.steer = s;
            V3 wf = fh * std::cos(s) + rh * std::sin(s);
            V3 wl = cross(wf, Y);

            double hubH = h.p.y;
            double comp = (R_WHEEL + MAX_EXT) - hubH;
            double N = 0, Fl = 0;
            w.ground = comp > 0 && up.y > 0.2;
            double ext = clampv(hubH - R_WHEEL, -0.06, MAX_EXT);
            w.cy = hubH - (w.ground ? ext : MAX_EXT);
            if (w.ground) {
                double spring = K_SUSP * comp;
                if (comp > MAX_EXT + 0.04) spring += 400000.0 * (comp - MAX_EXT - 0.04);  // bump stop
                double damper = C_SUSP * (-h.v.y);
                N = std::max(0.0, spring + damper);
                h.f.y += N;

                double vl = dot(h.v, wf), vt = dot(h.v, wl);
                double slipL = vl - w.omega * R_WHEEL;
                double slipT = vt;
                double cN = 3.5 * N;
                double fl = -cN * slipL, ft = -cN * slipT;
                double maxF = mu * tireGrip * N;
                double mag = std::sqrt(fl * fl + ft * ft);
                if (mag > maxF) {
                    double sc = (mag > 1e-9) ? 0.92 * maxF / mag : 0.0;
                    fl *= sc; ft *= sc;
                }
                h.f += wf * fl + wl * ft;
                Fl = fl;
            }
            w.load = N;

            // wheel rotation
            double T = 0;
            if (w.driven && ctl.throttle != 0.0) {
                double wmax = std::max(std::fabs(w.omega), 8.0);
                double Tmax = std::min(720.0, 60000.0 / wmax);
                T += (ctl.throttle > 0 ? ctl.throttle : ctl.throttle * 0.45) * Tmax;
            }
            T -= clampv(w.omega * 1.5, -35.0, 35.0);  // engine/rolling drag
            T -= Fl * R_WHEEL;
            w.omega += T / WHEEL_I * dt;
            double Tb = ctl.brake * (w.front ? 2000.0 : 1500.0);
            if (!w.front) Tb += ctl.handbrake * 4000.0;
            if (Tb > 0) {
                double dw = Tb / WHEEL_I * dt;
                if (std::fabs(w.omega) <= dw) w.omega = 0;
                else w.omega -= (w.omega > 0 ? dw : -dw);
            }
            w.omega = clampv(w.omega, -400.0, 400.0);
            w.angle += w.omega * dt;
        }
    }

    // --------------------------------------------------------- collisions ----
    static void applyContact(Node& n, const V3& nrm, double mu) {
        double vn = dot(n.v, nrm);
        if (vn < 0) {
            n.v -= nrm * vn;  // perfectly inelastic
            V3 vt = n.v;
            double sp = len(vt);
            double red = mu * (-vn);
            if (sp > red) n.v = vt * ((sp - red) / sp);
            else n.v = V3();
        }
    }

    void collideGround(Node& n) {
        const double r = 0.04;
        if (n.p.y < r) {
            n.p.y = r;
            applyContact(n, V3(0, 1, 0), 0.7);
        }
    }

    bool collideObstacle(Node& n, const Obstacle& o) {
        const double r = 0.03;
        if (o.type == 0) {
            V3 d = n.p - o.c;
            double px = o.h.x + r - std::fabs(d.x);
            double py = o.h.y + r - std::fabs(d.y);
            double pz = o.h.z + r - std::fabs(d.z);
            if (px <= 0 || py <= 0 || pz <= 0) return false;
            V3 nrm;
            if (px <= py && px <= pz) { nrm = V3(d.x >= 0 ? 1 : -1, 0, 0); n.p.x += nrm.x * px; }
            else if (py <= pz)        { nrm = V3(0, d.y >= 0 ? 1 : -1, 0); n.p.y += nrm.y * py; }
            else                      { nrm = V3(0, 0, d.z >= 0 ? 1 : -1); n.p.z += nrm.z * pz; }
            applyContact(n, nrm, 0.5);
            return true;
        } else {
            double dx = n.p.x - o.c.x, dz = n.p.z - o.c.z;
            double dist = std::sqrt(dx * dx + dz * dz);
            double rr = o.radius + r;
            if (dist >= rr || n.p.y > o.height) return false;
            V3 nrm = dist > 1e-9 ? V3(dx / dist, 0, dz / dist) : V3(-1, 0, 0);
            n.p.x = o.c.x + nrm.x * rr;
            n.p.z = o.c.z + nrm.z * rr;
            applyContact(n, nrm, 0.5);
            return true;
        }
    }
};

}  // namespace sim
