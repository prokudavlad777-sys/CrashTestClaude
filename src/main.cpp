// ============================================================================
//  CRASH TEST SIMULATOR  -  soft-body car crash physics in C++ (raylib)
//  Physics: src/car_sim.h      Rendering / input / HUD: this file
// ============================================================================
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#include "raylib.h"
#include "rlgl.h"

#include "car_sim.h"

namespace S = sim;

// ------------------------------------------------------------------ utils ----
static Vector3 toV(const S::V3& a) { return Vector3{(float)a.x, (float)a.y, (float)a.z}; }
static float lerpf(float a, float b, float t) { return a + (b - a) * t; }
static float clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

// ---------------------------------------------------------------- UI font ----
static Font gFont;
static bool gRu = false;  // true when a font with Cyrillic glyphs was found
#define L(ru, en) (gRu ? (ru) : (en))

static void loadUiFont() {
    std::vector<int> cps;
    for (int c = 32; c < 127; ++c) cps.push_back(c);
    for (int c = 0x0400; c <= 0x04FF; ++c) cps.push_back(c);
    int extra[] = {0xB0, 0xB7, 0xD7, 0x2190, 0x2191, 0x2192, 0x2193, 0x2014};
    for (int c : extra) cps.push_back(c);

    std::string base = GetApplicationDirectory();
    std::vector<std::string> cands = {base + "assets/DejaVuSans.ttf",
                                      "assets/DejaVuSans.ttf",
                                      "../assets/DejaVuSans.ttf",
                                      "C:/Windows/Fonts/segoeui.ttf",
                                      "C:/Windows/Fonts/arial.ttf",
                                      "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"};
    for (const auto& c : cands) {
        if (!FileExists(c.c_str())) continue;
        Font f = LoadFontEx(c.c_str(), 26, cps.data(), (int)cps.size());
        if (f.texture.id > 0 && f.glyphCount > 0) {
            gFont = f;
            SetTextureFilter(gFont.texture, TEXTURE_FILTER_BILINEAR);
            gRu = true;
            return;
        }
    }
    gFont = GetFontDefault();
    gRu = false;
}

static void txt(const std::string& s, float x, float y, float size, Color c) {
    DrawTextEx(gFont, s.c_str(), Vector2{x + 1, y + 1}, size, 1, Color{0, 0, 0, 160});
    DrawTextEx(gFont, s.c_str(), Vector2{x, y}, size, 1, c);
}

// ------------------------------------------------------------------ world ----
struct PaintColor { float r, g, b; };
static const PaintColor kPaints[] = {{196, 30, 36}, {32, 92, 205}, {235, 235, 238}, {236, 178, 20}, {35, 140, 80}};
static const int kPaintCount = 5;

static const S::V3 kSun = S::norm(S::V3(0.45, 0.85, 0.30));

static Color shadeCol(float r, float g, float b, float sh, unsigned char a = 255) {
    auto c = [&](float v) { return (unsigned char)std::min(255.f, std::max(0.f, v * sh)); };
    return Color{c(r), c(g), c(b), a};
}

struct GlassQuad {
    Vector3 p[4];
    Color c;
};

static void emitQuad(const S::V3* p, Color c) {
    rlColor4ub(c.r, c.g, c.b, c.a);
    int tri[6] = {0, 1, 2, 0, 2, 3};
    for (int t = 0; t < 6; ++t) rlVertex3f((float)p[tri[t]].x, (float)p[tri[t]].y, (float)p[tri[t]].z);
}

static void drawShadow(const S::CarSim& s) {
    rlDisableBackfaceCulling();
    rlBegin(RL_TRIANGLES);
    for (const auto& q : s.quads) {
        if (q.kind == S::Q_GLASS) continue;
        S::V3 p[4];
        for (int i = 0; i < 4; ++i) {
            const S::V3& a = s.nodes[q.n[i]].p;
            double t = a.y / kSun.y;
            p[i] = S::V3(a.x - kSun.x * t, 0.015, a.z - kSun.z * t);
        }
        emitQuad(p, Color{66, 70, 74, 255});
    }
    rlEnd();
    rlEnableBackfaceCulling();
}

static void drawSkin(const S::CarSim& s, int paintIdx) {
    static std::vector<GlassQuad> glass;
    glass.clear();
    const PaintColor pc = kPaints[paintIdx % kPaintCount];

    rlDisableBackfaceCulling();
    rlBegin(RL_TRIANGLES);
    for (const auto& q : s.quads) {
        bool torn = false;
        for (int e = 0; e < 4; ++e)
            if (q.eb[e] >= 0 && s.beams[q.eb[e]].broken) torn = true;
        if (torn) continue;

        const S::Node* nn[4] = {&s.nodes[q.n[0]], &s.nodes[q.n[1]], &s.nodes[q.n[2]], &s.nodes[q.n[3]]};
        S::V3 p[4] = {nn[0]->p, nn[1]->p, nn[2]->p, nn[3]->p};
        S::V3 nrm = S::norm(S::cross(p[2] - p[0], p[3] - p[1])) * (double)q.flip;
        float sh = 0.36f + 0.64f * (float)std::max(0.0, S::dot(nrm, kSun)) + 0.12f * (float)std::max(0.0, nrm.y);
        float dmg = (float)(nn[0]->dmg + nn[1]->dmg + nn[2]->dmg + nn[3]->dmg) * 0.25f;

        switch (q.kind) {
        case S::Q_PAINT: {
            float t = clamp01(dmg / 0.16f) * 0.88f;
            float r = lerpf(pc.r, 52, t), g = lerpf(pc.g, 48, t), b = lerpf(pc.b, 46, t);
            emitQuad(p, shadeCol(r, g, b, sh));
        } break;
        case S::Q_UNDER: emitQuad(p, shadeCol(34, 34, 38, sh)); break;
        case S::Q_TRIM: emitQuad(p, shadeCol(30, 30, 34, sh)); break;
        case S::Q_HEAD: emitQuad(p, shadeCol(255, 250, 215, 0.85f + 0.15f * sh)); break;
        case S::Q_TAIL: emitQuad(p, shadeCol(215, 22, 28, 0.8f + 0.2f * sh)); break;
        case S::Q_GLASS: {
            bool cracked = dmg > 0.06f;
            GlassQuad g;
            for (int i = 0; i < 4; ++i) g.p[i] = toV(p[i]);
            g.c = cracked ? shadeCol(190, 205, 215, 0.7f + 0.3f * sh, 215) : shadeCol(70, 105, 140, 0.6f + 0.4f * sh, 175);
            glass.push_back(g);
        } break;
        }
    }
    rlEnd();

    // translucent glass last, without writing depth
    rlDisableDepthMask();
    rlBegin(RL_TRIANGLES);
    for (const auto& g : glass) {
        rlColor4ub(g.c.r, g.c.g, g.c.b, g.c.a);
        int tri[6] = {0, 1, 2, 0, 2, 3};
        for (int t = 0; t < 6; ++t) rlVertex3f(g.p[tri[t]].x, g.p[tri[t]].y, g.p[tri[t]].z);
    }
    rlEnd();
    rlEnableDepthMask();
    rlEnableBackfaceCulling();
}

static void drawBeams(const S::CarSim& s) {
    for (const auto& b : s.beams) {
        if (b.broken) continue;
        float d = clamp01((float)((b.L0 - b.rest) / b.L0) / 0.20f);
        Color c = Color{(unsigned char)lerpf(70, 255, d), (unsigned char)lerpf(220, 40, d), (unsigned char)lerpf(120, 30, d), 255};
        DrawLine3D(toV(s.nodes[b.a].p), toV(s.nodes[b.b].p), c);
    }
}

static void drawWheels(const S::CarSim& s) {
    S::V3 fwd, right, up;
    s.chassisFrame(fwd, right, up);
    S::V3 Y(0, 1, 0);
    S::V3 fh = S::norm(S::V3(fwd.x, 0, fwd.z));
    S::V3 rh = S::cross(fh, Y);
    for (const auto& w : s.wheels) {
        const S::Node& h = s.nodes[w.hub];
        S::V3 wf = fh * std::cos(w.steer) + rh * std::sin(w.steer);
        S::V3 axis = S::cross(wf, Y);
        S::V3 c = S::V3(h.p.x, w.cy, h.p.z) + axis * (0.10 * w.side);
        Vector3 a = toV(c - axis * 0.12), b = toV(c + axis * 0.12);
        DrawCylinderEx(a, b, 0.32f, 0.32f, 28, Color{22, 22, 25, 255});
        DrawCylinderEx(toV(c - axis * 0.125), toV(c + axis * 0.125), 0.205f, 0.205f, 24, Color{178, 180, 188, 255});
        // spokes on the outer face
        S::V3 face = c + axis * (0.128 * w.side);
        for (int k = 0; k < 5; ++k) {
            double ang = w.angle * 1.0 + k * 6.2831853 / 5.0;
            S::V3 dir = wf * std::cos(ang) + Y * std::sin(ang);
            DrawLine3D(toV(face), toV(face + dir * 0.19), Color{50, 50, 56, 255});
        }
    }
}

static void drawObstacles(const S::CarSim& s) {
    for (const auto& o : s.obstacles) {
        if (o.type == 0) {
            Vector3 c = toV(o.c);
            DrawCube(c, (float)o.h.x * 2, (float)o.h.y * 2, (float)o.h.z * 2, Color{150, 150, 155, 255});
            DrawCubeWires(c, (float)o.h.x * 2, (float)o.h.y * 2, (float)o.h.z * 2, Color{60, 60, 66, 255});
            // measuring grid on the -X face (0.5 m squares)
            float fx = (float)(o.c.x - o.h.x) - 0.004f;
            float zmin = std::max((float)(o.c.z - o.h.z), -4.0f), zmax = std::min((float)(o.c.z + o.h.z), 4.0f);
            int zi0 = (int)std::floor(zmin / 0.5f), zi1 = (int)std::ceil(zmax / 0.5f);
            for (int zi = zi0; zi < zi1; ++zi)
                for (int yi = 0; yi < (int)std::ceil(o.h.y * 2 / 0.5); ++yi) {
                    if (((zi + yi) & 1) == 0) continue;
                    DrawCube(Vector3{fx, yi * 0.5f + 0.25f, zi * 0.5f + 0.25f}, 0.004f, 0.5f, 0.5f, Color{128, 128, 134, 255});
                }
        } else {
            DrawCylinder(toV(o.c), (float)o.radius, (float)o.radius, (float)o.height, 20, Color{230, 230, 230, 255});
            for (float y = 0.5f; y + 0.5f <= (float)o.height; y += 1.0f)
                DrawCylinder(Vector3{(float)o.c.x, y, (float)o.c.z}, (float)o.radius + 0.002f, (float)o.radius + 0.002f, 0.5f, 20,
                             Color{205, 40, 40, 255});
        }
    }
}

static void drawFloor() {
    DrawPlane(Vector3{0, 0, 0}, Vector2{700, 700}, Color{96, 100, 105, 255});
    for (int x = -80; x <= 100; x += 5) {
        bool major = (x % 10 == 0);
        DrawCube(Vector3{(float)x, 0.004f, 0}, major ? 0.10f : 0.05f, 0.008f, 10.0f, Color{205, 205, 190, 255});
    }
    DrawCube(Vector3{10, 0.004f, 5.0f}, 180, 0.008f, 0.10f, Color{225, 225, 100, 255});
    DrawCube(Vector3{10, 0.004f, -5.0f}, 180, 0.008f, 0.10f, Color{225, 225, 100, 255});
}

// -------------------------------------------------------------------- app ----
static const char* scenarioName(int id) {
    switch (id) {
    case 1: return L("Фронтальный удар в стену (100%)", "Full frontal wall impact");
    case 2: return L("Смещённый удар 40%", "40% offset impact");
    case 3: return L("Лобовой удар в столб", "Frontal pole impact");
    case 4: return L("Боковой удар в столб", "Side pole impact");
    default: return L("Свободная езда", "Free driving");
    }
}

int main() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);
    InitWindow(1600, 900, "Crash Test Simulator");
    SetTargetFPS(60);
    rlSetClipPlanes(0.1, 800.0);
    loadUiFont();

    S::CarSim car;
    int scenario = 1;
    double launchKmh = 56.0;
    car.loadScenario(scenario, launchKmh);

    // camera state
    float camYaw = 1.78f, camPitch = 0.26f, camDist = 7.5f;
    Vector3 camTarget = toV(car.com);
    auto resetCamera = [&]() {
        camYaw = (scenario == 4) ? 0.60f : 1.78f;
        camPitch = 0.26f;
        camDist = 7.5f;
        camTarget = toV(car.com);
    };
    resetCamera();

    // developer option: CT_SHOTS="30,90" CT_SCENARIO=1..5 CT_SPEED=56  -> saves shot_<frame>.png and exits
    std::vector<int> shotFrames;
    if (const char* e = std::getenv("CT_SHOTS")) {
        std::string t = e;
        size_t pos = 0;
        while (pos < t.size()) {
            size_t c = t.find(',', pos);
            if (c == std::string::npos) c = t.size();
            shotFrames.push_back(std::atoi(t.substr(pos, c - pos).c_str()));
            pos = c + 1;
        }
        if (const char* sc = std::getenv("CT_SCENARIO")) { scenario = std::atoi(sc); }
        if (const char* sp = std::getenv("CT_SPEED")) { launchKmh = std::atof(sp); }
        car.loadScenario(scenario, launchKmh);
        resetCamera();
    }
    int frameNo = 0;

    const double DT = 1.0 / 4000.0;
    double accum = 0;
    const float scales[4] = {1.0f, 0.5f, 0.25f, 0.1f};
    int scaleIdx = 0;
    bool autoSlow = true, paused = false, showHud = true;
    int beamMode = 0;  // 0 skin, 1 skin + beams, 2 beams only
    int paint = 0;
    if (const char* bm = std::getenv("CT_BEAMS")) beamMode = std::atoi(bm);
    double steerState = 0;

    auto reload = [&](int id) {
        scenario = id;
        car.loadScenario(id, launchKmh);
        if (id == 5) car.ctl = S::Controls();
        accum = 0;
        resetCamera();
    };

    while (!WindowShouldClose()) {
        float frameDt = shotFrames.empty() ? std::min(GetFrameTime(), 0.05f) : 1.0f / 60.0f;

        // ----- input -----
        for (int k = 0; k < 5; ++k)
            if (IsKeyPressed(KEY_ONE + k)) reload(k + 1);
        if (IsKeyPressed(KEY_R)) reload(scenario);
        if (IsKeyPressed(KEY_UP)) launchKmh = std::min(160.0, launchKmh + 5.0);
        if (IsKeyPressed(KEY_DOWN)) launchKmh = std::max(10.0, launchKmh - 5.0);
        if (IsKeyPressed(KEY_T)) scaleIdx = (scaleIdx + 1) % 4;
        if (IsKeyPressed(KEY_H)) autoSlow = !autoSlow;
        if (IsKeyPressed(KEY_P)) paused = !paused;
        if (IsKeyPressed(KEY_B)) beamMode = (beamMode + 1) % 3;
        if (IsKeyPressed(KEY_C)) paint = (paint + 1) % kPaintCount;
        if (IsKeyPressed(KEY_F1)) showHud = !showHud;

        // driving
        double thr = 0, brk = 0;
        double fwdSpeed = car.speedForward();
        if (IsKeyDown(KEY_W)) thr = 1.0;
        if (IsKeyDown(KEY_S)) {
            if (fwdSpeed > 1.0) brk = 1.0;
            else thr = -1.0;
        }
        double steerTarget = (IsKeyDown(KEY_D) ? 1.0 : 0.0) - (IsKeyDown(KEY_A) ? 1.0 : 0.0);
        steerState += (steerTarget - steerState) * std::min(1.0, frameDt * 6.0);
        car.ctl.throttle = thr;
        car.ctl.brake = brk;
        car.ctl.steer = steerState;
        car.ctl.handbrake = IsKeyDown(KEY_SPACE) ? 1.0 : 0.0;

        // camera input
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) || IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 d = GetMouseDelta();
            camYaw -= d.x * 0.006f;
            camPitch = std::min(1.45f, std::max(0.03f, camPitch + d.y * 0.006f));
        }
        camDist = std::min(40.0f, std::max(3.0f, camDist - GetMouseWheelMove() * 0.8f));

        // ----- physics -----
        float ts = scales[scaleIdx];
        if (autoSlow && car.impacted && car.timeSinceImpact < 0.35) ts = std::min(ts, 0.15f);
        if (!paused) {
            accum += frameDt * ts;
            int steps = 0;
            while (accum >= DT && steps < 400) {
                car.step(DT);
                accum -= DT;
                ++steps;
            }
            if (steps == 400) accum = 0;
        }
        car.updateStats();

        // ----- camera follow -----
        float k = 1.0f - std::exp(-7.0f * frameDt);
        Vector3 want = toV(car.com);
        want.y += 0.35f;
        camTarget.x += (want.x - camTarget.x) * k;
        camTarget.y += (want.y - camTarget.y) * k;
        camTarget.z += (want.z - camTarget.z) * k;

        float shake = 0;
        if (car.impacted && car.timeSinceImpact >= 0 && car.timeSinceImpact < 0.3)
            shake = 0.10f * (1.0f - (float)car.timeSinceImpact / 0.3f);

        Camera3D cam{};
        cam.target = camTarget;
        cam.position = Vector3{camTarget.x + camDist * std::cos(camPitch) * std::cos(camYaw),
                               camTarget.y + camDist * std::sin(camPitch),
                               camTarget.z + camDist * std::cos(camPitch) * std::sin(camYaw)};
        if (shake > 0) {
            cam.position.x += shake * (GetRandomValue(-100, 100) / 100.0f);
            cam.position.y += shake * (GetRandomValue(-100, 100) / 100.0f);
            cam.position.z += shake * (GetRandomValue(-100, 100) / 100.0f);
        }
        cam.up = Vector3{0, 1, 0};
        cam.fovy = 55.0f;
        cam.projection = CAMERA_PERSPECTIVE;

        // ----- render -----
        BeginDrawing();
        ClearBackground(Color{150, 190, 225, 255});
        DrawRectangleGradientV(0, 0, GetScreenWidth(), GetScreenHeight(), Color{92, 146, 214, 255}, Color{205, 224, 238, 255});
        BeginMode3D(cam);
        drawFloor();
        drawObstacles(car);
        drawShadow(car);
        if (beamMode != 2) {
            drawSkin(car, paint);
            drawWheels(car);
        }
        if (beamMode != 0) drawBeams(car);
        EndMode3D();

        // ----- HUD -----
        const int sw = GetScreenWidth(), sh = GetScreenHeight();
        if (showHud) {
            DrawRectangleRounded(Rectangle{14, 14, 400, 318}, 0.04f, 8, Color{12, 16, 24, 175});
            float y = 24;
            txt(scenarioName(scenario), 28, y, 24, Color{255, 214, 90, 255});
            y += 38;
            auto row = [&](const std::string& label, const std::string& val, Color vc = WHITE) {
                txt(label, 28, y, 20, Color{175, 188, 205, 255});
                DrawTextEx(gFont, val.c_str(), Vector2{262, y}, 20, 1, vc);
                y += 28;
            };
            row(L("Скорость", "Speed"), TextFormat("%.0f %s", car.speedKmh, L("км/ч", "km/h")));
            row(L("Скорость запуска", "Launch speed"), TextFormat("%.0f %s  (%s)", launchKmh, L("км/ч", "km/h"), "\xE2\x86\x91/\xE2\x86\x93"));
            row(L("Скорость удара", "Impact speed"), car.impacted ? TextFormat("%.0f %s", car.impactSpeedKmh, L("км/ч", "km/h")) : "—");
            row(L("Перегрузка", "Deceleration"), TextFormat("%.1f g", car.curG));
            row(L("Пик перегрузки", "Peak G"), TextFormat("%.1f g", car.peakG), car.peakG > 60 ? Color{255, 110, 90, 255} : WHITE);
            row(L("Смятие спереди", "Front crush"), TextFormat("%.0f %s", car.crushFront * 100.0, L("см", "cm")));
            row(L("Смятие по ширине", "Side intrusion"), TextFormat("%.0f %s", car.crushWidth * 100.0, L("см", "cm")));
            row(L("Энергия деформации", "Crush energy"), TextFormat("%.0f %s", car.plasticWork / 1000.0, L("кДж", "kJ")));
            row(L("Повреждено балок", "Damaged beams"), TextFormat("%.0f%%  (%s: %d)", car.damagedPct, L("разрывы", "torn"), car.brokenBeams));

            // G bar
            float gx = 28, gy = y + 4, gw = 372, gh = 10;
            DrawRectangle((int)gx, (int)gy, (int)gw, (int)gh, Color{40, 46, 58, 255});
            float cur = clamp01((float)car.curG / 100.0f), pk = clamp01((float)car.peakG / 100.0f);
            DrawRectangle((int)gx, (int)gy, (int)(gw * cur), (int)gh, Color{255, (unsigned char)(200 - 150 * cur), 60, 255});
            DrawRectangle((int)(gx + gw * pk) - 1, (int)gy - 3, 3, (int)gh + 6, WHITE);

            // controls
            const char* help[] = {
                L("W/S — газ/тормоз/задний   A/D — руль   Пробел — ручник", "W/S gas/brake/reverse   A/D steer   Space handbrake"),
                L("1-5 — сценарий   R — повтор   ↑/↓ — скорость запуска", "1-5 scenario   R restart   Up/Down launch speed"),
                L("T — замедление   H — авто-слоумо   P — пауза", "T slow-mo   H auto slow-mo   P pause"),
                L("B — вид балок   C — цвет кузова   F1 — скрыть HUD", "B beam view   C paint   F1 hide HUD"),
                L("Мышь: вращение камеры, колесо — зум", "Mouse: orbit camera, wheel: zoom")};
            float hy = (float)sh - 26.0f * 5 - 12;
            DrawRectangleRounded(Rectangle{14, hy - 8, 520, 26.0f * 5 + 14}, 0.06f, 8, Color{12, 16, 24, 150});
            for (int i = 0; i < 5; ++i) txt(help[i], 26, hy + i * 26.0f, 18, Color{210, 218, 230, 255});
        }
        if (paused) txt(L("ПАУЗА", "PAUSED"), sw / 2.0f - 50, 20, 30, Color{255, 255, 255, 255});
        if (ts < 0.999f) txt(TextFormat("SLOW-MO  %.2fx", ts), sw - 250.0f, 20, 28, Color{255, 214, 90, 255});
        DrawFPS(sw - 90, sh - 28);
        EndDrawing();

        ++frameNo;
        if (!shotFrames.empty()) {
            for (int f : shotFrames)
                if (f == frameNo) TakeScreenshot(TextFormat("shot_%d.png", f));
            if (frameNo > *std::max_element(shotFrames.begin(), shotFrames.end())) break;
        }
    }

    CloseWindow();
    return 0;
}
