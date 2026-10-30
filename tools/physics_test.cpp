// Headless physics test: runs a crash scenario without any graphics.
#include "../src/car_sim.h"
#include <cstdio>
#include <cstdlib>
using namespace sim;
int main(int argc, char** argv) {
    int scen = argc > 1 ? atoi(argv[1]) : 1;
    double kmh = argc > 2 ? atof(argv[2]) : 56;
    double T = argc > 3 ? atof(argv[3]) : 1.5;
    CarSim s;
    s.loadScenario(scen, kmh);
    const double dt = 1.0 / 4000.0;
    double nextPrint = 0;
    bool nan = false;
    for (double t = 0; t < T; t += dt) {
        s.step(dt);
        if (!(s.com.x == s.com.x)) { nan = true; break; }
        if (t >= nextPrint) {
            s.updateStats();
            printf("t=%.2f v=%6.1f km/h G=%5.1f peak=%5.1f crush=%.2fm W=%.0fkJ dam=%.1f%% broken=%d com=(%.2f %.2f %.2f)\n",
                   t, s.speedKmh, s.curG, s.peakG, s.crushFront, s.plasticWork / 1000.0, s.damagedPct, s.brokenBeams,
                   s.com.x, s.com.y, s.com.z);
            nextPrint += (getenv("FINE") ? 0.01 : 0.1);
        }
    }
    s.updateStats();
    printf("%s impactSpeed=%.1f peakG=%.1f crushFront=%.2f crushWidth=%.2f W=%.0f kJ\n", nan ? "NAN!" : "ok",
           s.impactSpeedKmh, s.peakG, s.crushFront, s.crushWidth, s.plasticWork / 1000.0);
}
