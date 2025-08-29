#define WIDTH 1280
#define HEIGHT 720
#define TARGET_FPS 120
#define N_PARTICLES 128
#define GRAVITY 1200.0f //1200.0f // 9.81 m/s^2 on Earth
#define DAMP_FAC 0.85f // velocity loss factor on collision
#define HEADLESS 0 // run sim backend without window and raylib draw calls
#define SHOW_METRICS 1

#include <stdlib.h>
#include <stdint.h>
#include <raylib.h>

#include "balls.c"

int main(void) {
    srand(time(NULL)); // Initialize randomizer

    // Initialize the particle simulation state
    SimState state = {};
    state.width = WIDTH;
    state.height = HEIGHT;
    init_sim_state(&state, N_PARTICLES);

    InitWindow(WIDTH, HEIGHT, "balls");
    SetTargetFPS(TARGET_FPS);

    while(!WindowShouldClose()) {
        state.dt = GetFrameTime(); // secs between frames
        update_sim(&state); // update the sim state

        if(!HEADLESS) {
            BeginDrawing();
            ClearBackground(BLACK);
            for(int i = 0; i < state.n_particles; ++i) {
                Particle* p = &state.particles[i];
                // NOTE: DrawCircle requires Color object
                // from raylib but I don't want any raylib dependencies in
                // the simulation layer. This way we can throw any frontend
                // on quite easily.
                Color color = (Color){p->color.r, p->color.g, p->color.b, p->color.a};
                DrawCircle(p->pos.x, p->pos.y, p->radius, color);
            }
            if(SHOW_METRICS) {
                DrawFPS(10, 10);
                DrawText(TextFormat("%.2fms", state.dt * 1000.0f), 10, 30, 20, RAYWHITE);
                DrawText(TextFormat("Ke=%.7f", state.total_kinetic_energy), 10, 50, 20, RAYWHITE);
            }
            EndDrawing();
        } else {
            // TODO: Add logging
        }
    }
    CloseWindow();
    return 0;
}
