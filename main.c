#define WIDTH 1280
#define HEIGHT 720
#define TARGET_FPS 120
#define N_PARTICLES 8
#define GRAVITY 1200.0f // 9.81 m/s^2 on Earth
#define DAMP_FAC 0.65f // velocity loss factor on collision
#define HEADLESS 0 // run sim backend without window and raylib draw calls

#include <stdlib.h>
#include <raylib.h>

#include "balls.c"

int main(void) {
    srand(time(NULL)); // Initialize randomizer

    // TODO: I kinda only like window resizing if
    // I can keep the sim running during resize. I
    // don't think this is currently possible with
    // raylib framework.
    //SetConfigFlags(FLAG_WINDOW_RESIZABLE);

    SimState state = {};
    state.width = WIDTH;
    state.height = HEIGHT;
    state.show_metrics = true;
    init_sim_state(&state, N_PARTICLES);

    InitWindow(WIDTH, HEIGHT, "balls");
    SetTargetFPS(TARGET_FPS);

    while(!WindowShouldClose()) {
    //    state.width = GetScreenWidth();
    //    state.height = GetScreenHeight();
        state.dt = GetFrameTime(); // secs
        update_sim(&state);

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
                //DrawText(TextFormat("%.2f,%.2f", p->vel.x, p->vel.y), p->pos.x, p->pos.y, 20, RAYWHITE);
            }

            if(state.show_metrics) {
                DrawFPS(10, 10);
                DrawText(TextFormat("%.2fms", state.dt * 1000.0f), 10, 30, 20, RAYWHITE);
            }

            EndDrawing();

        } else {
            // TODO: Add logging
        }
    }

    CloseWindow();
    return 0;
}
