#include <stdio.h> // TODO: Remove stdio.h when print debugging no longer needed
#include <time.h>
#include <math.h> // TODO: Maybe remove math.h if we can remove the need for sqrt out of collision math?

typedef union {
    uint8_t rgba[4];
    struct {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t a;
    };
} RGBA;

typedef union {
    float xy[2];
    struct {
        float x;
        float y;
    };
} Vector2D;

typedef struct {
    uint32_t id;
    Vector2D pos;
    Vector2D vel;
    Vector2D acc;
    float radius;
    float mass;
    float volume;
    float density;
    RGBA color;
} Particle;

// TODO: Implement a container inside the window? 
typedef struct {
    Vector2D pos;
    float width;
    float height;
} Container;

typedef struct {
    Particle* particles;
    uint32_t n_particles;
    float dt;
    uint32_t width;
    uint32_t height;
    float total_kinetic_energy;
} SimState;

void init_sim_state(SimState* state, uint32_t n_particles);
void update_particle(Particle* p, SimState* state);
void update_sim(SimState* state);
