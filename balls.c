#include "balls.h"

// Returns a random float between [min, max]
float randf(float min, float max) {
    // TODO: assert lower < upper
    return ((float)rand() / RAND_MAX) * (max - min) + min;
}

// Returns a random usigned integer between [min, max]
uint32_t randu(uint32_t min, uint32_t max) {
    return (rand() % (max - min + 1)) + min;
}

// Returns the dot product between two 2D vectors
float dotp2D(Vector2D a, Vector2D b) {
    return a.x * b.x + a.y * b.y;
}

void init_sim_state(SimState* state, uint32_t n_particles) {
    state->n_particles = n_particles;
    state->particles = (Particle*) malloc(n_particles * sizeof(Particle));
    Particle* p;
    for(int i = 0; i < state->n_particles; ++i) {
        p = &state->particles[i];
        p->id = i;
        p->radius = 50.0f;
        p->pos = (Vector2D){randf(p->radius, state->width - p->radius), randf(p->radius, state->height - p->radius)};
        p->vel = (Vector2D){randf(-1000.0f, 1000.0f), randf(-1000.0f, 1000.0f)};
        p->acc = (Vector2D){0.0f, 0.0f};
        p->volume = (4.0f / 3.0f) * PI * p->radius * p->radius * p->radius; // volume of sphere formula
        p->density = 1.0f; //randf(1.0f, 100.0f);
        p->mass = p->density * p->volume;
        p->color = (RGBA){randu(0, 255), randu(0, 255), randu(0, 255), 255};
    }
}

void update_particle(Particle* p, SimState* state) {
    float dt = state->dt;
    float width = (float) state->width;
    float height = (float) state->height;

    p->acc.x = 0.0f;
    p->acc.y = GRAVITY;

    p->vel.x += p->acc.x * dt;
    p->vel.y += p->acc.y * dt;

    p->pos.x += p->vel.x * dt;
    p->pos.y += p->vel.y * dt;

    // Handle boundary collision
    if(p->pos.x - p->radius <= 0.0f) {
        p->pos.x = p->radius;
        p->vel.x *= -DAMP_FAC;
    }
    if(p->pos.x + p->radius >= width) {
        p->pos.x = (width - p->radius);
        p->vel.x *= -DAMP_FAC;
    }
    if(p->pos.y - p->radius <= 0.0f) {
        p->pos.y = p->radius;
        p->vel.y *= -DAMP_FAC;
    }
    if(p->pos.y + p->radius >= height) {
        p->pos.y = (height - p->radius);
        p->vel.y *= -DAMP_FAC;
    }
}

void resolve_collision(Particle* p1, Particle* p2) {
    // NOTE: Since collision updates the velocity of both 
    // balls involved we can iterate starting at p1->id + 1
    // so we don't have to check twice.
    //global var reset every step
    float dx = p2->pos.x - p1->pos.x; // also normal vector at point of collision
    float dy = p2->pos.y - p1->pos.y;
    float dist = sqrt(dx * dx + dy * dy); // distance between centers
    float min_dist = p1->radius + p2->radius; // minimum distance between centers given radii
    if(dist < min_dist && dist > 0.0000001f) { // collision has occurred
        float nx = dx / dist; // unit normal vector
        float ny = dy / dist; // dist is magnitude
        float tx = -ny; // unit tangent vector
        float ty = nx;

        Vector2D normal = (Vector2D){nx, ny};
        Vector2D tangent = (Vector2D){tx, ty};

        float rx = nx * 0.5f * (min_dist - dist);
        float ry = ny * 0.5f * (min_dist - dist);
        p1->pos.x -= rx;
        p1->pos.y -= ry;
        p2->pos.x += rx;
        p2->pos.y += ry;

        float v1n = dotp2D(normal, p1->vel);
        float v1t = dotp2D(tangent, p1->vel);
        float v2n = dotp2D(normal, p2->vel);
        float v2t = dotp2D(tangent, p2->vel);

        float v1t_p = v1t;
        float v2t_p = v2t;

        float total_mass = p1->mass + p2->mass;
        float v1n_p = (v1n * (p1->mass - p2->mass) + 2.0f * p2->mass * v2n) / total_mass;
        float v2n_p = (v2n * (p2->mass - p1->mass) + 2.0f * p1->mass * v1n) / total_mass;

        p1->vel = (Vector2D){v1n_p * nx + v1t_p * tx, v1n_p * ny + v1t_p * ty};
        p2->vel = (Vector2D){v2n_p * nx + v2t_p * tx, v2n_p * ny + v2t_p * ty};
    }
}

double calc_total_kinetic_energy(SimState* state) {
    double ke = 0.0f; // total kinetic energy
    for(int i = 0; i < state->n_particles; ++i) {
        Particle* p = &state->particles[i];
        ke += (0.5f * p->mass * (p->vel.x * p->vel.x + p->vel.y * p->vel.y));
    }
    state->total_kinetic_energy = ke;
    return ke;
}

// NOTE: Not sure if this really needs to be its own
// function, unless we add new items to the sim that
// are not considered particles.
void update_sim(SimState* state) {
    for(int i = 0; i < state->n_particles; ++i) {
        Particle* p1 = &state->particles[i];
        update_particle(p1, state);
        for(int j = p1->id + 1; j < state->n_particles; ++j) {
            Particle* p2 = &state->particles[j];
            resolve_collision(p1, p2);
        }
    }
    calc_total_kinetic_energy(state);
}
