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
    return a.x * b.y + a.y * b.y;
}

// TODO: Decide whether I like these functions or not
Vector2D sum_vect2D(Vector2D a, Vector2D b) {
    return (Vector2D){a.x + b.x, a.y + b.y};
}
Vector2D sub_vect2D(Vector2D a, Vector2D b) {
    return (Vector2D){a.x - b.x, a.y - b.y};
}
Vector2D mul_vect2D(Vector2D a, Vector2D b) {
    return (Vector2D){a.x * b.x, a.y * b.y};
}
Vector2D div_vect2D(Vector2D a, Vector2D b) {
    return (Vector2D){a.x / b.x, a.y / b.y};
}

void init_sim_state(SimState* state, uint32_t n_particles) {
    state->n_particles = n_particles;
    state->particles = (Particle*) malloc(n_particles * sizeof(Particle));
    Particle* p;
    for(int i = 0; i < state->n_particles; ++i) {
        p = &state->particles[i];
        p->id = i;
        p->radius = randf(10.0f, 50.0f);
        p->pos = (Vector2D){randf(p->radius, state->width - p->radius), randf(p->radius, state->height - p->radius)};
        p->vel = (Vector2D){randf(-1000.0f, 1000.0f), randf(-1000.0f, 1000.0f)};
        p->acc = (Vector2D){0.0f, 0.0f};
        p->volume = (4.0f / 3.0f) * PI * p->radius * p->radius * p->radius; // volume of sphere formula
        p->mass = p->radius * 3.0f;
        p->density = p->mass / p->volume;
        p->color = (RGBA){randu(0, 255), randu(0, 255), randu(0, 255), 255};
    }
}

void update_particle(Particle* p, SimState* state) {
    float dt = state->dt;
    float width = (float) state->width;
    float height = (float) state->height;

    p->acc.x = 0;
    p->acc.y = 0;

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

// TODO: Finish implementing particle collision
/*
    Particle* p1 = p;
    // Handle for particle collision
    for(int i = 0; i < state->n_particles; ++i) {
        Particle* p2 = &state->particles[i];
        if(p1->id != p2->id) { // skip calculation on self
            float dx = p2->pos.x - p1->pos.x; // also normal vector at point of collision
            float dy = p2->pos.y - p1->pos.y;
            float dist = sqrt(dx * dx + dy * dy); // distance between centers
            float min_dist = p1->radius + p2->radius; // minimum distance between centers given radii
            if(dist <= min_dist) { // collision has occurred
                // We need to find the new vel vector for each particle after collision

                float vx = p2->vel.x - p1->vel.x;
                float vy = p2->vel.y - p1->vel.y;
                Vector2D relative_vel = (Vector2D){vx, vy};

                float nx = dx / dist; // unit normal vector
                float ny = dy / dist; // dist is magnitude
                float tx = -ny; // unit tangent vector
                float ty = nx;

                Vector2D normal = (Vector2D){nx, ny};
                Vector2D tangent = (Vector2D){tx, ty};
                float dotp = dotp2D(relative_vel, normal);

                Vectro2D v1n = dotp2D(normal, p1->vel);
                Vector2D v1t = dotp2D(tangent, p1->vel);
                Vector2D v2n = dotp2D(normal, p2->vel);
                Vector2D v2t = dotp2D(tangent, p2->vel);

                Vector2D v1t_p = v1t;
                Vector2D v2t_p = v2t;

                float total_mass = p1->mass + p2->mass;

                v1n.vel.x * (p1->mass - p2->mass) + 2.0f * p1->mass * v2n.vel.x;
                v1n.vel.y * (p1->mass - p2->mass);

                v2n.vel.x *= (p2->mass - p1->mass);
                v2n.vel.y *= (p2->mass - p1->mass);

            }
        }
    }
*/
}

// NOTE: Not sure if this really needs to be its own
// function, unless we add new items to the sim that
// are not considered particles.
void update_sim(SimState* state) {
    Particle* p;
    for(int i = 0; i < state->n_particles; ++i) {
        p = &state->particles[i];
        update_particle(p, state);
    }
}
