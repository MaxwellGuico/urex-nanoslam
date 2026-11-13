#include "experiment_plan.h"
#include <math.h>
experiment_xy_t experiment_corner(unsigned corner, float length, float width)
{
    switch(corner) {
    case 1: return (experiment_xy_t){length,0};
    case 2: return (experiment_xy_t){length,width};
    case 3: return (experiment_xy_t){0,width};
    default: return (experiment_xy_t){0,0};
    }
}
unsigned experiment_waypoint_count(uint8_t mode)
{
    switch (mode) {
    case 2: return 5;   /* home, 3 corners, home */
    case 3: return 6;   /* home, 4 corners, home */
    default: return 1;  /* takeoff-land: home only */
    }
}
experiment_xy_t experiment_waypoint(uint8_t mode, unsigned index, float length, float width)
{
    if (mode == 3) {
        const float half = 0.5f * width;
        switch (index) {
        case 1: return (experiment_xy_t){0, -half};       /* SW */
        case 2: return (experiment_xy_t){length, -half};  /* NW */
        case 3: return (experiment_xy_t){length, half};   /* NE */
        case 4: return (experiment_xy_t){0, half};        /* SE */
        default: return (experiment_xy_t){0, 0};          /* takeoff */
        }
    }
    if (mode == 2) return experiment_corner(index, length, width);
    return (experiment_xy_t){0, 0};
}
experiment_xy_t experiment_rotate(experiment_xy_t body, float heading)
{
    const float c=cosf(heading), s=sinf(heading);
    return (experiment_xy_t){body.x*c-body.y*s, body.x*s+body.y*c};
}
bool experiment_settled(float distance, float speed, float tolerance)
{ return distance <= tolerance && speed <= 0.08f; }
experiment_xy_t experiment_step(experiment_xy_t current, experiment_xy_t goal,
                                float speed, float dt)
{
    float dx=goal.x-current.x, dy=goal.y-current.y, distance=hypotf(dx,dy);
    float step=fmaxf(0,speed*dt);
    if (distance<=step || distance<1e-6f) return goal;
    return (experiment_xy_t){current.x+dx*step/distance,current.y+dy*step/distance};
}
