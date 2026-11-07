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
