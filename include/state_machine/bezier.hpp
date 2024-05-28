#ifndef BEZIER_HPP
#define BEZIER_HPP

#include <stddef.h>
#include <stdlib.h>

struct bezier2d
{
    float *xpos;
    float *ypos;
    int npoints;
};

struct bezier3d
{
    float *xpos;
    float *ypos;
    float *zpos;
    int npoints;
};

void bezier2d_init(struct bezier2d *curve);
void bezier2d_addPoint(struct bezier2d *curve, float x, float y);
void bezier2d_getPos(struct bezier2d *curve, float t, float *xret, float *yret);
void bezier2d_generate_curve(struct bezier2d *curve, float startx, float startz, float controlx,
                             float controlz, float endx, float endz);
void bezier2d_generate_straight_back(struct bezier2d *stright_back, float startx, float startz,
                                     float endx, float endy);

void bezier3d_init(struct bezier3d *curve);
void bezier3d_addpoint(struct bezier3d *curve, float x, float y, float z);
void bezier3d_getpos(struct bezier3d *curve, float t, float *xret, float *yret, float *zret);
void bezier3d_generate_curve(struct bezier3d *curve, float startx, float starty, float startz,
                             float controlx, float controly, float controlz, float endx, float endy,
                             float endz);

#endif /*BEZIER_HPP*/