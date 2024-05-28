#include "state_machine/bezier.hpp"

void bezier2d_init(struct bezier2d *curve)
{
    curve->xpos = NULL;
    curve->ypos = NULL;
    curve->npoints = 0;
}

void bezier2d_addPoint(struct bezier2d *curve, float x, float y)
{
    curve->npoints++;
    curve->xpos = (float *)realloc(curve->xpos, curve->npoints * sizeof(float));
    curve->ypos = (float *)realloc(curve->ypos, curve->npoints * sizeof(float));
    curve->xpos[curve->npoints - 1] = x;
    curve->ypos[curve->npoints - 1] = y;
}

void bezier2d_getPos(struct bezier2d *curve, float t, float *xret, float *yret)
{
    int ii, ij;
    float *x, *y;

    if (curve->npoints == 0) {
        *xret = 0;
        *yret = 0;
        return;
    }

    x = (float *)malloc(curve->npoints * sizeof(float));
    y = (float *)malloc(curve->npoints * sizeof(float));

    // load with current points
    for (ii = 0; ii < curve->npoints; ii++) {
        x[ii] = curve->xpos[ii];
        y[ii] = curve->ypos[ii];
    }

    // iterate over levels
    for (ii = 0; ii < curve->npoints - 1; ii++) {
        for (ij = 0; ij < curve->npoints - ii - 1; ij++) {
            x[ij] = (1.0 - t) * x[ij] + t * x[ij + 1];
            y[ij] = (1.0 - t) * y[ij] + t * y[ij + 1];
        }
    }

    *xret = x[0];
    *yret = y[0];

    free(x);
    free(y);
}

void bezier2d_generate_curve(struct bezier2d *curve, float startx, float startz, float controlx,
                             float controlz, float endx, float endz)
{
    bezier2d_addPoint(curve, startx, startz);
    bezier2d_addPoint(curve, controlx, controlz);
    bezier2d_addPoint(curve, endx, endz);
}

void bezier3d_init(struct bezier3d *curve)
{
    curve->xpos = NULL;
    curve->ypos = NULL;
    curve->zpos = NULL;
    curve->npoints = 0;
}

void bezier3d_addpoint(struct bezier3d *curve, float x, float y, float z)
{
    curve->npoints++;
    curve->xpos = (float *)realloc(curve->xpos, curve->npoints * sizeof(float));
    curve->ypos = (float *)realloc(curve->ypos, curve->npoints * sizeof(float));
    curve->zpos = (float *)realloc(curve->zpos, curve->npoints * sizeof(float));
    curve->xpos[curve->npoints - 1] = x;
    curve->ypos[curve->npoints - 1] = y;
    curve->zpos[curve->npoints - 1] = z;
}

void bezier3d_getpos(struct bezier3d *curve, float t, float *xret, float *yret, float *zret)
{
    int ii, ij;
    float *x, *y, *z;

    if (curve->npoints == 0) {
        *xret = 0;
        *yret = 0;
        *zret = 0;
        return;
    }

    x = (float *)malloc(curve->npoints * sizeof(float));
    y = (float *)malloc(curve->npoints * sizeof(float));
    z = (float *)malloc(curve->npoints * sizeof(float));

    // load with the current points
    for (ii = 0; ii < curve->npoints; ii++) {
        x[ii] = curve->xpos[ii];
        y[ii] = curve->ypos[ii];
        z[ii] = curve->zpos[ii];
    }

    // iterate over levels
    for (ii = 0; ii < curve->npoints - 1; ii++) {
        for (ij = 0; ij < curve->npoints - ii - 1; ij++) {
            x[ij] = (1.0 - t) * x[ij] + t * x[ij + 1];
            y[ij] = (1.0 - t) * y[ij] + t * y[ij + 1];
            z[ij] = (1.0 - t) * z[ij] + t * z[ij + 1];
        }
    }

    *xret = x[0];
    *yret = y[0];
    *zret = z[0];

    free(x);
    free(y);
    free(z);
}
void bezier3d_generate_curve(struct bezier3d *curve, float startx, float starty, float startz,
                             float controlx, float controly, float controlz, float endx, float endy,
                             float endz)
{
    bezier3d_addpoint(curve, startx, starty, startz);
    bezier3d_addpoint(curve, controlx, controly, controlz);
    bezier3d_addpoint(curve, endx, endy, endz);
}