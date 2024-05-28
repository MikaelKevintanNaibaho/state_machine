#ifndef DH_H
#define DH_H

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_vector.h>

typedef struct
{
    float alpha;
    float a;
    float d;
    float theta;
} DHParameters;

void init_DH_params(DHParameters *params, float alpha, float a, float d, float theta);
void create_DH_matrix(const DHParameters *params, gsl_matrix *matrix);
void calculate_DH_transformation(const DHParameters *params_array, int num_links,
                                 gsl_matrix *result);

#endif /*DH_H*/