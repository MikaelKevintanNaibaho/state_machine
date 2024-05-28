#include "state_machine/dh.hpp"

void init_DH_params(DHParameters *params, float alpha, float a, float d, float theta)
{
    params->alpha = alpha;
    params->a = a;
    params->d = d;
    params->theta = theta;
}

void create_DH_matrix(const DHParameters *params, gsl_matrix *matrix)
{
    float alpha = params->alpha;
    float theta = params->theta;

    // Fill the DH matrix
    gsl_matrix_set(matrix, 0, 0, cos(theta));
    gsl_matrix_set(matrix, 0, 1, -sin(theta) * cos(alpha));
    gsl_matrix_set(matrix, 0, 2, sin(theta) * sin(alpha));
    gsl_matrix_set(matrix, 0, 3, params->a * cos(theta));

    gsl_matrix_set(matrix, 1, 0, sin(theta));
    gsl_matrix_set(matrix, 1, 1, cos(theta) * cos(alpha));
    gsl_matrix_set(matrix, 1, 2, -cos(theta) * sin(alpha));
    gsl_matrix_set(matrix, 1, 3, params->a * sin(theta));

    gsl_matrix_set(matrix, 2, 0, 0.0);
    gsl_matrix_set(matrix, 2, 1, sin(alpha));
    gsl_matrix_set(matrix, 2, 2, cos(alpha));
    gsl_matrix_set(matrix, 2, 3, params->d);

    gsl_matrix_set(matrix, 3, 0, 0.0);
    gsl_matrix_set(matrix, 3, 1, 0.0);
    gsl_matrix_set(matrix, 3, 2, 0.0);
    gsl_matrix_set(matrix, 3, 3, 1.0);
}

void calculate_DH_transformation(const DHParameters *params_array, int num_links,
                                 gsl_matrix *result)
{
    // Identity matrix
    gsl_matrix *identityMatrix = gsl_matrix_alloc(4, 4);
    gsl_matrix_set_identity(identityMatrix);

    // Iterate through each link and calculate intermediate matrices
    for (int i = 0; i < num_links; i++) {
        gsl_matrix *linkMatrix = gsl_matrix_alloc(4, 4);
        create_DH_matrix(&params_array[i], linkMatrix);

        // Multiply identityMatrix and linkMatrix
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, identityMatrix, linkMatrix, 0.0, result);

        // Update the identityMatrix with the multiplied matrix
        gsl_matrix_memcpy(identityMatrix, result);

        // Free the linkMatrix since it's not needed anymore
        gsl_matrix_free(linkMatrix);
    }

    // Free the identityMatrix
    gsl_matrix_free(identityMatrix);
}
