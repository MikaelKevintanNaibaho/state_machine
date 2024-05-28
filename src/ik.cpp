#include "state_machine/ik.hpp"

float degrees(float rad)
{
    return rad * (180.0 / M_PI);
}

float radians(float deg)
{
    return deg * (M_PI / 180.0);
}

float normalize_angle(float angle)
{
    angle = fmodf(angle,
                  360.0); // Ensure angle is within the range of -360.0 to 360.0

    // Convert negative angles to their corresponding positive angles within the
    // same position
    if (angle < 0)
        angle += 360.0;

    // Ensure angle is within the range of 0.0 to 180.0
    if (angle > 180.0)
        angle = 360.0 - angle;

    return angle;
}

float *get_target(SpiderLeg *leg)
{
    return leg->joints[3];
}

void set_angles(SpiderLeg *leg, float angles[3])
{
    leg->theta1 = normalize_angle(angles[0]);
    leg->theta2 = normalize_angle(angles[1]);
    leg->theta3 = normalize_angle(angles[2]);

    for (int i = 0; i < 3; i++) {
        set_pwm_angle(leg->servo_channles[i], (int)angles[i], PWM_FREQ);
        printf("theta%d: %.2f degrees\n", i + 1, angles[i]);
    }
}

// Helper function to check if two sets of angles are approximately equal
int angles_equal(const float angles1[3], const float angles2[3])
{
    for (int i = 0; i < 3; ++i) {
        if (fabs(angles1[i] - angles2[i]) > 0.01) {
            return 0;
        }
    }
    return 1;
}

void move_to_angle(SpiderLeg *leg, float target_angles[3], int speed)
{
    float current_angles[3] = { leg->theta1, leg->theta2, leg->theta3 };

    // hitung max change dalam angle per step berdasarkan speed
    float delta_theta = DELTA_THETA_MAX * (float)speed / 1.0;

    float delta_directions[3];
    for (int i = 0; i < 3; i++) {
        delta_directions[i] = (target_angles[i] > current_angles[i]) ? 1.0 : -1.0;
    }

    // adjust angle gradually toward the target
    while (!angles_equal(current_angles, target_angles)) {
        for (int i = 0; i < 3; i++) {
            // hitung next angle berdasarkan current_angles dan delta_theta
            float next_angle = current_angles[i] + delta_theta * delta_directions[i];

            // make sure the next angle nggk overshoot dari target angles
            if ((delta_directions[i] > 0 && next_angle > target_angles[i])
                || (delta_directions[i] < 0 && next_angle < target_angles[i])) {
                next_angle = target_angles[i];
            }

            current_angles[i] = next_angle;
        }

        // set servo angle dan kasih delay
        set_angles(leg, current_angles);
        usleep(DELAY_US);
    }
}

void forward_kinematics(SpiderLeg *leg, float angles[3], LegPosition position_leg)
{
    // Convert to radians
    float theta1 = radians(angles[0]);
    float theta2 =
        radians(angles[1]) - radians(90); // -90 because of angle offset of mounting servo
    float theta3 =
        -radians(angles[2]) + radians(90); // -90 because of angle offset of mounting_servo

    float zero_offset = 0.0;
    switch (position_leg) {
    case KANAN_DEPAN:
        zero_offset = 0.0;
        break;
    case KIRI_DEPAN:
        zero_offset = 90.0;
        break;
    case KIRI_BELAKANG:
        zero_offset = 180.0;
        break;
    case KANAN_BELAKANG:
        zero_offset = 90.0;
    default:
        break;
    }
    theta1 += radians(zero_offset);

    DHParameters params_array[NUM_LINKS];
    init_DH_params(&params_array[0], radians(90.0), COXA_LENGTH, 0.0, (theta1 + radians(90.0)));
    init_DH_params(&params_array[1], radians(0.0), FEMUR_LENGTH, 0.0, theta2);
    init_DH_params(&params_array[2], radians(-90.0), TIBIA_LENGTH, 0.0, (theta3 - radians(90.0)));
    init_DH_params(&params_array[3], radians(90.0), 0.0, 0.0, radians(-90.0));

    gsl_matrix *trans_matrix = gsl_matrix_alloc(4, 4);
    calculate_DH_transformation(params_array, NUM_LINKS, trans_matrix);

    float x = fabs(gsl_matrix_get(trans_matrix, 0, 3));

    float y = gsl_matrix_get(trans_matrix, 1, 3);

    if (y < 0) {
        y = -y;
    }
    float z = gsl_matrix_get(trans_matrix, 2, 3);

    const float position[3] = { x, y, z };

    // Update leg joints end-effector
    for (int i = 0; i < 3; i++) {
        leg->joints[3][i] = position[i];
    }

    printf("end-effector position: x = %.2f, y = %.2f, z = %.2f\n", leg->joints[3][0],
           leg->joints[3][1], leg->joints[3][2]);

    gsl_matrix_free(trans_matrix);
}

void inverse_kinematics(SpiderLeg *leg, const float target_positions[3], LegPosition position_leg)
{
    float x = target_positions[0];
    float y = target_positions[1];
    float z = target_positions[2];

    // adjust_coordinate(x, y, z, position_leg, &x, &y, &z);
    // angle antara coxa dengan horizontal plane
    float theta1 = atan2(x, y);

    float P = sqrt(pow(x, 2) + powf(y, 2)) - COXA_LENGTH;

    float G = sqrt(pow(z, 2) + pow(P, 2));

    float alpha = atan2(z, P);

    float gamma_cos =
        (pow(FEMUR_LENGTH, 2) + pow(G, 2) - pow(TIBIA_LENGTH, 2)) / (2 * FEMUR_LENGTH * G);
    float gamma = acos(gamma_cos);

    float beta_cos = (pow(FEMUR_LENGTH, 2) + pow(TIBIA_LENGTH, 2) - pow(G, 2))
        / (2 * FEMUR_LENGTH * TIBIA_LENGTH);
    float beta = acos(beta_cos);

    float theta2 = M_PI / 2 + (gamma - fabs(alpha));

    float theta3 = M_PI - beta;

    // Convert angles to degrees
    theta1 = degrees(theta1);
    theta2 = degrees(theta2);
    theta3 = degrees(theta3);

    const float orientation_offset[NUM_LEGS] = { 0, -90.0, -180.0, -270.0 };

    theta1 += orientation_offset[position_leg];

    // Ensure angles are within the valid range
    theta1 = normalize_angle(theta1);
    theta2 = normalize_angle(theta2);
    theta3 = normalize_angle(theta3);

    if (theta1 > 90) {
        theta1 = 180.0 - theta1;
    }

    float angles[3] = { theta1, theta2, theta3 };
    set_angles(leg, angles);
    forward_kinematics(leg, angles, position_leg);
    // printf("theta1 = %.2f, theta2 = %.2f, theta3 = %.2f\n", theta1, theta2, theta3);
}

