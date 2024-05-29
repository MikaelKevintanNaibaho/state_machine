#include "state_machine/move.hpp"

void generate_walk_trajectory(struct bezier2d *curve, SpiderLeg *leg, float stride_length,
                              float swing_height, LegPosition position_leg)
{
    // get current position
    float startx = leg->joints[3][0] - stride_length;
    float startz = leg->joints[3][2];
    printf("startx = %f, startz %f\n", startx, startz);

    // control points
    float controlx = startx + stride_length / 2;

    printf("controlx = %f \t", controlx);
    float controlz = startz + 2 * swing_height;
    printf("controlz = %f\n", controlz);
    float endx_forward = startx + stride_length;
    float endz_forward = startz;
    // buar bezier curve
    bezier2d_generate_curve(curve, startx, startz, controlx, controlz, endx_forward, endz_forward);

    float startx_2 = endx_forward;
    float startz_2 = endz_forward;

    float controlx_2 = startx_2 - stride_length / 2;
    float controlz_2 = startz_2 - swing_height / 2;

    float endx_2 = startx_2 - stride_length;
    float endz_2 = startz_2;

    bezier2d_generate_curve(curve, startx_2, startz_2, controlx_2, controlz_2, endx_2, endz_2);
}

void generate_walk_back_leg(struct bezier2d *curve, SpiderLeg *leg, float stride_length,
                            float swing_height, LegPosition leg_position)
{
    float startx = leg->joints[3][0];
    float startz = leg->joints[3][2];
    printf("startx = %f, startz %f\n", startx, startz);

    float controlx = startx - stride_length / 2;

    printf("controlx = %f \t", controlx);
    float controlz = startz + 2 * swing_height;
    printf("controlz = %f\n", controlz);
    float endx_forward = startx - stride_length;
    float endz_forward = startz;
    // buar bezier curve
    bezier2d_generate_curve(curve, startx, startz, controlx, controlz, endx_forward, endz_forward);

    float startx_2 = endx_forward;
    float startz_2 = endz_forward;

    float controlx_2 = startx_2 + stride_length / 2;
    float controlz_2 = startz_2 - swing_height / 2;

    float endx_2 = startx_2 + stride_length;
    float endz_2 = startz_2;

    bezier2d_generate_curve(curve, startx_2, startz_2, controlx_2, controlz_2, endx_2, endz_2);

    // // Append straight line for moving backward
    // bezier2d_addPoint(curve, endx_forward, endz_forward);
    // bezier2d_addPoint(curve, startx, startz);
}

void bezier2d_generate_straight_back(struct bezier2d *stright_back, float startx, float startz,
                                     float endx, float endy)
{
    bezier2d_addPoint(stright_back, startx, startz);
    bezier2d_addPoint(stright_back, endx, endy);
}

void generate_stright_back_trajectory(struct bezier2d *stright_back, SpiderLeg *leg,
                                      float stride_length)
{
    float startx = leg->joints[3][0];
    float startz = leg->joints[3][2];
    printf("startx : %f", startx);

    float endx = startx - stride_length / 2;
    float endz = startz;

    bezier2d_generate_straight_back(stright_back, startx, startz, endx, endz);
}
void generate_circular_trajectory(struct bezier3d *curve, SpiderLeg *leg, float radius, 
                                  float swing_height, float angle_offset) {
    // Calculate the current position of the leg
    float startx = leg->joints[3][0];
    float starty = leg->joints[3][1];
    float startz = leg->joints[3][2];

    // Calculate the angle for the circular motion
    float angle = atan2(starty, startx) + angle_offset;

    // Define the control point and end point for the circular trajectory
    float controlx = radius * cos(angle);
    float controly = radius * sin(angle) - radius / 2;
    float controlz = startz;

    float endx = radius * cos(angle + M_PI / 2); // 90 degree increment for the next point on the circle
    float endy = radius * sin(angle + M_PI / 2);
    float endz = startz;

    // Generate the Bezier curve with the specified points
    bezier3d_generate_curve(curve, startx, starty, startz, controlx, controly, controlz, endx, endy, endz);
}
void print_trajectory(struct bezier2d *curve, int num_points)
{
    printf("Trajectory Points:\n");
    for (int i = 0; i <= num_points; i++) {
        float t = (float)i / num_points;
        float x, y;
        bezier2d_getPos(curve, t, &x, &y);
        printf("Point %d: (%.2f, %.2f)\n", i, x, y);
    }
}

void print_trajectory_3d(struct bezier3d *curve, int num_points) {
    printf("Trajectory Points:\n");
    for (int i = 0; i <= num_points; i++) {
        float t = (float)i / num_points;
        float x, y, z;
        bezier3d_getpos(curve, t, &x, &y, &z);
        printf("Point %d: (%.2f, %.2f, %.2f)\n", i, x, y, z);
    }
}


void save_trajectory_points(struct bezier2d *curve, const char *filename, int num_points)
{
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        printf("Error opening file.\n");
        return;
    }

    for (int i = 0; i <= num_points; i++) {
        float t = (float)i / num_points;
        float x, z;
        bezier2d_getPos(curve, t, &x, &z);
        fprintf(file, "%.2f %.2f\n", x, z);
    }

    fclose(file);
}

void update_leg_wave_gait(struct bezier2d curve[NUM_LEGS], int num_points,
                          SpiderLeg *legs[NUM_LEGS], LegPosition leg_positions[NUM_LEGS])
{
    float desired_duration = DESIRED_TIME;
    float dt = desired_duration / num_points;

    for (int i = 0; i <= num_points; i++) {
        float t = (float)i / num_points;

        // Calculate phase offsets for each leg
        float phase_offsets[NUM_LEGS];
        for (int j = 0; j < NUM_LEGS; j++) {
            phase_offsets[j] = fmod(t + j * (1.0 / NUM_LEGS), 1.0);
        }

        // Calculate positions for each leg based on the phase offsets
        float x[NUM_LEGS], z[NUM_LEGS];
        for (int j = 0; j < NUM_LEGS; j++) {
            bezier2d_getPos(&curve[j], phase_offsets[j], &x[j], &z[j]);
        }

        for (int j = 0; j < NUM_LEGS; j++) {
            printf("Y value at joints[3][1] for leg %d: %f\n", j, legs[j]->joints[3][1]);
        }

        // Update leg positions using inverse kinematics
        for (int j = 0; j < NUM_LEGS; j++) {
            printf("------------------------------\n");
            float pos[3] = { x[j], legs[j]->joints[3][1], z[j] };
            inverse_kinematics(legs[j], pos, leg_positions[j]);
            printf("Leg Position: %s\n", leg_position_to_string(leg_positions[j]));
            usleep(10000);
        }

        usleep((long)(dt * 1e6));
    }
}
void update_leg_trot_gait(struct bezier2d curve[NUM_LEGS], int num_points,
                          SpiderLeg *legs[NUM_LEGS], LegPosition leg_positions[NUM_LEGS])
{
    float desired_duration = DESIRED_TIME;
    float dt = desired_duration / num_points;

    for (int i = 0; i <= num_points; i++) {
        float t = (float)i / num_points;

        // Calculate phase offsets for each leg in a crawl gait
        float phase_offsets[NUM_LEGS];
        for (int j = 0; j < NUM_LEGS; j++) {
            // Adjust phase offsets for diagonal leg movement
            phase_offsets[j] = fmod(t + (j % 2 == 0 ? 0.25 : 0.75), 1.0);
        }

        // Calculate positions for each leg based on the phase offsets
        float x[NUM_LEGS], z[NUM_LEGS];
        for (int j = 0; j < NUM_LEGS; j++) {
            bezier2d_getPos(&curve[j], phase_offsets[j], &x[j], &z[j]);
        }

        // Update leg positions using inverse kinematics
        for (int j = 0; j < NUM_LEGS; j++) {
            printf("------------------------------\n");
            // For crawl gait, adjust leg positions to create diagonal movement
            // float z_offset = (j % 2 == 0) ? LEG_HEIGHT_OFFSET : -LEG_HEIGHT_OFFSET;

            //   if (j == 1 || j == 4) {
            //     z_offset *= -1;
            // }
            float pos[3] = { x[j], legs[j]->joints[3][1], z[j] };
            inverse_kinematics(legs[j], pos, leg_positions[j]);

            printf("Leg Position: %s\n", leg_position_to_string(leg_positions[j]));
        }

        usleep((long)(dt * 1e6));
    }
}
void update_leg_left(struct bezier3d curve[NUM_LEGS], int num_points, SpiderLeg *legs[NUM_LEGS],
                     LegPosition leg_positions[NUM_LEGS]) {
    float desired_duration = DESIRED_TIME;
    float dt = desired_duration / num_points;

    // Define the adjusted gait pattern for each leg pair (phase offsets)
    float phase_offsets[NUM_LEGS] = { 0.0, 0.5, 0.25, 0.75 }; // Adjusted phase offsets for alternating swing and stable phases

    for (int i = 0; i <= num_points; i++) {
        float t = (float)i / num_points;

        // Update positions for each leg based on the adjusted gait pattern
        float x[NUM_LEGS], y[NUM_LEGS], z[NUM_LEGS];
        for (int j = 0; j < NUM_LEGS; j++) {
            float phase_offset = fmod(t + phase_offsets[j % 2], 1.0); // Alternate between 0.0 and 0.5 for swing and stable phases
            bezier3d_getpos(&curve[j], phase_offset, &x[j], &y[j], &z[j]);
        }

        // Update leg positions using inverse kinematics
        for (int j = 0; j < NUM_LEGS; j++) {
            float pos[3] = { x[j], legs[j]->joints[3][1], z[j] };
            inverse_kinematics(legs[j], pos, leg_positions[j]);
        }

        usleep((long)(dt * 1e6));
    }
}

const char *leg_position_to_string(LegPosition position)
{
    switch (position) {
    case KIRI_DEPAN:
        return "KIRI_DEPAN";
    case KIRI_BELAKANG:
        return "KIRI_BELAKANG";
    case KANAN_BELAKANG:
        return "KANAN_BELAKANG";
    case KANAN_DEPAN:
        return "KANAN_DEPAN";
    default:
        return "Unknown";
    }
}

void stand_position(void)
{
    for (int i = 0; i < NUM_LEGS; i++) {
        printf("standby position for Leg %s (Position %d):\n", legs[i]->name, leg_positions[i]);
        set_angles(legs[i], stance_angles[i]);
        forward_kinematics(legs[i], stance_angles[i], leg_positions[i]);
        printf("----------------------------\n");
    }
}

void move_forward(void)
{
    struct bezier2d curve[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) {
        bezier2d_init(&curve[i]);
        if (leg_positions[i] == KANAN_BELAKANG || leg_positions[i] == KIRI_BELAKANG) {
            generate_walk_back_leg(&curve[i], legs[i], STRIDE_LENGTH, SWING_HEIGHT,
                                   leg_positions[i]);
        } else {
            generate_walk_trajectory(&curve[i], legs[i], STRIDE_LENGTH, SWING_HEIGHT,
                                     leg_positions[i]);
        }
        print_trajectory(&curve[i], 30);
    }

    while (is_program_running) {
        update_leg_trot_gait(curve, NUM_POINTS, legs, leg_positions);
        usleep(100);
    }
}

void move_left_turn(void)
{
    struct bezier3d curve[NUM_LEGS];
    float radius = 150.0;
    float angle_offsets[4] = {
        0.0,               // Front Left Leg
        M_PI / 2,          // Front Right Leg (90 degrees)
        M_PI,              // Back Right Leg (180 degrees)
        3 * M_PI / 2       // Back Left Leg (270 degrees)
    };
    for(int i = 0; i < NUM_LEGS; i++) {
        bezier3d_init(&curve[i]);
        generate_circular_trajectory(&curve[i], legs[i], radius, SWING_HEIGHT, angle_offsets[i]);
        print_trajectory_3d(&curve[i], NUM_POINTS);
    }
    
    while(is_program_running) {
        update_leg_left(curve, NUM_POINTS, legs, leg_positions);
        usleep(100000);
    }
}

void move_right_turn(void)
{
    struct bezier3d curve[NUM_LEGS];
    float radius = 150.0;
    float angle_offsets[4] = {
        M_PI / 2,          // Front Left Leg (90 degrees)
        M_PI,              // Front Right Leg (180 degrees)
        3 * M_PI / 2,      // Back Right Leg (270 degrees)
        2 * M_PI           // Back Left Leg (360 degrees, equivalent to 0 degrees)
    };
    for(int i = 0; i < NUM_LEGS; i++) {
        bezier3d_init(&curve[i]);
        generate_circular_trajectory(&curve[i], legs[i], radius, SWING_HEIGHT, angle_offsets[i]);
    }
    
    while(is_program_running) {
        update_leg_left(curve, NUM_POINTS, legs, leg_positions);
        usleep(100000);
    }
}

void adjust_leg_positions(float pitch, float roll, SpiderLeg *legs[NUM_LEGS])
{
    for (int i = 0; i < NUM_LEGS; i++) {
        // Adjust leg positions based on pitch
        if (fabs(pitch) > PITCH_THRESHOLD) {
            float adjustment = pitch > 0 ? LEG_ADJUSTMENT_ANGLE : -LEG_ADJUSTMENT_ANGLE;
            legs[i]->theta1 += adjustment;
        }

        // Adjust leg positions based on roll
        if (fabs(roll) > ROLL_THRESHOLD) {
            float adjustment = roll > 0 ? LEG_ADJUSTMENT_ANGLE : -LEG_ADJUSTMENT_ANGLE;
            legs[i]->theta2 += adjustment;
            legs[i]->theta3 -= adjustment; // Adjust opposite leg segment to maintain balance
        }

        // Ensure leg angles are within valid range
        legs[i]->theta1 = normalize_angle(legs[i]->theta1);
        legs[i]->theta2 = normalize_angle(legs[i]->theta2);
        legs[i]->theta3 = normalize_angle(legs[i]->theta3);
    }
}

void self_balance(float roll, float pitch)
{
    while (is_program_running) {

        // Adjust leg positions based on pitch and roll
        adjust_leg_positions(pitch, roll, legs);

        // Update leg positions with adjusted angles
        for (int i = 0; i < NUM_LEGS; i++) {
            float angles[3] = { legs[i]->theta1, legs[i]->theta2, legs[i]->theta3 };
            set_angles(legs[i], angles);
        }

        // Add delay before next iteration to control loop frequency
        usleep(100); // Adjust as needed based on desired loop frequency
    }
}
