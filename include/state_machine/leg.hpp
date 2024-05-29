#ifndef LEG_HPP
#define LEG_HPP

#include <string.h>
#include <stdlib.h>

#define NUM_LINKS 4
#define NUM_LEGS 4

#define SERVO_CHANNEL_1 1
#define SERVO_CHANNEL_2 2
#define SERVO_CHANNEL_3 3
#define SERVO_CHANNEL_4 4
#define SERVO_CHANNEL_5 5
#define SERVO_CHANNEL_6 6
#define SERVO_CHANNEL_7 7
#define SERVO_CHANNEL_8 8
#define SERVO_CHANNEL_9 9
#define SERVO_CHANNEL_10 10
#define SERVO_CHANNEL_11 11
#define SERVO_CHANNEL_12 12

#define COXA_LENGTH 60.4
#define FEMUR_LENGTH 78.0
#define TIBIA_LENGTH 167.23

#define SUDUT_AWAL 90.0

typedef struct
{
    char name[20];
    float COXA;
    float FEMUR;
    float TIBIA;
    float theta1;
    float theta2;
    float theta3;
    float mounted_angle;
    float joints[4][3]; // Joint positions: [0] - start joint, [1] - coxa-femur joint, [2] -
                        // femur-tibia joint, [3] - tip of the leg
    int servo_channles[3];
} SpiderLeg;

typedef enum
{
    KANAN_DEPAN,
    KANAN_BELAKANG,
    KIRI_BELAKANG,
    KIRI_DEPAN
} LegPosition;

extern SpiderLeg *legs[NUM_LEGS];
extern LegPosition leg_positions[NUM_LEGS];
extern float stance_angles[NUM_LEGS][3];

void initialize_leg(SpiderLeg *leg, const char *name, int servo_ch1, int servo_ch2, int servo_ch3);
void initialize_all_legs();

#endif /*LEG_HPP*/