#include "state_machine/leg.hpp"

static SpiderLeg leg_kiri_depan;
static SpiderLeg leg_kiri_belakang;
static SpiderLeg leg_kanan_belakang;
static SpiderLeg leg_kanan_depan;

SpiderLeg *legs[NUM_LEGS] = { &leg_kiri_depan, &leg_kiri_belakang, &leg_kanan_belakang,
                              &leg_kanan_depan };

// Define leg positions
LegPosition leg_positions[NUM_LEGS];

// Define initial angles for the stance position
float stance_angles[NUM_LEGS][3];

void initialize_leg(SpiderLeg *leg, const char *name, int servo_ch1, int servo_ch2, int servo_ch3)
{
    strcpy(leg->name, name);
    leg->servo_channles[0] = servo_ch1;
    leg->servo_channles[1] = servo_ch2;
    leg->servo_channles[2] = servo_ch3;
}

void initialize_all_legs()
{
    // Initialize each leg individually
    initialize_leg(&leg_kiri_depan, "Kiri Depan", SERVO_CHANNEL_1, SERVO_CHANNEL_2,
                   SERVO_CHANNEL_3);
    initialize_leg(&leg_kiri_belakang, "Kiri Belakang", SERVO_CHANNEL_4, SERVO_CHANNEL_5,
                   SERVO_CHANNEL_6);
    initialize_leg(&leg_kanan_belakang, "Kanan Belakang", SERVO_CHANNEL_7, SERVO_CHANNEL_8,
                   SERVO_CHANNEL_9);
    initialize_leg(&leg_kanan_depan, "Kanan Depan", SERVO_CHANNEL_10, SERVO_CHANNEL_11,
                   SERVO_CHANNEL_12);

    // Define leg positions
    leg_positions[0] = KIRI_DEPAN;
    leg_positions[1] = KIRI_BELAKANG;
    leg_positions[2] = KANAN_BELAKANG;
    leg_positions[3] = KANAN_DEPAN;

    // Define initial angles for the stance position
    stance_angles[0][0] = 45.0;
    stance_angles[0][1] = SUDUT_AWAL;
    stance_angles[0][2] = SUDUT_AWAL;

    stance_angles[1][0] = 45.0;
    stance_angles[1][1] = SUDUT_AWAL;
    stance_angles[1][2] = SUDUT_AWAL;

    stance_angles[2][0] = 45.0;
    stance_angles[2][1] = SUDUT_AWAL;
    stance_angles[2][2] = SUDUT_AWAL;

    stance_angles[3][0] = 45.0;
    stance_angles[3][1] = SUDUT_AWAL;
    stance_angles[3][2] = SUDUT_AWAL;
}
