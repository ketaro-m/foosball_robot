/* commands for stepping motor */

// #include "L6470_SPI_stepMoter_sketch.ino"

long field_size[2] = {590, 345}; //[mm] (width, height)
long max_y = 85; // 
long player_size = field_size[1] / 3 - max_y;
boolean kick_flag[3] = {false, false, false}; // true if the rotate motor is in the kicking motion


/* judge which x-axis part of 6 (left:0, right 5) the ball is in. even:me, odd:opponent. */
int xIndex(long x) {
    int index = x / (field_size[0]/6);
    return index;
}
/* return bar position to place a player to y position. */
long y2pos(long y) {
    long y_tmp = y % (field_size[1] / 3);
    return min(max(y_tmp - player_size/2, 0), max_y);
}

void defense(long y) {
    pos[3] = pos[4] = pos[5] = 90;
    // later change to more complex
    pos[0] = pos[1] = pos[2] = y2pos(y);
    stepper_msg.data[26] = y2pos(y);
    distang2pos(pos);
    L6470_goto_u(N, pos);
}

void offense(long y, int x_index) {
    int motor_num = x_index / 2; // motor in the ball zone

    pos[0] = pos[1] = pos[2] = y2pos(y);
    // move other motors to opening position
    for (int i = 0; i < 3; i += 1) {
        if (i != motor_num) {
            pos[i+3] = 10;
        }
    }
    // if (kick_flag[motor_num] == false) {
    //     kick(motor_num+3);
    // } else {
    //     pos
    // }
}

void command(long x, long y) {

    y = field_size[1] - y;
    int x_index = xIndex(x);
    stepper_msg.data[25] = x_index;
    if (x_index % 2 == 1) {
        defense(y);
    } else {
        // offense(y, x_index);
        defense(y);
    }
    

}

/* motion when the error flags occur. Move rotation motor forward dir=1, linear motor backward*/
void error_recovery_motion(uint8_t motors, long *speed) {
    uint8_t rot_motors = motors & 0b111000;
    uint8_t lin_motors = motors & 0b000111;
    if (checkflag(rot_motors)) {
        L6470_run_u(rot_motors, N, 1, speed); // dir=1, forward, or to positive
    }
    if (checkflag(lin_motors)) {
        L6470_run_u(lin_motors, N, 0, speed); // dir=0, backward, or to negative
    }

}


void initial_setup() {
    uint8_t motors = 0b111111;
    long speed[N] = {0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000};
    error_recovery_motion(motors, speed);
    /* until all motors have been reset. */
    while (checkflag(motors)) {
        uint8_t reset_pin = check_reset_pin();
        motors = motors & ~reset_pin;
    }
    long pos[N] = {30, 30, 30, 90, 90, 90};
    distang2pos(pos);
    L6470_goto(N, pos);
    L6470_busydelay(1000);
}