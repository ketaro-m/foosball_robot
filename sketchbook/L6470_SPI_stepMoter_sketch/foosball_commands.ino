/* commands for stepping motor */

long field_size[2] = {590, 345}; //[mm] (width, height)
long max_y = 80; //[mm] the stroke 
long player_size = field_size[1] / 3 - max_y;
uint8_t kick_motors = 0b000000; // if the rotate motor is in the kicking motion (3~5 bits)


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

void defense(long y, int x_index) {
    int motor_num = (x_index-1) / 2; // linear motor in the ball zone
    for (int i = 3; i > motor_num; i -= 1) {
        pos[i+3] = 45;
    } // forward than the ball possessing opposite
    for (int i = motor_num; i >= 0; i -= 1) {
        pos[i+3] = 75;
        pos[i] = y2pos(y);
    } // backward than the ball posessing opposit
    array_copy(pos, pos_copy, N);
    distang2pos(pos);
    uint8_t non_kick_motors = ~kick_motors & non_error_motors;
    L6470_goto_u(non_kick_motors, N, pos);
    array_copy(pos_copy, pos, N);
}

/* kicking motion */
void kick(uint8_t motors) {
    motors = motors & 0b111000; // for safety
    long take_back = ang2pos(45);
    long follow_through = ang2pos(130);
    L6470_goto_u(motors, N, take_back);
    L6470_goto(motors, N, follow_through);
}

void offense(long y, int x_index) {
    int motor_num = x_index / 2; // linear motor in the ball zone
    for (int i = 3; i > motor_num; i -= 1) {
        pos[i+3] = 30;
    } // forward than the kick bar
    for (int i = motor_num-1; i >= 0; i -= 1) {
        pos[i+3] = 90;
    } // backward than the kick bar
    pos[motor_num] = y2pos(y);
    if (checkBit(motor_num+3, kick_motors)==0) {
        kick_motors = bitFlip(motor_num+3, kick_motors);
    }
    uint8_t non_kick_motors = ~kick_motors & non_error_motors;

    array_copy(pos, pos_copy, N);
    distang2pos(pos);
    L6470_goto_u(non_kick_motors, N, pos);
    if (checkflag(kick_motors)) {
        kick(kick_motors);
        kick_motors = 0b0;
    }
    array_copy(pos_copy, pos, N);
}

void command(long x, long y) {
    y = field_size[1] - y;
    int x_index = xIndex(x);
    stepper_msg.data[N*3] = x;
    stepper_msg.data[N*3+1] = y;

    if (x_index % 2 == 1) {
        defense(y, x_index);
    } else {
        offense(y, x_index);
    }
}

/* motion when the error flags occur. Move rotation motor forward dir=1, linear motor backward*/
void error_recovery_motion(uint8_t motors, long *speed) {
    uint8_t rot_motors = motors & 0b111000;
    uint8_t lin_motors = motors & 0b000111;
    if (checkflag(rot_motors)) {
        L6470_run_u(rot_motors, N, 0, speed); // dir=0, backward, or to negative
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
    array_copy(pos, pos_copy, N);
    distang2pos(pos);
    L6470_goto(N, pos);
    array_copy(pos_copy, pos, N);
    L6470_busydelay(2000);
}