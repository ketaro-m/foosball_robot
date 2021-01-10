#include <SPI.h>
#include <MsTimer2.h>
// #include <math.h>
#include <ros.h>
// #include <sensor_msgs/Joy.h> // Joy stick
#include <opencv_apps/Circle.h>
#include <std_msgs/Int32MultiArray.h> // to publish pin values
ros::NodeHandle nh;
// #include "L6470_commands.ino"
// #include "L6470_commands_multi.ino"
// #include "foosball_commands.ino"

// number of motors
#define N 6
// define pins
#define PIN_SPI_MOSI 51
#define PIN_SPI_MISO 50
#define PIN_SPI_SCK 52
#define PIN_SPI_SS 53
#define PIN_BUSY 9
const int ERROR_PIN[N] = {2,3,4,5,6,7}; // digital pins
const int RESET_PIN[N] = {54,55,56,57,58,59}; // analog pins A0,...,A5 
const int PSEUDO_ERROR_PIN[3] = {62, 63, 64}; // エラーを自分で再現するため
// parameters according to this machine environment
int ROT = 200*pow(2, 7); // steps per 1 rotation
int STROKE = 40; // stroke[mm] per 1 rotation

long param[N]; // array to get and store register values such as abs_pos, speed, etc.
/* position array to control motors */
long pos[N] = {0, 0, 0, 0, 0, 0};
long pos_copy[N] = {0, 0, 0, 0, 0, 0}; // copy pos because distang2pos change its values.
uint8_t non_error_motors = 0b111111; // mth bit is 0 if the mth motor steps out
uint8_t error_motion = 0b000000; // if mth motor is in error recovery motion or not (in order to avoid double command sending)
// long error_recovery_speed = 0x6000; // motor speed of error recovery motion.
long error_recovery_speed[N] = {0x6000, 0x6000, 0x6000, 0x3000, 0x3000, 0x3000}; // motor speed of error recovery motion.
long reset_pos[N] = {0, 0, 0, 180, 180, 180}; // set positions when the reset pin is pushed.
long reset_pos2[N] = {5, 5, 5, 170, 170, 170}; // positions to move just after touching reset pin in order to avoid touching pins forever

void stepper_cb(const opencv_apps::Circle& msg);
ros::Subscriber<opencv_apps::Circle> stepper_sub("ball_position", stepper_cb);
std_msgs::Int32MultiArray stepper_msg;
ros::Publisher stepper_pub("stepper", &stepper_msg);


void stepper_setup() {
  stepper_msg.data_length = N*3 + 2;
  stepper_msg.data = (int32_t *)malloc(sizeof(int32_t)*(N*3+2));
  for (int i = 0; i < N*3+2; i += 1) {
    stepper_msg.data[i] = 0;
  }

  nh.subscribe(stepper_sub);
  nh.advertise(stepper_pub);
}

/* publish the motors' state [abs_pos*6, error_pin*6, reset_pin*6, ball_position(x,y)] */
void stepper_publish_loop() {
  L6470_getparam_abspos(param, N);
  pos2distang(param);
  for (int i = 0; i < N; i += 1) {
    stepper_msg.data[i] = param[i];
    stepper_msg.data[i+N] = digitalRead(ERROR_PIN[i]);
    stepper_msg.data[i+N*2] = digitalRead(RESET_PIN[i]);
  }
  stepper_pub.publish(&stepper_msg);
}

/* subscribe ball_position and control stepper motors */
void stepper_cb(const opencv_apps::Circle& msg) {
  command((long)msg.center.x, (long)msg.center.y);
  // for (int i = 0; i < N; i += 1) {
  //   stepper_msg.data[i+N*3] = pos[i];
  // }
}

void setup()
{
  distang2pos(reset_pos);
  distang2pos(reset_pos2);
  // delay(1000);
  nh.initNode();
  stepper_setup();

  pinMode(PIN_SPI_MOSI, OUTPUT);
  pinMode(PIN_SPI_MISO, INPUT);
  pinMode(PIN_SPI_SCK, OUTPUT);
  pinMode(PIN_SPI_SS, OUTPUT);
  pinMode(PIN_BUSY, INPUT);
  for (int i=0; i<N; i+=1) {
    pinMode(ERROR_PIN[i], INPUT);
    /* use RESET PINs (analog pin) as digital pull down pins. */
    pinMode(RESET_PIN[i], OUTPUT);
    digitalWrite(RESET_PIN[i], HIGH);
  }
  // あとで消す
  for (int i=0; i<3; i+=1) {
    pinMode(PSEUDO_ERROR_PIN[i], OUTPUT);
    digitalWrite(PSEUDO_ERROR_PIN[i], HIGH);
  }
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  // Serial.begin(115200);
  digitalWrite(PIN_SPI_SS, HIGH);
 
  L6470_resetdevice(N); //reset all motors
  L6470_setup(0b111111);
  delay(3000);
  initial_setup();
  
  // MsTimer2::set(100, fulash);//シリアルモニター用のタイマー割り込み
  // MsTimer2::start();
  // uint8_t mo = 0b111000;
  // L6470_run_u(mo, N, 0, 0x6000);
}

unsigned long publish_timer = 0;
unsigned long motor_timer = 0;
void loop(){
  unsigned long now = millis();

  if ((now - publish_timer) > 100) {
    stepper_publish_loop();
    publish_timer = now;
  }

  nh.spinOnce();
  delay(1);

  check_reset_pin();
  check_error_pin();



  // L6470_gohome(N);
  // delay(2000);
  // int8_t mo = 0b111000;
  // L6470_run_u(mo, N, 0, 0x3000);
  // delay(1000);
  // L6470_hardstop_u(mo, N);
  // delay(2000);
  

  /* basic move for multi motors*/
  // L6470_goto(N, ang2pos(-45));
  // for (int i = 0; i < 10; i += 1) {
  //   long pos_array[N];
  //   for (int j = 0; j < N; j += 1) {
  //     if (j == i%N && j >= 3) {
  //       pos_array[j] = ang2pos(180);
  //     } else {
  //       pos_array[j] = 0;
  //     }
  //   }
  //   // dist2pos(N,pos_array);
  //   L6470_goto(N, pos_array);
  // }
  // L6470_goto(N, ang2pos(45));
  // L6470_hardhiz(N);
  // L6470_busydelay(2000);
}

/* setup motors. */
void L6470_setup(uint8_t motors){
  long acc[N] = {0x500, 0x500, 0x500, 0x500, 0x500, 0x500};
  long dec[N] = {0x500, 0x500, 0x500, 0x500, 0x500, 0x500};
  long maxspeed[N] = {0x100, 0x100, 0x100, 0x100, 0x100, 0x100};
  long kvalhold[N] = {0x10, 0x10, 0x10, 0x80, 0x80, 0x80};
  long kvalrun[N] = {0x10, 0x10, 0x10, 0x80, 0x80, 0x80};
  long kvalacc[N] = {0x10, 0x10, 0x10, 0x80, 0x80, 0x80};
  long kvaldec[N] = {0x10, 0x10, 0x10, 0x80, 0x80, 0x80};
  L6470_setparam_acc(motors, N, acc); //[R, WS] 加速度default 0x08A (12bit) (14.55*val+14.55[step/s^2])
  L6470_setparam_dec(motors, N, dec); //[R, WS] 減速度default 0x08A (12bit) (14.55*val+14.55[step/s^2])
  L6470_setparam_maxspeed(motors, N, maxspeed); //[R, WR]最大速度default 0x041 (10bit) (15.25*val+15.25[step/s])
  L6470_setparam_minspeed(motors, N, 0x01); //[R, WS]最小速度default 0x000 (1+12bit) (0.238*val[step/s])
  L6470_setparam_fsspd(motors, N, 0x3ff); //[R, WR]μステップからフルステップへの切替点速度default 0x027 (10bit) (15.25*val+7.63[step/s])
  L6470_setparam_kvalhold(motors, N, kvalhold); //[R, WR]停止時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
  L6470_setparam_kvalrun(motors, N, kvalrun); //[R, WR]定速回転時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
  L6470_setparam_kvalacc(motors, N, kvalacc); //[R, WR]加速時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
  L6470_setparam_kvaldec(motors, N, kvaldec); //[R, WR]減速時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
  L6470_setparam_alareen(motors, N, 0x30); // alarm enable, stall detection
  L6470_setparam_stallth(motors, N, 0xff); // 脱調検知の閾値 need tuning
  L6470_setparam_ocdth(motors, N, 0xf);

  L6470_setparam_stepmood(motors, N, 0x07); //ステップモードdefault 0x07 (1+3+1+3bit) : 1/2^n*1.8[deg] が 1step 
} 


/* functions to change distance[mm] or angle[degree] to steps(pos)*/
long dist2pos(long distance) {
  return ROT * distance / STROKE;
}
long ang2pos(long angle) {
  return ROT * (angle) / 360;
}
long pos2dist(long position) {
  return position * STROKE / ROT;
}
long pos2ang(long position) {
  return (position) * 360 / ROT;
}
void distang2pos(long *distang) {
  for (int i = 0; i < N; i += 1) {
    if (i < 3) {
      distang[i] = dist2pos(distang[i]);
    } else {
      distang[i] = ang2pos(distang[i]);
    }
  }
}
void pos2distang(long *position) {
  for (int i = 0; i < N; i += 1) {
    if (i < 3) {
      position[i] = pos2dist(position[i]);
    } else {
      position[i] = pos2ang(position[i]);
    }
  }
}

/* return true if the N lease bits (pointing at N motors) of uint8_t flag contains at least one 1 value.
this is for not sending unnecessary nothing commands. */
boolean checkflag(uint8_t flag) {
  return (flag << (8 - N)) > 0;
}


/* check reset pins and */
uint8_t check_reset_pin() {
  uint8_t reset_motors = 0b000000;
  for (int i = 0; i < N; i += 1) {
    if (digitalRead(RESET_PIN[i]) == 0) {
      reset_motors = bitFlip(i, reset_motors);
    }
  }
  uint8_t recovered_motors = ~(non_error_motors) & reset_motors; // motors which recovered from the error, not just touched reset pin 
  if (checkflag(reset_motors)) {  // in order to avoid unncessary nothing command.
    L6470_hardstop_u(reset_motors, N);
    L6470_setparam_abspos(reset_motors, N, reset_pos);
    L6470_goto(reset_motors, N, reset_pos2);
    // fulash();
    if (checkflag(recovered_motors)) {
      L6470_resetdevice(recovered_motors, N);
      L6470_setup(recovered_motors);
      L6470_setparam_abspos(recovered_motors, N, reset_pos2);
    }
    L6470_resetdevice(reset_motors, N);
    L6470_setup(reset_motors);
    L6470_setparam_abspos(reset_motors, N, reset_pos2);
    // fulash();
    // L6470_hardhiz(reset_motors, N);
  }
  non_error_motors = non_error_motors | recovered_motors; // recover if from error recovery mode

  return reset_motors;
}

/* check error pins and change global non_error_motors flag. */
void check_error_pin() {
  // for (int i = 0; i < N; i += 1) {
  //   if (digitalRead(ERROR_PIN[i]) == 0) {
  //     non_error_motors = bitFlip2(i, non_error_motors);
  //   }
  // }
  /* あとでまるまる上に変更 デバッグ用 */
  for (int i = 0; i < 3; i += 1) {
    if (digitalRead(PSEUDO_ERROR_PIN[i]) == 0) {
      // non_error_motors = bitFlip2(i, non_error_motors);
      non_error_motors = bitFlip2(i+3, non_error_motors);
    }
  }
  uint8_t error_motion_first = ~(non_error_motors) & ~(error_motion); // motors which were not in error recovery motion
  if (checkflag(error_motion_first)) {
    error_recovery_motion(error_motion_first, error_recovery_speed);
  }
  error_motion = ~(non_error_motors);
}

/*copying array a to b, n elements. */
void array_copy(long *a, long *b, int n) {
  for (int i = 0; i < n; i += 1) {
    b[i] = a[i];
  }
}


// void fulash() {
//   L6470_getparam_abspos(param, N);
//   pos2distang(param);
//   Serial.print("ABS_POS : ");
//   for (int i = 0; i < N; i += 1) {
//     Serial.print(param[i],DEC);
//     Serial.print(",");
//   }
//   // Serial.print( pos2dist(L6470_getparam_abspos()),DEC);
//   L6470_getparam_speed(param, N);
//   Serial.print("  SPEED : ");
//   for (int i = 0; i < N; i += 1) {
//     Serial.print("0x");
//     Serial.print(param[i],HEX);
//     Serial.print(",");
//   }
//   // Serial.print( L6470_getparam_speed(),HEX);
//   Serial.print("  ERROR : ");
//   for (int i=0; i<N; i+=1) {
//     Serial.print(digitalRead(ERROR_PIN[i]));
//   }
//   Serial.print(",  RESET : ");
//   for (int i=0; i<N; i+=1) {
//     Serial.print(digitalRead(RESET_PIN[i]));
//   }
//   Serial.print(",  non_error_motors : ");
//   Serial.print(non_error_motors,BIN);
//   Serial.println();
// }