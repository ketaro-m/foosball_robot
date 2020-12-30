#include <SPI.h>
// #include <MsTimer2.h>
// #include <math.h>
#include <ros.h>
#include <sensor_msgs/Joy.h> // Joy stick
#include <std_msgs/Int8MultiArray.h> // to publish pin values
ros::NodeHandle nh;
// #include "L6470_commands.ino"
// #include "L6470_commands_multi.ino"

// number of motors
#define N 6

// define pins
#define PIN_SPI_MOSI 51
#define PIN_SPI_MISO 50
#define PIN_SPI_SCK 52
#define PIN_SPI_SS 53
#define PIN_BUSY 9
const int FLAG_PIN[N] = {2,3,4,5,6,7}; // digital pins
const int RESET_PIN[N] = {54,55,56,57,58,59}; // analog pins A0,...,A5 
int dist = 20;
int ang = 90;
boolean err_flag = false;

#define LED_PIN 10
int pin = 0;


// parameters according to this machine environment
int ROT = 200*pow(2, 7); // steps per 1 rotation
int STROKE = 40; // stroke[mm] per 1 rotation

long param[N]; // array to get and store register values such as abs_pos, speed, etc.
/* position array to control motors */
long pos[N] = {0, 0, 0, 0, 0, 0};
long pos_copy[N] = {0, 0, 0, 0, 0, 0}; // copy for motor command because distang2pos rewrites array itself

// sensor_msgs::Joy joy_msg;
void stepper_cb(const sensor_msgs::Joy& msg);
ros::Subscriber<sensor_msgs::Joy> stepper_sub("joy", stepper_cb);
std_msgs::Int8MultiArray stepper_msg;
ros::Publisher stepper_pub("stepper", &stepper_msg);


void stepper_setup() {
  stepper_msg.data_length = N*4;
  stepper_msg.data = (int8_t *)malloc(sizeof(int8_t)*N*4);
  for (int i = 0; i < N*4; i += 1) {
    stepper_msg.data[i] = 0;
  }

  nh.subscribe(stepper_sub);
  nh.advertise(stepper_pub);
}

/* publish the motors' state [abs_pos * 6, error_flag * 6, reset_pin * 6] */
void stepper_publish_loop() {
  L6470_getparam_abspos(param, N);
  pos2distang(param);
  for (int i = 0; i < N; i += 1) {
    stepper_msg.data[i] = param[i];
    stepper_msg.data[i+N] = digitalRead(FLAG_PIN[i]);
    stepper_msg.data[i+N*2] = digitalRead(RESET_PIN[i]);
  }
  stepper_pub.publish(&stepper_msg);
}

/* coefficients for motor commands from joy inputs. */
float ang_alpha = 180;
float dist_alpha = 80;
int motor_index[2] = {0, 1}; // motor now operating
int button_pre[2] = {0, 0}; // button [left, right] in the pre time to avoid double count of one press

/* subscribe joy and control stepper motors */
void stepper_cb(const sensor_msgs::Joy& msg) {

  // pin = (pin+1)%2; // 1->0,0->1
  // digitalWrite(LED_PIN, pin);

  /* change operating motors according to the LR buttons. */
  if (button_pre[0] == 0 && msg.buttons[4] == 1) {
    button_pre[0] = 1;
    motor_index_left();
  }
  if (button_pre[0] == 1 && msg.buttons[4] == 0) {
    button_pre[0] = 0;
  }
  if (button_pre[1] == 0 && msg.buttons[5] == 1) {
    button_pre[1] = 1;
    motor_index_right();
  }
  if (button_pre[1] == 1 && msg.buttons[5] == 0) {
    button_pre[1] = 0;
  }

  pos[motor_index[0]] = pos_copy[motor_index[0]] = max(min(pos[motor_index[0]] + msg.axes[1]*dist_alpha, 90), 0);
  pos[motor_index[1]] = pos_copy[motor_index[1]] = max(min(pos[motor_index[1]] + msg.axes[4]*dist_alpha, 90), 0);
  pos[motor_index[0]+3] = pos_copy[motor_index[0]+3] = max(min(pos[motor_index[0]+3] - msg.axes[0]*ang_alpha, 180), 0);
  pos[motor_index[1]+3] = pos_copy[motor_index[1]+3] = max(min(pos[motor_index[1]+3] - msg.axes[3]*ang_alpha, 180), 0); 
  for (int i = 0; i < N; i += 1) {
    stepper_msg.data[i+N*3] = pos[i];
  }
}

/* functions to change motor indexes.*/
void motor_index_left() {
  switch (motor_index[0]) {
  case 0:
    if (motor_index[1] == 2) {
      motor_index[1] = 1;
    }
    break;
  case 1:
    motor_index[0] = 0;
    break;
  default:
    break;
  }
}
void motor_index_right() {
  switch (motor_index[1]) {
  case 2:
    if (motor_index[0] == 0) {
      motor_index[0] = 1;
    }
    break;
  case 1:
    motor_index[1] = 2;
    break;
  default:
    break;
  }
}


void setup()
{
  // delay(1000);
  nh.initNode();
  stepper_setup();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 0);

  pinMode(PIN_SPI_MOSI, OUTPUT);
  pinMode(PIN_SPI_MISO, INPUT);
  pinMode(PIN_SPI_SCK, OUTPUT);
  pinMode(PIN_SPI_SS, OUTPUT);
  pinMode(PIN_BUSY, INPUT);
  for (int i=0; i<N; i+=1) {
    pinMode(FLAG_PIN[i], INPUT);
    /* use RESET PINs (analog pin) as digital pull down pins. */
    pinMode(RESET_PIN[i], OUTPUT);
    digitalWrite(RESET_PIN[i], HIGH);
  }
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  // Serial.begin(115200);
  digitalWrite(PIN_SPI_SS, HIGH);
 
  L6470_resetdevice(N); //reset all motors
  L6470_setup();
  delay(2000);
  
  // delay(5000);
  // MsTimer2::set(50, fulash);//シリアルモニター用のタイマー割り込み
  // MsTimer2::set(3000, func_timer);//overwrite用のタイマー割り込み
  // MsTimer2::start();
}

unsigned long publish_timer = 0;
unsigned long motor_timer = 0;
void loop(){
  unsigned long now = millis();

  if ((now - publish_timer) > 100) {
    stepper_publish_loop();
    publish_timer = now;
  }
  if ((now - motor_timer) > 100) {
    distang2pos(pos_copy);
    // L6470_hardstop_u(N);
    L6470_goto_u(N, pos_copy);
    memcpy(pos_copy, pos, sizeof(pos));
    motor_timer = now;
  }
  nh.spinOnce();
  delay(1);

  // boolean reset_flag = false;
  // for (int i = 0; i < N; i += 1) {
  //   if (!digitalRead[RESET_PIN[i]]) {
  //     reset_flag = true;
  //     pos[i] = 0;
  //     pos_copy[i] = 0;
  //   }
  // }
  // if (reset_flag) {
  //   distang2pos(pos_copy);
  //   L6470_setparam_abspos(N, pos_copy);
  //   memcpy(pos_copy, pos, sizeof(pos));
  // }
  

  // pin = (pin+1)%2; // 1->0,0->1
  // digitalWrite(LED_PIN, pin);

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

/* setup all motors. */
void L6470_setup(){
long acc[N] = {0x200, 0x200, 0x200, 0x300, 0x300, 0x300};
long dec[N] = {0x200, 0x200, 0x200, 0x300, 0x300, 0x300};
long maxspeed[N] = {0x100, 0x100, 0x100, 0x100, 0x100, 0x100};
long kvalhold[N] = {0x50, 0x50, 0x50, 0xc0, 0xc0, 0xc0};
long kvalrun[N] = {0x80, 0x80, 0x80, 0xd0, 0xd0, 0xd0};
long kvalacc[N] = {0x80, 0x80, 0x80, 0xd0, 0xd0, 0xd0};
long kvaldec[N] = {0x80, 0x80, 0x80, 0xd0, 0xd0, 0xd0};
L6470_setparam_acc(N, acc); //[R, WS] 加速度default 0x08A (12bit) (14.55*val+14.55[step/s^2])
L6470_setparam_dec(N, dec); //[R, WS] 減速度default 0x08A (12bit) (14.55*val+14.55[step/s^2])
L6470_setparam_maxspeed(N, maxspeed); //[R, WR]最大速度default 0x041 (10bit) (15.25*val+15.25[step/s])
L6470_setparam_minspeed(N, 0x01); //[R, WS]最小速度default 0x000 (1+12bit) (0.238*val[step/s])
L6470_setparam_fsspd(N, 0x3ff); //[R, WR]μステップからフルステップへの切替点速度default 0x027 (10bit) (15.25*val+7.63[step/s])
L6470_setparam_kvalhold(N, kvalhold); //[R, WR]停止時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_kvalrun(N, kvalrun); //[R, WR]定速回転時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_kvalacc(N, kvalacc); //[R, WR]加速時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_kvaldec(N, kvaldec); //[R, WR]減速時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_alareen(N, 0x70); // alarm enable for switch, stall detection
L6470_setparam_stallth(N, 0x10); // 脱調検知の閾値 need tuning
L6470_setparam_ocdth(N, 0xf);

L6470_setparam_stepmood(N, 0x07); //ステップモードdefault 0x07 (1+3+1+3bit) : 1/2^n*1.8[deg] が 1step 
} 
/* sample for overwrite.*/
// long pos_array[N];
// void func_timer() {
//   if (!err_flag) {
//     L6470_hardstop_u(N);
//     for (int i = 0; i < N; i += 1) {
//       if (i < 3) {
//         pos_array[i] = dist2pos(dist);
//       } else {
//         pos_array[i] = ang2pos(ang);
//       }
//     }
//     // dist2pos(N,pos_array);
//     L6470_goto_u(N, pos_array);
//     // L6470_goto_u(N, dist2pos(pos));
//     ang *= -1;
//     dist *= -1;
//     delay(100);
//   }
// }

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

// void fulash(){
//   L6470_getparam_abspos(param, N);
//   Serial.print("ABS_POS : ");
//   for (int i = 0; i < N; i += 1) {
//     Serial.print(pos2dist(param[i]),DEC);
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
//   Serial.print("  FLAG : ");
//   for (int i=0; i<N; i+=1) {
//     Serial.print(digitalRead(FLAG_PIN[i]));
//   }
//   Serial.print(",  RESET : ");
//   for (int i=0; i<N; i+=1) {
//     Serial.print(digitalRead(RESET_PIN[i]));
//   }
//   Serial.println();
// }
