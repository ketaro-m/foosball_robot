#include <SPI.h>
#include <MsTimer2.h>
#include <math.h>
#include <ros.h>
#include <sensor_msgs/Joy.h> // Joy stick
#include <std_msgs/Int8MultiArray.h> // to publish pin values
ros::NodeHandle nh;
// #include "L6470_commands.ino"
// #include "L6470_commands_multi.ino"

// number of motors
#define N 6

// define pins
#define PIN_SPI_MOSI 11
#define PIN_SPI_MISO 12
#define PIN_SPI_SCK 13
#define PIN_SPI_SS 10
#define PIN_BUSY 9
int FLAG_PIN[N] = {2,3,4,5,6,7}; // digital pins
int RESET_PIN[N] = {14,15,16,17,18,19}; // analog pins
int dist = 20;
int ang = 90;
boolean err_flag = false;


// #include <std_msgs/Int16.h>
// std_msgs::Int16 switch_msg; 
// ros::Publisher arduino_pub("arduino", &switch_msg);
// void pin_setup()
// {
//   switch_msg.data = 0;
//   nh.advertise(arduino_pub);
// }
// void pin_loop()
// {
//   switch_msg.data = digitalRead(RESET_PIN[0]);
//   arduino_pub.publish(&switch_msg);
// }

// parameters according to this machine environment
int ROT = 200*pow(2, 7); // steps per 1 rotation
int STROKE = 40; // stroke[mm] per 1 rotation

long param[N]; // array to get and store register values such as abs_pos, speed, etc.

// sensor_msgs::Joy joy_msg;
void stepper_cb(const sensor_msgs::Joy& msg);
ros::Subscriber<sensor_msgs::Joy> stepper_sub("joy", stepper_cb);
std_msgs::Int8MultiArray stepper_msg;
ros::Publisher stepper_pub("stepper", &stepper_msg);
// std_msgs::Int16MultiArray pin_msg; 
// ros::Publisher pin_pub("stepper/pin", &pin_msg);


void stepper_setup() {
  stepper_msg.data_length = N*2;
  stepper_msg.data = (int8_t *)malloc(sizeof(int8_t)*N*2);
  for (int i = 0; i < N*2; i += 1) {
    stepper_msg.data[i] = 0;
  }
  // pin_msg.data_length = N;
  // pin_msg.data = (int16_t *)malloc(sizeof(int16_t)*N);
  // for (int i = 0; i < N; i += 1) {
  //   pin_msg.data[i] = 0;
  // }

  // nh.subscribe(stepper_sub);
  nh.advertise(stepper_pub);
  // nh.advertise(pin_pub);
}


void stepper_publish_loop() {
  L6470_getparam_abspos(param, N);
  for (int i = 0; i < N; i += 1) {
    stepper_msg.data[i] = pos2ang(param[i]);
  }
  for (int i = N; i < N*2; i += 1) {
    stepper_msg.data[i] = digitalRead(FLAG_PIN[i]);
  }
  stepper_pub.publish(&stepper_msg);
  // pin_pub.publish(&pin_msg);
}

int abs_pos = 0;
/* subscribe joy and control stepper motors */
void stepper_cb(const sensor_msgs::Joy& msg) {
  long rel_pos = msg.axes[0]*90 - abs_pos;
  abs_pos = msg.axes[0]*90;
  L6470_goto(1,ang2pos(rel_pos));
}

void setup()
{
  // delay(1000);
  nh.initNode();
  stepper_setup();
  // pin_setup();

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

  // /* basic move */
  // L6470_goto(dist2pos(40*1.75));
  // L6470_busydelay(250);
  // for (int i = 0; i < 5; i += 1) {
  //   L6470_goto(dist2pos(40*0.75));
  //   // L6470_busydelay(250);
  //   L6470_goto(dist2pos(40*1.75));
  //   // L6470_busydelay(250);
  // }

  // /* basic move for multi motors*/
  // L6470_goto(N, dist2pos(40));
  // for (int i = 0; i < 10; i += 1) {
  //   long pos_array[N];
  //   for (int j = 0; j < N; j += 1) {
  //     if (j == i%N) {
  //       pos_array[j] = 70;
  //     } else {
  //       pos_array[j] = 0;
  //     }
  //   }
  //   dist2pos(N,pos_array);
  //   L6470_goto(N, pos_array);
  // }

  // /* sample for encoder reset */
  // L6470_goto(dist2pos(80));
  // L6470_busydelay(1000);
  // L6470_goto(dist2pos(40));
  // L6470_busydelay(1000);

  // /* sample for gountil */
  // L6470_setparam_mark(ang2pos(-300));
  // Serial.print("ABS_POS : ");
  // Serial.print(pos2ang(L6470_getparam_abspos()),DEC);
  // Serial.print("  MARK : ");
  // Serial.println(pos2ang(L6470_getparam_mark()),DEC);
  // L6470_gountil(0, 1, 800); // if act==1, mark->abs_pos, if act==0, abs_pos->0
  // L6470_busydelay(2000);
  // Serial.print("ABS_POS : ");
  // Serial.print(pos2ang(L6470_getparam_abspos()),DEC);
  // Serial.print("  MARK : ");
  // Serial.println(pos2ang(L6470_getparam_mark()),DEC);
  // L6470_gomark();
  


  // L6470_gohome(N);
  // L6470_hardhiz(N);
}

unsigned long state_timer = 0;
void loop(){
  // if (!digitalRead(ERROR_PIN[0])) {
  //   L6470_hardstop_u();
  //   err_flag = true;
  // } else {
  //   err_flag = false;
  // }
  unsigned long now = millis();

  if ((now - state_timer) > 100) {
    stepper_publish_loop();
    // pin_loop();
    state_timer = now;
  }
  nh.spinOnce();

  /* basic move for multi motors*/
  // L6470_goto(N, dist2pos(40));
  for (int i = 0; i < 10; i += 1) {
    long pos_array[N];
    for (int j = 0; j < N; j += 1) {
      if (j == i%N && j >= 3) {
        pos_array[j] = ang2pos(180);
      } else {
        pos_array[j] = 0;
      }
    }
    // dist2pos(N,pos_array);
    L6470_goto(N, pos_array);
  }
  L6470_goto(N, ang2pos(30));
  L6470_hardhiz(N);
  L6470_busydelay(2000);
}

/* setup all motors. */
void L6470_setup(){
long acc[N] = {0x500, 0x500, 0x500, 0x200, 0x200, 0x200};
long dec[N] = {0x500, 0x500, 0x500, 0x200, 0x200, 0x200};
long maxspeed[N] = {0x100, 0x100, 0x100, 0x60, 0x60, 0x60};
long kvalhold[N] = {0x50, 0x50, 0x50, 0xd0, 0xd0, 0xd0};
long kvalrun[N] = {0x50, 0x50, 0x50, 0xd0, 0xd0, 0xd0};
long kvalacc[N] = {0x50, 0x50, 0x50, 0xd0, 0xd0, 0xd0};
long kvaldec[N] = {0x50, 0x50, 0x50, 0xd0, 0xd0, 0xd0};
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
L6470_setparam_stallth(N, 0xf0); // 脱調検知の閾値 need tuning
L6470_setparam_ocdth(N, 0xf);

L6470_setparam_stepmood(N, 0x07); //ステップモードdefault 0x07 (1+3+1+3bit) : 1/2^n*1.8[deg] が 1step 
} 

// void func_on() {
//   noInterrupts();
//   L6470_setparam_abspos(ang2pos(0));
//   interrupts();
// }
/* sample for overwrite.*/
long pos_array[N];
void func_timer() {
  if (!err_flag) {
    L6470_hardstop_u(N);
    for (int i = 0; i < N; i += 1) {
      if (i < 3) {
        pos_array[i] = dist2pos(dist);
      } else {
        pos_array[i] = ang2pos(ang);
      }
    }
    // dist2pos(N,pos_array);
    L6470_goto_u(N, pos_array);
    // L6470_goto_u(N, dist2pos(pos));
    ang *= -1;
    dist *= -1;
    delay(100);
  }
}

/* functions to change distance[mm] or angle[degree] to steps(pos)*/
long dist2pos(long distance) {
  // if (distance > 90) {
  //   distance = 90;
  // } else if (distance < 0) {
  //   distance = 0;
  // }
  return ROT * distance / STROKE;
}
void dist2pos(int n, long *distance) {
  for (int i = 0; i < n; i += 1) {
    distance[i] = dist2pos(distance[i]);
  }
}
long ang2pos(long angle) {
  return ROT * (angle) / 360;
}
void ang2pos(int n, long *angle) {
  for (int i = 0; i < n; i += 1) {
    angle[i]  = ang2pos(angle[i]);
  }
}
long pos2dist(long position) {
  return position * STROKE / ROT;
}
void pos2dist(int n, long *position) {
  for (int i = 0; i < n; i += 1) {
    position[i] = pos2dist(position[i]);
  }
}
long pos2ang(long position) {
  return (position) * 360 / ROT;
}
void pos2ang(int n, long *position) {
  for (int i = 0; i < n; i += 1) {
    position[i] = pos2ang(position[i]);
  }
}

void fulash(){
  L6470_getparam_abspos(param, N);
  Serial.print("ABS_POS : ");
  for (int i = 0; i < N; i += 1) {
    Serial.print(pos2dist(param[i]),DEC);
    Serial.print(",");
  }
  // Serial.print( pos2dist(L6470_getparam_abspos()),DEC);
  L6470_getparam_speed(param, N);
  Serial.print("  SPEED : ");
  for (int i = 0; i < N; i += 1) {
    Serial.print("0x");
    Serial.print(param[i],HEX);
    Serial.print(",");
  }
  // Serial.print( L6470_getparam_speed(),HEX);
  Serial.print("  FLAG : ");
  for (int i=0; i<N; i+=1) {
    Serial.print(digitalRead(FLAG_PIN[i]));
  }
  Serial.print(",  RESET : ");
  for (int i=0; i<N; i+=1) {
    Serial.print(digitalRead(RESET_PIN[i]));
  }
  Serial.println();
}
