#include <SPI.h>
#include <MsTimer2.h>
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
int pos = -30;
boolean err_flag = false;

// parameters according to this machine environment
long ROT = 200*pow(2, 7); // steps per 1 rotation
long STROKE = 40; // stroke[mm] per 1 rotation

long param[N]; // array to get and store register values such as abs_pos, speed, etc.

void setup()
{
  delay(1000);
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
  Serial.begin(115200);
  digitalWrite(PIN_SPI_SS, HIGH);
 
  L6470_resetdevice(N); //reset all motors
  L6470_setup();
  delay(2000);
  
  MsTimer2::set(50, fulash);//シリアルモニター用のタイマー割り込み
  // MsTimer2::set(500, func_timer);//overwrite用のタイマー割り込み
  MsTimer2::start();
  delay(2000);

  // /* basic move */
  // L6470_goto(dist2pos(40*1.75));
  // L6470_busydelay(250);
  // for (int i = 0; i < 5; i += 1) {
  //   L6470_goto(dist2pos(40*0.75));
  //   // L6470_busydelay(250);
  //   L6470_goto(dist2pos(40*1.75));
  //   // L6470_busydelay(250);
  // }

  /* basic move for multi motors*/
  L6470_goto(N, dist2pos(40));
  for (int i = 0; i < 10; i += 1) {
    long pos_array[N];
    for (int j = 0; j < N; j += 1) {
      if (j == i%N) {
        pos_array[j] = 70;
      } else {
        pos_array[j] = 0;
      }
    }
    dist2pos(N,pos_array);
    L6470_goto(N, pos_array);
  }

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
  


  L6470_gohome(N);
  L6470_hardhiz(N);
}

void loop(){
  // if (!digitalRead(ERROR_PIN[0])) {
  //   L6470_hardstop_u();
  //   err_flag = true;
  // } else {
  //   err_flag = false;
  // }
}

/* setup all motors. */
void L6470_setup(){
L6470_setparam_acc(N, 0x50); //[R, WS] 加速度default 0x08A (12bit) (14.55*val+14.55[step/s^2])
L6470_setparam_dec(N, 0x50); //[R, WS] 減速度default 0x08A (12bit) (14.55*val+14.55[step/s^2])
L6470_setparam_maxspeed(N, 0x60); //[R, WR]最大速度default 0x041 (10bit) (15.25*val+15.25[step/s])
L6470_setparam_minspeed(N, 0x01); //[R, WS]最小速度default 0x000 (1+12bit) (0.238*val[step/s])
L6470_setparam_fsspd(N, 0x3ff); //[R, WR]μステップからフルステップへの切替点速度default 0x027 (10bit) (15.25*val+7.63[step/s])
L6470_setparam_kvalhold(N, 0x50); //[R, WR]停止時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_kvalrun(N, 0x50); //[R, WR]定速回転時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_kvalacc(N, 0x50); //[R, WR]加速時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_kvaldec(N, 0x50); //[R, WR]減速時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_alareen(N, 0x70); // alarm enable for switch, stall detection
L6470_setparam_stallth(N, 0x30); // 脱調検知の閾値 need tuning

L6470_setparam_stepmood(N, 0x07); //ステップモードdefault 0x07 (1+3+1+3bit) : 1/2^n*1.8[deg] が 1step 
} 

// void func_on() {
//   noInterrupts();
//   L6470_setparam_abspos(ang2pos(0));
//   interrupts();
// }
/* sample for overwrite.*/
void func_timer() {
  if (!err_flag) {
    L6470_hardstop_u(N);
    L6470_goto_u(N, dist2pos(pos));
    pos *= -1;
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
