#include <SPI.h>
#include <MsTimer2.h>

// ピン定義。
#define PIN_SPI_MOSI 11
#define PIN_SPI_MISO 12
#define PIN_SPI_SCK 13
#define PIN_SPI_SS 10
#define PIN_BUSY 9
#define FLAG_PIN 4
int pos = 80;

long stroke = 200*pow(2, 7); // steps per stroke

void setup()
{
  delay(1000);
  pinMode(PIN_SPI_MOSI, OUTPUT);
  pinMode(PIN_SPI_MISO, INPUT);
  pinMode(PIN_SPI_SCK, OUTPUT);
  pinMode(PIN_SPI_SS, OUTPUT);
  pinMode(PIN_BUSY, INPUT);
  pinMode(FLAG_PIN, INPUT);
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  Serial.begin(9600);
  digitalWrite(PIN_SPI_SS, HIGH);
 
  L6470_resetdevice(); //L6470リセット
  L6470_setup();  //L6470を設定
  delay(2000);
  
  // MsTimer2::set(50, fulash);//シリアルモニター用のタイマー割り込み
  MsTimer2::set(500, func_timer);//overwrite用のタイマー割り込み
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
  // delay(250);

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
  


  // L6470_gohome();
  // L6470_hardhiz();
}

void loop(){
}

void L6470_setup(){
// スライダは1回転 40mm
L6470_setparam_acc(0x300); //[R, WS] 加速度default 0x08A (12bit) (14.55*val+14.55[step/s^2])
L6470_setparam_dec(0x300); //[R, WS] 減速度default 0x08A (12bit) (14.55*val+14.55[step/s^2])
L6470_setparam_maxspeed(0x60); //[R, WR]最大速度default 0x041 (10bit) (15.25*val+15.25[step/s])
L6470_setparam_minspeed(0x01); //[R, WS]最小速度default 0x000 (1+12bit) (0.238*val[step/s])
L6470_setparam_fsspd(0x3ff); //[R, WR]μステップからフルステップへの切替点速度default 0x027 (10bit) (15.25*val+7.63[step/s])
L6470_setparam_kvalhold(0x50); //[R, WR]停止時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_kvalrun(0x50); //[R, WR]定速回転時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_kvalacc(0x50); //[R, WR]加速時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_kvaldec(0x50); //[R, WR]減速時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_alareen(0x70); // alarm enable for switch, stall detection
L6470_setparam_stallth(0x30); // 脱調検知の閾値 need tuning

L6470_setparam_stepmood(0x07); //ステップモードdefault 0x07 (1+3+1+3bit) : 1/2^n*1.8[deg] が 1step 
}

// void func_on() {
//   noInterrupts();
//   L6470_setparam_abspos(ang2pos(0));
//   interrupts();
// }
/* sample for overwrite.*/
void func_timer() {
  L6470_hardstop_u();
  L6470_goto_u(dist2pos(pos));
  pos *= -1;
}

/* functions to change distance[mm] or angle[degree] to steps(pos)*/
long dist2pos(long distance) {
  if (distance > 90) {
    distance = 90;
  } else if (distance < 0) {
    distance = 0;
  }
  return stroke * distance / 40;
}
long ang2pos(long angle) {
  return stroke * (-angle) / 360;
}
long pos2dist(long position) {
  return position * 40 / stroke;
}
long pos2ang(long position) {
  return (-position) * 360 / stroke;
}

void fulash(){
  Serial.print("ABS_POS : ");
  Serial.print( pos2dist(L6470_getparam_abspos()),DEC);
  Serial.print(",  SPEED : ");
  Serial.print("0x");
  Serial.print( L6470_getparam_speed(),HEX);
  Serial.print(",  FLAG : ");
  Serial.println(digitalRead(FLAG_PIN));
}
