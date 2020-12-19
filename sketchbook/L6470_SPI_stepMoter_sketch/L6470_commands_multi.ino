/* Multiple motors' control commands by daisy chain connected L6470 */

void L6470_setparam_abspos(int n, long val){L6470_transfer(n,0x01,3,val);}
void L6470_setparam_elpos(int n, long val){L6470_transfer(n,0x02,2,val);}
void L6470_setparam_mark(int n, long val){L6470_transfer(n,0x03,3,val);}
void L6470_setparam_acc(int n, long val){L6470_transfer(n,0x05,2,val);}
void L6470_setparam_dec(int n, long val){L6470_transfer(n,0x06,2,val);}
void L6470_setparam_maxspeed(int n, long val){L6470_transfer(n,0x07,2,val);}
void L6470_setparam_minspeed(int n, long val){L6470_transfer(n,0x08,2,val);}
void L6470_setparam_fsspd(int n, long val){L6470_transfer(n,0x15,2,val);}
void L6470_setparam_kvalhold(int n, long val){L6470_transfer(n,0x09,1,val);}
void L6470_setparam_kvalrun(int n, long val){L6470_transfer(n,0x0a,1,val);}
void L6470_setparam_kvalacc(int n, long val){L6470_transfer(n,0x0b,1,val);}
void L6470_setparam_kvaldec(int n, long val){L6470_transfer(n,0x0c,1,val);}
void L6470_setparam_intspd(int n, long val){L6470_transfer(n,0x0d,2,val);}
void L6470_setparam_stslp(int n, long val){L6470_transfer(n,0x0e,1,val);}
void L6470_setparam_fnslpacc(int n, long val){L6470_transfer(n,0x0f,1,val);}
void L6470_setparam_fnslpdec(int n, long val){L6470_transfer(n,0x10,1,val);}
void L6470_setparam_ktherm(int n, long val){L6470_transfer(n,0x11,1,val);}
void L6470_setparam_ocdth(int n, long val){L6470_transfer(n,0x13,1,val);}
void L6470_setparam_stallth(int n, long val){L6470_transfer(n,0x14,1,val);}
void L6470_setparam_stepmood(int n, long val){L6470_transfer(n,0x16,1,val);}
void L6470_setparam_alareen(int n, long val){L6470_transfer(n,0x17,1,val);}
void L6470_setparam_config(int n, long val){L6470_transfer(n,0x18,2,val);}

// long L6470_getparam_abspos(){return L6470_getparam(0x01,3);}
// long L6470_getparam_elpos(){return L6470_getparam(0x02,2);}
// long L6470_getparam_mark(){return L6470_getparam(0x03,3);}
// long L6470_getparam_speed(){return L6470_getparam(0x04,3);}
// long L6470_getparam_acc(){return L6470_getparam(0x05,2);}
// long L6470_getparam_dec(){return L6470_getparam(0x06,2);}
// long L6470_getparam_maxspeed(){return L6470_getparam(0x07,2);}
// long L6470_getparam_minspeed(){return L6470_getparam(0x08,2);}
// long L6470_getparam_fsspd(){return L6470_getparam(0x15,2);}
// long L6470_getparam_kvalhold(){return L6470_getparam(0x09,1);}
// long L6470_getparam_kvalrun(){return L6470_getparam(0x0a,1);}
// long L6470_getparam_kvalacc(){return L6470_getparam(0x0b,1);}
// long L6470_getparam_kvaldec(){return L6470_getparam(0x0c,1);}
// long L6470_getparam_intspd(){return L6470_getparam(0x0d,2);}
// long L6470_getparam_stslp(){return L6470_getparam(0x0e,1);}
// long L6470_getparam_fnslpacc(){return L6470_getparam(0x0f,1);}
// long L6470_getparam_fnslpdec(){return L6470_getparam(0x10,1);}
// long L6470_getparam_ktherm(){return L6470_getparam(0x11,1);}
// long L6470_getparam_adcout(){return L6470_getparam(0x12,1);}
// long L6470_getparam_ocdth(){return L6470_getparam(0x13,1);}
// long L6470_getparam_stallth(){return L6470_getparam(0x14,1);}
// long L6470_getparam_stepmood(){return L6470_getparam(0x16,1);}
// long L6470_getparam_alareen(){return L6470_getparam(0x17,1);}
// long L6470_getparam_config(){return L6470_getparam(0x18,2);}
// long L6470_getparam_status(){return L6470_getparam(0x19,2);}


// void L6470_run(int dia,long spd){
//   if(dia==1)
//     L6470_transfer(0x51,3,spd);
//   else
//     L6470_transfer(0x50,3,spd);
// }
// void L6470_stepclock(int dia){
//   if(dia==1)
//     L6470_transfer(0x59,0,0);    
//   else
//     L6470_transfer(0x58,0,0);
// }
// void L6470_move(int dia,long n_step){
//   if(dia==1)
//     L6470_transfer(0x41,3,n_step);
//   else
//     L6470_transfer(0x40,3,n_step);
// }
void L6470_goto(int n, long pos){
  Serial.println("goto");
  L6470_transfer(n, 0x60,3,pos);
    //  L6470_send(n, 0x60);
    //  send3_L6470(pos, pos);
}
void L6470_goto_u(int n, long pos) {
  L6470_transfer_u(n, 0x60,3,pos);
}
// void L6470_goto2(long pos1, long pos2) {
//   L6470_transfer2(0x60,3,pos1, pos2);
// }
// void L6470_gotodia(int dia,int pos){
//   if(dia==1)    
//     L6470_transfer(0x69,3,pos);
//   else    
//     L6470_transfer(0x68,3,pos);
// }
// void L6470_gountil(int act,int dia,long spd){
//   if(act==1)
//     if(dia==1)
//       L6470_transfer(0x8b,3,spd);
//     else
//       L6470_transfer(0x8a,3,spd);
//   else
//     if(dia==1)
//       L6470_transfer(0x83,3,spd);
//     else
//       L6470_transfer(0x82,3,spd);
// }  
// void L6470_relesesw(int act,int dia){
//   if(act==1)
//     if(dia==1)
//       L6470_transfer(0x9b,0,0);
//     else
//       L6470_transfer(0x9a,0,0);
//   else
//     if(dia==1)
//       L6470_transfer(0x93,0,0);
//     else
//       L6470_transfer(0x92,0,0);
// }
void L6470_gohome(int n){
  L6470_transfer(n,0x70,0,0);
}
// void L6470_gomark(){
//   L6470_transfer(0x78,0,0);
// }
// void L6470_resetpos(){
//   L6470_transfer(0xd8,0,0);
// }
void L6470_resetdevice(int n){
  L6470_send_u(n,0x00);//nop命令
  L6470_send_u(n,0x00);
  L6470_send_u(n,0x00);
  L6470_send_u(n,0x00);
  L6470_send_u(n,0xc0);
}
// void L6470_softstop(){
//   L6470_transfer(0xb0,0,0);
// }
void L6470_hardstop(int n){
  L6470_transfer(n,0xb8,0,0);
}
void L6470_hardstop_u(int n){
  L6470_transfer_u(n,0xb8,0,0);
}
// void L6470_softhiz(){
//   L6470_transfer(0xa0,0,0);
// }
void L6470_hardhiz(int n){
  L6470_transfer(n,0xa8,0,0);
}
// long L6470_getstatus(){
//   long val=0;
//   L6470_send_u(0xd0);
//   for(int i=0;i<=1;i++){
//     val = val << 8;
//     digitalWrite(PIN_SPI_SS, LOW); // ~SSイネーブル。
//     val = val | SPI.transfer(0x00); // アドレスもしくはデータ送信。
//     digitalWrite(PIN_SPI_SS, HIGH); // ~SSディスエーブル 
//   }
//   return val;
// }

void L6470_transfer(int n, int add,int bytes,long val){
  int data[3][n];
  L6470_send(n, add);
  for (int i = 0; i < 3; i += 1) {
    for (int j = 0; j < n; j += 1) {
      data[i][j] = val & 0xff;
    }
    val = val >> 8;
  }
  if(bytes==3){
    L6470_send(n, data[2]);
  }
  if(bytes>=2){
    L6470_send(n, data[1]);
  }
  if(bytes>=1){
    L6470_send(n, data[0]);
  }  
  // for (int i = 0; i < n; i += 1) {
  //   Serial.println("data");
  //   for (int j = 0; j < 3; j += 1) {
  //     Serial.println(data[j][i]);
  //   }
  // }
}
void L6470_transfer_u(int n, int add,int bytes,long val){
  int data[3][n];
  L6470_send_u(n, add);
  for (int i = 0; i < 3; i += 1) {
    for (int j = 0; j < n; j += 1) {
      data[i][j] = val & 0xff;
    }
    val = val >> 8;
  }
  if(bytes==3){
    L6470_send_u(n, data[2]);
  }
  if(bytes>=2){
    L6470_send_u(n, data[1]);
  }
  if(bytes>=1){
    L6470_send_u(n, data[0]);
  }  
}
/* send same data to n motors. */
void L6470_send(int n, int add_or_val){
  Serial.println(add_or_val,BIN);
  while(!digitalRead(PIN_BUSY)){
  } //BESYが解除されるまで待機
  digitalWrite(PIN_SPI_SS, LOW); // ~SSイネーブル
  for (int i = 0; i < n; i += 1) {
    SPI.transfer(add_or_val); // アドレスもしくはデータ送信
  }
  digitalWrite(PIN_SPI_SS, HIGH); // ~SSディスエーブル。
}
/* send different data to n motors. */
void L6470_send(int n, int *add_or_val){
  while(!digitalRead(PIN_BUSY)){
  } //BESYが解除されるまで待機
  digitalWrite(PIN_SPI_SS, LOW); // ~SSイネーブル
  for (int i = 0; i < n; i += 1) {
    SPI.transfer(add_or_val[i]); // アドレスもしくはデータ送信
  }
  digitalWrite(PIN_SPI_SS, HIGH); // ~SSディスエーブル。
}

/* send same data to n motors. */
void L6470_send_u(int n, int add_or_val){
  digitalWrite(PIN_SPI_SS, LOW); // ~SSイネーブル
  for (int i = 0; i < n; i += 1) {
    SPI.transfer(add_or_val); // アドレスもしくはデータ送信
  }
  digitalWrite(PIN_SPI_SS, HIGH); // ~SSディスエーブル。
}
/* send different data to n motors. */
void L6470_send_u(int n, int *add_or_val){
  digitalWrite(PIN_SPI_SS, LOW); // ~SSイネーブル
  for (int i = 0; i < n; i += 1) {
    SPI.transfer(add_or_val[i]); // アドレスもしくはデータ送信
  }
  digitalWrite(PIN_SPI_SS, HIGH); // ~SSディスエーブル。
}
// void L6470_busydelay(long time){//BESYが解除されるまで待機
//   while(!digitalRead(PIN_BUSY)){
//   }
//   delay(time);
// }
// long L6470_getparam(int add,int bytes){
//   long val=0;
//   int send_add = add | 0x20;
//   L6470_send_u(send_add);
//   for(int i=0;i<=bytes-1;i++){
//     val = val << 8;
//     digitalWrite(PIN_SPI_SS, LOW); // ~SSイネーブル。
//     val = val | SPI.transfer(0x00); // アドレスもしくはデータ送信。
//     digitalWrite(PIN_SPI_SS, HIGH); // ~SSディスエーブル 
//   }
//   return val;
// }

// void send1_L6470(int8_t x , int8_t y) {
//   digitalWrite(10, LOW);
//   SPI.transfer(x);
//   SPI.transfer(y);
//   digitalWrite(10, HIGH);
// }
// void send2_L6470(int16_t x, int16_t y) {
//   int8_t buf1[2];
//   buf1[0] = x >> 8;
//   buf1[1] = x & 0xff;
//   int8_t buf2[2];
//   buf2[0] = y >> 8;
//   buf2[1] = y & 0xff;

//   digitalWrite(10, LOW);
//   SPI.transfer(buf1[0]);
//   SPI.transfer(buf2[0]);
//   digitalWrite(10, HIGH);

//   digitalWrite(10, LOW);
//   SPI.transfer(buf1[1]);
//   SPI.transfer(buf2[1]);
//   digitalWrite(10, HIGH);
// }
// void send3_L6470(int32_t x, int32_t y) {
//   int8_t buf1[3];
//   buf1[0] = x >> 16;
//   buf1[1] = x >> 8;
//   buf1[2] = x & 0xff;
//   int8_t buf2[3];
//   buf2[0] = y >> 16;
//   buf2[1] = y >> 8;
//   buf2[2] = y & 0xff;

//   digitalWrite(10, LOW);
//   SPI.transfer(buf1[0]);
//   SPI.transfer(buf2[0]);
//   digitalWrite(10, HIGH);

//   digitalWrite(10, LOW);
//   SPI.transfer(buf1[1]);
//   SPI.transfer(buf2[1]);
//   digitalWrite(10, HIGH);
  
//   digitalWrite(10, LOW);
//   SPI.transfer(buf1[2]);
//   SPI.transfer(buf2[2]);
//   digitalWrite(10, HIGH);
// }