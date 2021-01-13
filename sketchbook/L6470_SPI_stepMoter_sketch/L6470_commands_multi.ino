/* Multiple motors' control commands by daisy chain connected L6470 */

/* check the nth bit of a is 1. */
int checkBit(int n, uint8_t a) {
    return a >> n & 0b1;
}
/* change nth bit to 1*/
uint8_t bitFlip(int n, uint8_t pre_flag) {
    return pre_flag | (0b01 << n);
}
/* change nth bit to 0*/
uint8_t bitFlip2(int n, uint8_t pre_flag) {
    return pre_flag & ~(0b01 << n);
}

void L6470_setparam_abspos(int n, long val){L6470_transfer(n,0x01,3,val);}
void L6470_setparam_abspos(int n, long* val){L6470_transfer(n,0x01,3,val);}
void L6470_setparam_abspos(uint8_t m, int n, long val){L6470_transfer(m,n,0x01,3,val);}
void L6470_setparam_abspos(uint8_t m, int n, long *val){L6470_transfer(m,n,0x01,3,val);}
void L6470_setparam_elpos(int n, long val){L6470_transfer(n,0x02,2,val);}
void L6470_setparam_mark(int n, long val){L6470_transfer(n,0x03,3,val);}
void L6470_setparam_acc(int n, long val){L6470_transfer(n,0x05,2,val);}
void L6470_setparam_dec(int n, long val){L6470_transfer(n,0x06,2,val);}
void L6470_setparam_acc(int n, long* val){L6470_transfer(n,0x05,2,val);}
void L6470_setparam_dec(int n, long* val){L6470_transfer(n,0x06,2,val);}
void L6470_setparam_acc(uint8_t m, int n, long* val){L6470_transfer(m,n,0x05,2,val);}
void L6470_setparam_dec(uint8_t m, int n, long* val){L6470_transfer(m,n,0x06,2,val);}
void L6470_setparam_maxspeed(int n, long val){L6470_transfer(n,0x07,2,val);}
void L6470_setparam_minspeed(int n, long val){L6470_transfer(n,0x08,2,val);}
void L6470_setparam_maxspeed(int n, long* val){L6470_transfer(n,0x07,2,val);}
void L6470_setparam_maxspeed(uint8_t m, int n, long* val){L6470_transfer(m,n,0x07,2,val);}
void L6470_setparam_minspeed(uint8_t m, int n, long val){L6470_transfer(m,n,0x08,2,val);}
void L6470_setparam_fsspd(int n, long val){L6470_transfer(n,0x15,2,val);}
void L6470_setparam_fsspd(uint8_t m, int n, long val){L6470_transfer(m,n,0x15,2,val);}
void L6470_setparam_kvalhold(int n, long val){L6470_transfer(n,0x09,1,val);}
void L6470_setparam_kvalrun(int n, long val){L6470_transfer(n,0x0a,1,val);}
void L6470_setparam_kvalacc(int n, long val){L6470_transfer(n,0x0b,1,val);}
void L6470_setparam_kvaldec(int n, long val){L6470_transfer(n,0x0c,1,val);}
void L6470_setparam_kvalhold(int n, long* val){L6470_transfer(n,0x09,1,val);}
void L6470_setparam_kvalrun(int n, long* val){L6470_transfer(n,0x0a,1,val);}
void L6470_setparam_kvalacc(int n, long* val){L6470_transfer(n,0x0b,1,val);}
void L6470_setparam_kvaldec(int n, long* val){L6470_transfer(n,0x0c,1,val);}
void L6470_setparam_kvalhold(uint8_t m, int n, long* val){L6470_transfer(m,n,0x09,1,val);}
void L6470_setparam_kvalrun(uint8_t m, int n, long* val){L6470_transfer(m,n,0x0a,1,val);}
void L6470_setparam_kvalacc(uint8_t m, int n, long* val){L6470_transfer(m,n,0x0b,1,val);}
void L6470_setparam_kvaldec(uint8_t m, int n, long* val){L6470_transfer(m,n,0x0c,1,val);}
void L6470_setparam_intspd(int n, long val){L6470_transfer(n,0x0d,2,val);}
void L6470_setparam_stslp(int n, long val){L6470_transfer(n,0x0e,1,val);}
void L6470_setparam_fnslpacc(int n, long val){L6470_transfer(n,0x0f,1,val);}
void L6470_setparam_fnslpdec(int n, long val){L6470_transfer(n,0x10,1,val);}
void L6470_setparam_ktherm(int n, long val){L6470_transfer(n,0x11,1,val);}
void L6470_setparam_ocdth(int n, long val){L6470_transfer(n,0x13,1,val);}
void L6470_setparam_ocdth(uint8_t m, int n, long val){L6470_transfer(m,n,0x13,1,val);}
void L6470_setparam_stallth(int n, long val){L6470_transfer(n,0x14,1,val);}
void L6470_setparam_stallth(uint8_t m, int n, long val){L6470_transfer(m,n,0x14,1,val);}
void L6470_setparam_stepmood(int n, long val){L6470_transfer(n,0x16,1,val);}
void L6470_setparam_stepmood(uint8_t m, int n, long val){L6470_transfer(m,n,0x16,1,val);}
void L6470_setparam_alareen(int n, long val){L6470_transfer(n,0x17,1,val);}
void L6470_setparam_alareen(uint8_t m, int n, long val){L6470_transfer(m,n,0x17,1,val);}
void L6470_setparam_config(int n, long val){L6470_transfer(n,0x18,2,val);}

void L6470_getparam_abspos(long *val, int n){return L6470_getparam(val,n,0x01,3);}
void L6470_getparam_elpos(long *val, int n){return L6470_getparam(val,n,0x02,2);}
void L6470_getparam_mark(long *val, int n){return L6470_getparam(val,n,0x03,3);}
void L6470_getparam_speed(long *val, int n){return L6470_getparam(val,n,0x04,3);}
void L6470_getparam_acc(long *val, int n){return L6470_getparam(val,n,0x05,2);}
void L6470_getparam_dec(long *val, int n){return L6470_getparam(val,n,0x06,2);}
void L6470_getparam_maxspeed(long *val, int n){return L6470_getparam(val,n,0x07,2);}
void L6470_getparam_minspeed(long *val, int n){return L6470_getparam(val,n,0x08,2);}
void L6470_getparam_fsspd(long *val, int n){return L6470_getparam(val,n,0x15,2);}
void L6470_getparam_kvalhold(long *val, int n){return L6470_getparam(val,n,0x09,1);}
void L6470_getparam_kvalrun(long *val, int n){return L6470_getparam(val,n,0x0a,1);}
void L6470_getparam_kvalacc(long *val, int n){return L6470_getparam(val,n,0x0b,1);}
void L6470_getparam_kvaldec(long *val, int n){return L6470_getparam(val,n,0x0c,1);}
void L6470_getparam_intspd(long *val, int n){return L6470_getparam(val,n,0x0d,2);}
void L6470_getparam_stslp(long *val, int n){return L6470_getparam(val,n,0x0e,1);}
void L6470_getparam_fnslpacc(long *val, int n){return L6470_getparam(val,n,0x0f,1);}
void L6470_getparam_fnslpdec(long *val, int n){return L6470_getparam(val,n,0x10,1);}
void L6470_getparam_ktherm(long *val, int n){return L6470_getparam(val,n,0x11,1);}
void L6470_getparam_adcout(long *val, int n){return L6470_getparam(val,n,0x12,1);}
void L6470_getparam_ocdth(long *val, int n){return L6470_getparam(val,n,0x13,1);}
void L6470_getparam_stallth(long *val, int n){return L6470_getparam(val,n,0x14,1);}
void L6470_getparam_stepmood(long *val, int n){return L6470_getparam(val,n,0x16,1);}
void L6470_getparam_alareen(long *val, int n){return L6470_getparam(val,n,0x17,1);}
void L6470_getparam_config(long *val, int n){return L6470_getparam(val,n,0x18,2);}
void L6470_getparam_status(long *val, int n){return L6470_getparam(val,n,0x19,2);}


void L6470_run(int n,int dia,long spd){
  if(dia==1)
    L6470_transfer(n,0x51,3,spd);
  else
    L6470_transfer(n,0x50,3,spd);
}
/* run mth motors*/
void L6470_run_u(uint8_t m, int n, int dia, long spd){
  if(dia==1){
    L6470_transfer_u(m,n,0x51,3,spd);
  } else {
    L6470_transfer_u(m,n,0x50,3,spd);
  }
}
/* run mth motors*/
void L6470_run_u(uint8_t m, int n, int dia, long *spd){
  if(dia==1){
    L6470_transfer_u(m,n,0x51,3,spd);
  } else {
    L6470_transfer_u(m,n,0x50,3,spd);
  }
}
void L6470_stepclock(int n,int dia){
  if(dia==1)
    L6470_transfer(n,0x59,0,(long)0);    
  else
    L6470_transfer(n,0x58,0,(long)0);
}
void L6470_move(int n,int dia,long n_step){
  if(dia==1)
    L6470_transfer(n,0x41,3,n_step);
  else
    L6470_transfer(n,0x40,3,n_step);
}
void L6470_goto(int n, long pos){
  L6470_transfer(n, 0x60,3,pos);
}
void L6470_goto(uint8_t m, int n, long pos){
  L6470_transfer(m, n, 0x60,3,pos);
}
void L6470_goto(int n, long *pos){
  L6470_transfer(n, 0x60,3,pos);
}
void L6470_goto(uint8_t m, int n, long *pos){
  L6470_transfer(m, n, 0x60,3,pos);
}
void L6470_goto_u(int n, long pos) {
  L6470_transfer_u(n, 0x60,3,pos);
}
void L6470_goto_u(uint8_t m, int n, long pos){
  L6470_transfer_u(m, n, 0x60,3,pos);
}
void L6470_goto_u(int n, long *pos) {
  L6470_transfer_u(n, 0x60,3,pos);
}
void L6470_goto_u(uint8_t m, int n, long *pos) {
  L6470_transfer_u(m, n, 0x60,3,pos);
}
void L6470_gotodia(int n,int dia,int pos){
  if(dia==1)    
    L6470_transfer(n,0x69,3,pos);
  else    
    L6470_transfer(n,0x68,3,pos);
}
void L6470_gountil(int n,int act,int dia,long spd){
  if(act==1)
    if(dia==1)
      L6470_transfer(n,0x8b,3,spd);
    else
      L6470_transfer(n,0x8a,3,spd);
  else
    if(dia==1)
      L6470_transfer(n,0x83,3,spd);
    else
      L6470_transfer(n,0x82,3,spd);
}  
void L6470_relesesw(int n,int act,int dia){
  if(act==1)
    if(dia==1)
      L6470_transfer(n,0x9b,0,(long)0);
    else
      L6470_transfer(n,0x9a,0,(long)0);
  else
    if(dia==1)
      L6470_transfer(n,0x93,0,(long)0);
    else
      L6470_transfer(n,0x92,0,(long)0);
}
void L6470_gohome(int n){
  L6470_transfer(n,0x70,0,(long)0);
}
void L6470_gomark(int n){
  L6470_transfer(n,0x78,0,(long)0);
}
void L6470_resetpos(int n){
  L6470_transfer(n,0xd8,0,(long)0);
}
void L6470_resetpos(uint8_t m, int n){
  L6470_transfer(m,n,0xd8,0,(long)0);
}
void L6470_resetdevice(int n){
  L6470_send(n,0x00);//nop命令
  L6470_send(n,0x00);
  L6470_send(n,0x00);
  L6470_send(n,0x00);
  L6470_send(n,0xc0);
}
void L6470_resetdevice_u(int n){
  L6470_send_u(n,0x00);//nop命令
  L6470_send_u(n,0x00);
  L6470_send_u(n,0x00);
  L6470_send_u(n,0x00);
  L6470_send_u(n,0xc0);
}
void L6470_resetdevice(uint8_t m, int n){
  L6470_send(n,0x00);
  L6470_send(n,0x00);
  L6470_send(n,0x00);
  L6470_send(n,0x00);
  L6470_send(m,n,0xc0);
}
void L6470_resetdevice_u(uint8_t m, int n){
  L6470_send_u(n,0x00);
  L6470_send_u(n,0x00);
  L6470_send_u(n,0x00);
  L6470_send_u(n,0x00);
  L6470_send_u(m,n,0xc0);
}
void L6470_softstop(int n){
  L6470_transfer(n,0xb0,0,(long)0);
}
void L6470_hardstop(int n){
  L6470_transfer(n,0xb8,0,(long)0);
}
void L6470_hardstop_u(int n){
  L6470_transfer_u(n,0xb8,0,(long)0);
}
/* hardstop mth motors in n motors.*/
void L6470_hardstop_u(uint8_t m, int n){
  L6470_transfer_u(m,n,0xb8,0,(long)0);
}
void L6470_softhiz(int n){
  L6470_transfer(n,0xa0,0,(long)0);
}
void L6470_hardhiz(int n){
  L6470_transfer(n,0xa8,0,(long)0);
}
void L6470_hardhiz(uint8_t m, int n){
  L6470_transfer(m,n,0xa8,0,(long)0);
}
void L6470_getstatus(long *val, int n){
  L6470_send_u(n,0xd0);
  for(int i=0;i<=1;i++){
    digitalWrite(PIN_SPI_SS, LOW);
    for (int j = 0; j < n; j += 1) {
      val[j] = val[j] << 8;
      val[j] = val[j] | SPI.transfer(0x00);
    }
    digitalWrite(PIN_SPI_SS, HIGH);
  }
}



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
}
void L6470_transfer(uint8_t m, int n, int add,int bytes,long val){
  int data[3][n];
  L6470_send(m, n, add);
  for (int i = 0; i < 3; i += 1) {
    for (int j = 0; j < n; j += 1) {
      data[i][j] = val & 0xff;
    }
    val = val >> 8;
  }
  if(bytes==3){
    L6470_send(m, n, data[2]);
  }
  if(bytes>=2){
    L6470_send(m, n, data[1]);
  }
  if(bytes>=1){
    L6470_send(m, n, data[0]);
  }  
}
void L6470_transfer(int n, int add,int bytes,long *val){
  long val_copy[n];// copy not to destruct *val
  for (int i = 0; i < n; i += 1) {val_copy[i] = val[i];}
  int data[3][n];
  L6470_send(n, add);
  for (int i = 0; i < 3; i += 1) {
    for (int j = 0; j < n; j += 1) {
      data[i][j] = val_copy[j] & 0xff;
      val_copy[j] = val_copy[j] >> 8;
    }
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
}
/* send data to mth motors (m=ob001010 or something) and 0x00 to others in n motors. */
void L6470_transfer(uint8_t m, int n, int add,int bytes,long *val){
  long val_copy[n];// copy not to destruct *val
  for (int i = 0; i < n; i += 1) {val_copy[i] = val[i];}
  int data[3][n];
  L6470_send(m, n, add);
  for (int i = 0; i < 3; i += 1) {
    for (int j = 0; j < n; j += 1) {
      data[i][j] = val_copy[j] & 0xff;
      val_copy[j] = val_copy[j] >> 8;
    }
  }
  if(bytes==3){
    L6470_send(m, n, data[2]);
  }
  if(bytes>=2){
    L6470_send(m ,n, data[1]);
  }
  if(bytes>=1){
    L6470_send(m, n, data[0]);
  }  
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
void L6470_transfer_u(uint8_t m, int n, int add,int bytes,long val){
  int data[3][n];
  L6470_send_u(m, n, add);
  for (int i = 0; i < 3; i += 1) {
    for (int j = 0; j < n; j += 1) {
      data[i][j] = val & 0xff;
    }
    val = val >> 8;
  }
  if(bytes==3){
    L6470_send_u(m, n, data[2]);
  }
  if(bytes>=2){
    L6470_send_u(m, n, data[1]);
  }
  if(bytes>=1){
    L6470_send_u(m, n, data[0]);
  }  
}
void L6470_transfer_u(int n, int add,int bytes,long *val){
  long val_copy[n];// copy not to destruct *val
  for (int i = 0; i < n; i += 1) {val_copy[i] = val[i];}
  int data[3][n];
  L6470_send_u(n, add);
  for (int i = 0; i < 3; i += 1) {
    for (int j = 0; j < n; j += 1) {
      data[i][j] = val_copy[j] & 0xff;
      val_copy[j] = val_copy[j] >> 8;
    }
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
/* send data to mth motors (m=ob001010 or something) and 0x00 to others in n motors. */
void L6470_transfer_u(uint8_t m, int n, int add,int bytes,long *val){
  long val_copy[n];// copy not to destruct *val
  for (int i = 0; i < n; i += 1) {val_copy[i] = val[i];}
  int data[3][n];
  L6470_send_u(m, n, add);
  for (int i = 0; i < 3; i += 1) {
    for (int j = 0; j < n; j += 1) {
      data[i][j] = val_copy[j] & 0xff;
      val_copy[j] = val_copy[j] >> 8;
    }
  }
  if(bytes==3){
    L6470_send_u(m, n, data[2]);
  }
  if(bytes>=2){
    L6470_send_u(m ,n, data[1]);
  }
  if(bytes>=1){
    L6470_send_u(m, n, data[0]);
  }  
}

/* send same data to n motors. */
void L6470_send(int n, int add_or_val){
  while(!digitalRead(PIN_BUSY)){
  }
  digitalWrite(PIN_SPI_SS, LOW);
  for (int i = 0; i < n; i += 1) {
    SPI.transfer(add_or_val);
  }
  digitalWrite(PIN_SPI_SS, HIGH);
}
/* send same data to mth(bit flag) motors in n motors. */
void L6470_send(uint8_t m, int n, int add_or_val){
  while(!digitalRead(PIN_BUSY)){
  }
  digitalWrite(PIN_SPI_SS, LOW);
  for (int i = 0; i < n; i += 1) {
    if (checkBit(i, m) == 1) {
      SPI.transfer(add_or_val);
    } else {
      SPI.transfer(0x00);
    }
  }
  digitalWrite(PIN_SPI_SS, HIGH);
}
/* send different data to n motors. */
void L6470_send(int n, int *add_or_val){
  while(!digitalRead(PIN_BUSY)){
  }
  digitalWrite(PIN_SPI_SS, LOW);
  for (int i = 0; i < n; i += 1) {
    SPI.transfer(add_or_val[i]);
  }
  digitalWrite(PIN_SPI_SS, HIGH);
}
/* send different data to mth(bit flag) motors in n motors. */
void L6470_send(uint8_t m, int n, int *add_or_val){
  while(!digitalRead(PIN_BUSY)){
  }
  digitalWrite(PIN_SPI_SS, LOW);
  for (int i = 0; i < n; i += 1) {
    if (checkBit(i, m) == 1) {
      SPI.transfer(add_or_val[i]);
    } else {
      SPI.transfer(0x00);
    }
  }
  digitalWrite(PIN_SPI_SS, HIGH);
}
/* send same data to n motors. */
void L6470_send_u(int n, int add_or_val){
  digitalWrite(PIN_SPI_SS, LOW);
  for (int i = 0; i < n; i += 1) {
    SPI.transfer(add_or_val);
  }
  digitalWrite(PIN_SPI_SS, HIGH);
}
/* send same data to mth(bit flag) motors in n motors. */
void L6470_send_u(uint8_t m, int n, int add_or_val){
  digitalWrite(PIN_SPI_SS, LOW);
  for (int i = 0; i < n; i += 1) {
    if (checkBit(i, m) == 1) {
      SPI.transfer(add_or_val);
    } else {
      SPI.transfer(0x00);
    }
  }
  digitalWrite(PIN_SPI_SS, HIGH);
}
/* send different data to n motors. */
void L6470_send_u(int n, int *add_or_val){
  digitalWrite(PIN_SPI_SS, LOW);
  for (int i = 0; i < n; i += 1) {
    SPI.transfer(add_or_val[i]);
  }
  digitalWrite(PIN_SPI_SS, HIGH);
}
/* send different data to mth(bit flag) motors in n motors. */
void L6470_send_u(uint8_t m, int n, int *add_or_val){
  digitalWrite(PIN_SPI_SS, LOW);
  for (int i = 0; i < n; i += 1) {
    if (checkBit(i, m) == 1) {
      SPI.transfer(add_or_val[i]);
    } else {
      SPI.transfer(0x00);
    }
  }
  digitalWrite(PIN_SPI_SS, HIGH);
}
void L6470_getparam(long *val, int n,int add,int bytes){
  for (int i = 0; i < n; i += 1) {
    val[i] = 0;
  }
  int send_add = add | 0x20;
  L6470_send_u(n,send_add);
  for(int i=0;i<=bytes-1;i++){
    digitalWrite(PIN_SPI_SS, LOW);
    for (int j = 0; j < n; j += 1) {
      val[j] = val[j] << 8;
      val[j] = val[j] | SPI.transfer(0x00);
    }
      digitalWrite(PIN_SPI_SS, HIGH); 
  }
}