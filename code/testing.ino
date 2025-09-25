// ESP32 (M5 Atom Lite) -> Pololu 3pi (3pi-serial-slave firmware)
// Line-following met duidelijke "stil tot lijn" logica:
// - Blijft STIL tot lijn sterk gedetecteerd is (debounced) -> "LINE FOUND" + start PID
// - Bij lijn kwijt -> "LINE LOST" + PID stop + motors 0 (blijft stil)
// - Trage/smooth PID (lage P/D & lagere maxSpeed)
//
// Referenties (commands & ranges):
// - Serial slave cmds: BA (autocal), BB (start PID: [max,Pn,Pd,Dn,Dd]), BC (stop), B6 (line pos 0..4000), 87 (calibrated 0..1000), C1/C2/C5/C6 (motoren). 
//   Pololu 3pi User’s Guide sec. 10.a/7.c. 

#include <Arduino.h>

static const int RX_PIN   = 32;       // ESP32 RX  (from 3pi TX)
static const int TX_PIN   = 26;       // ESP32 TX  (to 3pi RX)
static const long UART_BAUD = 115200; // 115200 8N1 (serial slave)
static const long USB_BAUD  = 115200; // USB debug

HardwareSerial RobotSerial(1);

// ---------- utils ----------
static inline uint8_t spd127(int v){
  v = constrain(v, -255, 255);
  return (uint8_t)(abs(v) * 127 / 255);
}
void setMotors_3pi(int left, int right){
  uint8_t buf[4];
  buf[0] = (left  >= 0) ? 0xC1 : 0xC2; buf[1] = spd127(left);
  buf[2] = (right >= 0) ? 0xC5 : 0xC6; buf[3] = spd127(right);
  RobotSerial.write(buf,2); RobotSerial.write(buf+2,2);
}
void sendCmd(uint8_t c){ RobotSerial.write(&c,1); }
size_t readBytesExact(uint8_t* dst, size_t n, uint32_t to=400){
  uint32_t t0=millis(); size_t i=0;
  while(i<n && (millis()-t0)<to){ int c=RobotSerial.read(); if(c>=0) dst[i++]=(uint8_t)c; }
  return i;
}
static inline uint16_t rd16LE(uint8_t lo, uint8_t hi){ return (uint16_t)lo | ((uint16_t)hi<<8); }

// ---------- 3pi helpers (per Pololu 3pi Serial Slave) ----------
bool readSignature(char out[7]){
  sendCmd(0x81); uint8_t b[6];
  if(readBytesExact(b,6,200)==6){ memcpy(out,b,6); out[6]=0; return true; }
  return false;
}
bool resetCalibration(){ sendCmd(0xB5); return true; }
bool autoCalibrate(uint32_t timeout_ms=7000){
  sendCmd(0xBA);
  uint32_t t0=millis();
  while(millis()-t0<timeout_ms){
    int c=RobotSerial.read();
    if(c=='c'){ return true; }      // klaar
  }
  return false;
}
// 5× calibrated 0..1000 (10 bytes LE)
bool readCalibrated(uint16_t out5[5]){
  sendCmd(0x87);
  uint8_t b[10]; if(readBytesExact(b,10,200)!=10) return false;
  for(int i=0;i<5;i++) out5[i]=rd16LE(b[2*i], b[2*i+1]);
  return true;
}
// 0..4000 (0=links, 2000=center, 4000=rechts)
int readLinePosition(){
  sendCmd(0xB6);
  uint8_t b[2]; if(readBytesExact(b,2,200)!=2) return -1;
  return (int)rd16LE(b[0], b[1]);
}
bool startPID(uint8_t maxSpeed, uint8_t Pn,uint8_t Pd, uint8_t Dn,uint8_t Dd){
  uint8_t pkt[6]={0xBB,maxSpeed,Pn,Pd,Dn,Dd}; RobotSerial.write(pkt,6); return true;
}
void stopPID(){ sendCmd(0xBC); }

// ---------- lijn-detectie (debounced) ----------
struct LineGate {
  // thresholds: “sterke” lijn = hoog contrast in calibrated (0..1000)
  // tweakbaar: mx>=700 & mn<=150; plus pos binnen safe zone
  uint16_t minOk = 150;
  uint16_t maxOk = 700;
  int posMin = 200;    // vermijd uiterste randen
  int posMax = 3800;

  // debounce: vereis N hits binnen M checks
  static const int W = 6;   // window
  static const int NEED = 4;
  bool hist[W] = {false,false,false,false,false,false};
  int idx = 0;

  bool senseOnce(){
    uint16_t s[5];
    if(!readCalibrated(s)) return false;
    uint16_t mn=1000, mx=0;
    for(int i=0;i<5;i++){ mn=min(mn,s[i]); mx=max(mx,s[i]); }

    int pos = readLinePosition();   // mag -1 zijn (timeout)
    bool validPos = (pos>=posMin && pos<=posMax);

    bool strong = (mx>=maxOk && mn<=minOk) && validPos;
    // Debug print compact:
    Serial.printf("sense mn=%u mx=%u pos=%d -> %s\n", mn, mx, pos, strong?"STRONG":"weak");
    return strong;
  }

  bool update(){
    bool strong = senseOnce();
    hist[idx] = strong;
    idx = (idx+1)%W;
    int hits=0; for(int i=0;i<W;i++) if(hist[i]) hits++;
    return hits>=NEED;
  }

  void clear(){ for(int i=0;i<W;i++) hist[i]=false; idx=0; }
};

// ---------- state ----------
enum class Mode{ SEARCHING, FOLLOWING };
Mode mode = Mode::SEARCHING;
LineGate gate;
uint32_t lastLog=0;

// PID settings — rustig
uint8_t MAXSPD = 65;     // traag/veilig (0..127)
uint8_t Pn=1, Pd=18;     // ~0.055
uint8_t Dn=1, Dd=72;     // ~0.014

void goIdle(){
  stopPID();
  setMotors_3pi(0,0);
  mode = Mode::SEARCHING;
}

void goFollow(){
  startPID(MAXSPD, Pn, Pd, Dn, Dd);
  mode = Mode::FOLLOWING;
}

// ---------- setup ----------
void setup(){
  Serial.begin(USB_BAUD);
  delay(150);
  RobotSerial.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("\n[3pi line follow: strict gating & stop-on-miss]");
  char sig[7]; if(readSignature(sig)) Serial.printf("3pi sig: %s\n", sig);

  resetCalibration();
  Serial.println("Autocalibrating...");
  bool ok = autoCalibrate();
  Serial.println(ok? "Autocal OK" : "Autocal TIMEOUT");

  // start in SEARCHING (stil) — geen PID tot lijn echt gedetecteerd
  goIdle();
  Serial.println("STATE: SEARCHING (motors OFF). Wacht op duidelijke lijn...");
}

// ---------- loop ----------
void loop(){
  // Check elke ~120ms de lijnstatus
  if(millis()-lastLog >= 120){
    lastLog = millis();

    if(mode == Mode::SEARCHING){
      // stil blijven & checken
      bool haveLine = gate.update();
      if(haveLine){
        Serial.println("*** LINE FOUND -> start PID ***");
        gate.clear();
        goFollow();
      }
    } else if(mode == Mode::FOLLOWING){
      // bewaken: als opeenvolgende zwakke detecties -> line lost
      bool stillGood = gate.update();  // gebruikt dezelfde gate (hit-count)
      if(!stillGood){
        // omgekeerde interpretatie: als te weinig STRONG in window -> los
        // (we checken extra met pos voor zekerheid)
        int pos = readLinePosition();
        bool lost = (pos<=50 || pos>=3950 || pos<0);
        if(lost){
          Serial.println("*** LINE LOST -> STOP & wait ***");
          gate.clear();
          goIdle();   // PID uit + motors 0
        }
      }
    }
  }

  // seriële passthrough eventueel
  while(RobotSerial.available()) Serial.write(RobotSerial.read());

  // optionele USB commands
  while(Serial.available()){
    int ch=Serial.read();
    if(ch=='r'){                 // recalibrate
      Serial.println("Recalibrating...");
      goIdle();
      resetCalibration();
      if(autoCalibrate()) Serial.println("Autocal OK");
      gate.clear();
      Serial.println("STATE: SEARCHING. Wacht op lijn...");
    } else if(ch=='s'){          // force stop
      Serial.println("STOP (user).");
      goIdle();
    }
  }
}