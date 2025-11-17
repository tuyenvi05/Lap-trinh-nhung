/*******************************************************
 * 🚗 AlphaBot (Waveshare) – Dò line PID (Vào cua chậm)
 *******************************************************/
#include <Arduino.h>
#include "TRSensors.h"

#define SO_CAM_BIEN 5
TRSensors trs;
unsigned int giaTriCamBien[SO_CAM_BIEN];

// ==== Động cơ ====
const int IN1 = A0, IN2 = A1, ENA = 5;
const int IN3 = A2, IN4 = A3, ENB = 6;

// ==== PID cơ bản ====
float Kp_base = 0.045;
float Ki = 0.0005;
float Kd_base = 0.0045;

int tocDoCoBan = 80;
int tocDoToiDa = 120;
int tocDoToiThieu = 0;

long saiSoCu = 0;
float tongSaiSo = 0;
unsigned long thoiGianCu = 0;
unsigned long lanInCu = 0;

// ==== Độ nhạy cảm biến ====
float heSoNhanBietCamBien[SO_CAM_BIEN] = {0.25,0.25,0.15,0.25,0.25};

// ==== Cờ trạng thái bắt lại line ====
bool daBatLaiLine = false;

// ==== Điều khiển động cơ ====
void setDongCoTrai(int tocDo){
  tocDo = constrain(tocDo,-255,255);
  digitalWrite(IN1, tocDo>=0?LOW:HIGH);
  digitalWrite(IN2, tocDo>=0?HIGH:LOW);
  analogWrite(ENA, abs(tocDo));
}

void setDongCoPhai(int tocDo){
  tocDo = constrain(tocDo,-255,255);
  digitalWrite(IN3, tocDo>=0?HIGH:LOW);
  digitalWrite(IN4, tocDo>=0?LOW:HIGH);
  analogWrite(ENB, abs(tocDo));
}

// ==== Kiểm tra cảm biến có thấy line ====
bool thayLine(unsigned int *vals){
  for(int i=0;i<SO_CAM_BIEN;i++){
    unsigned int minv = trs.calibratedMin[i];
    unsigned int maxv = trs.calibratedMax[i];
    unsigned int nguong = minv + (maxv - minv) * heSoNhanBietCamBien[i];
    if(vals[i] < nguong) return true;
  }
  return false;
}

// ==== matLine ổn định ====
bool matLine(unsigned int *vals){
  static unsigned long thoiGianTrang=0;
  int soTrang=0;
  for(int i=0;i<SO_CAM_BIEN;i++){
    unsigned int nguong = trs.calibratedMin[i] + (trs.calibratedMax[i]-trs.calibratedMin[i])*heSoNhanBietCamBien[i];
    if(vals[i] > nguong) soTrang++;
  }
  if(soTrang <= 1){
    if(millis()-thoiGianTrang>80) return true;
  } else thoiGianTrang=millis();
  return false;
}

// ==== In giá trị cảm biến ====
void inCamBien(unsigned int *mang,int sl){
  for(int i=0;i<sl;i++){
    unsigned int minv = trs.calibratedMin[i];
    unsigned int maxv = trs.calibratedMax[i];
    unsigned int nguong = minv + (maxv - minv)*heSoNhanBietCamBien[i];
    bool trang = mang[i]>nguong;
    Serial.print(trang?"1":"0");
    Serial.print("("); Serial.print(mang[i]); Serial.print(")");
    if(i<sl-1) Serial.print(" ");
  }
}

// ==== Tìm lại line ====
bool timLaiLine(){
  Serial.println("⚠️  Mất line! Quét tìm lại...");
  int tocDo=70;
  unsigned long startTime=millis();
  unsigned long maxTime=4000;

  while(millis()-startTime<maxTime){
    // Tiến thẳng
    setDongCoTrai(tocDo); setDongCoPhai(tocDo);
    trs.readCalibrated(giaTriCamBien); 
    if(thayLine(giaTriCamBien)){Serial.println("✅ Bắt line khi tiến thẳng!"); return true;}

    // Quét trái
    setDongCoTrai(-tocDo); setDongCoPhai(tocDo);
    trs.readCalibrated(giaTriCamBien);
    if(thayLine(giaTriCamBien)){Serial.println("✅ Bắt line khi quét trái!"); return true;}

    // Quét phải
    setDongCoTrai(tocDo); setDongCoPhai(-tocDo);
    trs.readCalibrated(giaTriCamBien);
    if(thayLine(giaTriCamBien)){Serial.println("✅ Bắt line khi quét phải!"); return true;}
  }

  setDongCoTrai(0); setDongCoPhai(0);
  Serial.println("❌ Không tìm được line!");
  return false;
}

// ==== Hiệu chuẩn tự động ====
void hieuChuanTuDong(){
  Serial.println("🔧 Hiệu chuẩn cảm biến chính xác...");

  for(int i=0;i<SO_CAM_BIEN;i++){
    trs.calibratedMin[i] = 1023;
    trs.calibratedMax[i] = 0;
  }

  unsigned int vals[SO_CAM_BIEN];

  for(int i=0;i<600;i++){
    setDongCoTrai(i<300?40:-40);
    setDongCoPhai(i<300?40:-40);

    for(int j=0;j<SO_CAM_BIEN;j++){
      vals[j] = analogRead(j);
      if(vals[j] < trs.calibratedMin[j]) trs.calibratedMin[j] = vals[j];
      if(vals[j] > trs.calibratedMax[j]) trs.calibratedMax[j] = vals[j];
    }
    delay(5);
  }

  setDongCoTrai(0);
  setDongCoPhai(0);

  Serial.print("Min: ");
  for(int i=0;i<SO_CAM_BIEN;i++){ Serial.print(trs.calibratedMin[i]); Serial.print(" "); }
  Serial.println();
  Serial.print("Max: ");
  for(int i=0;i<SO_CAM_BIEN;i++){ Serial.print(trs.calibratedMax[i]); Serial.print(" "); }
  Serial.println();
}

// ==== Setup ====
void setup(){
  Serial.begin(115200);
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT); pinMode(ENA,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT); pinMode(ENB,OUTPUT);

  hieuChuanTuDong();
  thoiGianCu=millis();
}

// ==== Loop ====
void loop(){
  if(Serial.available()){
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if(cmd.startsWith("nhay[")){
      int idx = cmd.substring(5,cmd.indexOf(']')).toInt();
      float val = cmd.substring(cmd.indexOf('=')+1).toFloat();
      if(idx>=0 && idx<SO_CAM_BIEN){
        heSoNhanBietCamBien[idx]=constrain(val,0.2,0.9);
        Serial.print("⚙️ CamBien["); Serial.print(idx); Serial.print("] = "); Serial.println(heSoNhanBietCamBien[idx],2);
      }
    } else if(cmd.startsWith("nhay=")){
      float val = cmd.substring(5).toFloat(); val=constrain(val,0.2,0.9);
      for(int i=0;i<SO_CAM_BIEN;i++) heSoNhanBietCamBien[i]=val;
      Serial.print("⚙️ Tất cả cảm biến = "); Serial.println(val,2);
    }
  }

  unsigned int viTri = trs.readLine(giaTriCamBien);
  trs.readCalibrated(giaTriCamBien);

  long saiSo = (long)viTri - 2000L;
  if(abs(saiSo)<15) saiSo=0;

  unsigned long bayGio = millis();
  float dt = max(1.0f,(float)(bayGio-thoiGianCu));
  float daoHam = (float)(saiSo-saiSoCu)/dt;
  tongSaiSo += saiSo*dt;
  tongSaiSo = constrain(tongSaiSo,-30000,30000);

  // ==== Mức độ lệch (0 → 1) ====
  float doLech = abs(saiSo) / 2000.0;

  // ================================
  // 🚗 NEW 1: Giảm tốc tổng khi vào cua
  // ================================
  int tocDoThucTe = tocDoCoBan - (int)(doLech * 70);
  tocDoThucTe = constrain(tocDoThucTe, 40, tocDoCoBan);

  // ================================
  // 🚗 NEW 2: PID động theo độ cua
  // ================================
  float Kp = Kp_base * (1 + doLech * 1.5);
  float Kd = Kd_base * (1 + doLech * 2.2);

  float chinhLai = Kp * saiSo + Ki * tongSaiSo + Kd * daoHam;

  int tocTrai = tocDoThucTe - (int)chinhLai;
  int tocPhai = tocDoThucTe + (int)chinhLai;

  // ================================
  // 🚗 NEW 3: Giảm tốc bánh ngoài cua
  // ================================
  if (saiSo > 400) {          // Cua phải mạnh
      tocTrai *= 0.55;        
  } 
  else if (saiSo < -400) {    // Cua trái mạnh
      tocPhai *= 0.55;        
  }
  else if (saiSo > 200) {     
      tocTrai *= 0.55;
  }
  else if (saiSo < -200) {    
      tocPhai *= 0.55;
  }

  tocTrai = constrain(tocTrai,tocDoToiThieu,tocDoToiDa);
  tocPhai = constrain(tocPhai,tocDoToiThieu,tocDoToiDa);

  // ================================
  // NEW 4: Làm mượt tốc độ
  // ================================
  static int cuTrai = 0, cuPhai = 0;
  tocTrai = (tocTrai + cuTrai * 2) / 3;
  tocPhai = (tocPhai + cuPhai * 2) / 3;
  cuTrai = tocTrai;
  cuPhai = tocPhai;

  // ==== Nếu mất line → tìm lại ====
  if(!daBatLaiLine && matLine(giaTriCamBien)){
    setDongCoTrai(0); setDongCoPhai(0);
    if(timLaiLine()){
      saiSoCu=0; tongSaiSo=0; thoiGianCu=millis();
      daBatLaiLine=true;
    }
    return;
  } 
  else if(thayLine(giaTriCamBien)){
    daBatLaiLine=false;
  }

  setDongCoTrai(tocTrai);
  setDongCoPhai(tocPhai);

  // ==== In log ====
  if(millis()-lanInCu>250){
    lanInCu=millis();
    Serial.print("📍 pos="); Serial.print(viTri);
    Serial.print(" | err="); Serial.print(saiSo);
    Serial.print(" | toc="); Serial.print(tocDoThucTe);
    Serial.print(" | CamBien: "); inCamBien(giaTriCamBien,SO_CAM_BIEN);
    Serial.println();
  }

  saiSoCu=saiSo;
  thoiGianCu=bayGio;
  delay(5);
}
