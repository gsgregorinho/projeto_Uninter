// src/main.cpp
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <SD.h>
#include <vector>
#include <TJpg_Decoder.h>
// ==== BEGIN PATCH HELPERS (injected by assistant) ====
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <sys/time.h> // for settimeofday
#include <ESPmDNS.h>

// Globals for web module
static File webUploadFile;
static bool webUploadOpen = false;
String reminderText = "";

// --- web sync / state counters (para cliente web detectar mudanças)
volatile unsigned long sdChangeCounter = 0;       // incrementado ao criar/excluir/editar arquivos no SD
volatile unsigned long serverStateVersion = 0;   // uso geral (incluir quando quiser forçar refresh)
unsigned long reminderStartEpoch = 0;            // início do lembrete (epoch seconds)
unsigned long reminderEndEpoch = 0;              // fim do lembrete (epoch seconds)

static SemaphoreHandle_t tftMutex = NULL;
static SemaphoreHandle_t sdMutex = NULL;
static bool sdMounted = false;

void ensureInitPatches(){
  if (!tftMutex) tftMutex = xSemaphoreCreateMutex();
  if (!sdMutex) sdMutex = xSemaphoreCreateMutex();
}

// Logging buffer
#define ASSIST_LOG_LINES 128
static String assistLogBuf[ASSIST_LOG_LINES];
static int assistLogIdx = 0;
void logAddAssist(const char *fmt, ...) {
  char buf[256];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  String s = String("[") + String(millis()) + "] " + buf;
  assistLogBuf[assistLogIdx++] = s;
  if (assistLogIdx >= ASSIST_LOG_LINES) assistLogIdx = 0;
  Serial.println(s);
}

// --- Protótipos do módulo Web ---
void initWebServer();
void loadSavedReminder();

void dumpAssistLogToSd(const char *path = "/error.log") {
  if (!sdMounted) { logAddAssist("[LOG] SD not mounted"); return; }
  if (!xSemaphoreTake(sdMutex, pdMS_TO_TICKS(500))) return;
  File f = SD.open(path, FILE_APPEND);
  if (!f) { xSemaphoreGive(sdMutex); logAddAssist("[LOG] failed to open %s", path); return; }
  for (int i=0;i<ASSIST_LOG_LINES;i++){
    int idx = (assistLogIdx + i) % ASSIST_LOG_LINES;
    if (assistLogBuf[idx].length()) f.println(assistLogBuf[idx]);
  }
  f.close();
  xSemaphoreGive(sdMutex);
  logAddAssist("[LOG] dumped to SD");
}

// Toast
struct AssistToast { String text; uint32_t until; bool active; };
static AssistToast assistToast = {"",0,false};
void showToastAssist(const String &txt, uint32_t ms=2000){
  assistToast.text = txt; assistToast.until = millis() + ms; assistToast.active = true;
  logAddAssist("[TOAST] %s", txt.c_str());
}

void processToastAssist(TFT_eSPI &tft){
  if (!assistToast.active) return;
  if (millis() > assistToast.until) { assistToast.active = false; return; }
  if (xSemaphoreTake(tftMutex, pdMS_TO_TICKS(100))){
    int pad = 10;
    int w = tft.width() - pad*2;
    int h = 30;
    int x = pad;
    int y = tft.height() - h - pad;
    tft.fillRoundRect(x,y,w,h,6,TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE);
    tft.drawString(assistToast.text, tft.width()/2, y+h/2);
    xSemaphoreGive(tftMutex);
  }
}


//jj

// SD wrappers
bool sdInitAssist(int csPin, SPIClass &spi = SPI, uint32_t freq = 4000000){
  ensureInitPatches();
  if (!xSemaphoreTake(sdMutex, pdMS_TO_TICKS(500))) return false;
  bool ok = SD.begin(csPin, spi, freq);
  sdMounted = ok;
  xSemaphoreGive(sdMutex);
  logAddAssist(ok ? "[SD] mounted" : "[SD] init failed");
  return ok;
}

File sdOpenAssist(const char *path, const char* mode = FILE_READ){
  if (!sdMounted) return File();
  if (!xSemaphoreTake(sdMutex, pdMS_TO_TICKS(500))) return File();
  File f = SD.open(path, mode);
  xSemaphoreGive(sdMutex);
  return f;
}

// WiFi helpers
void wifiEventAssist(WiFiEvent_t event){
  if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP){
    showToastAssist("WiFi conectado",1500);
    logAddAssist("[WIFI] IP: %s", WiFi.localIP().toString().c_str());
  } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED){
    showToastAssist("WiFi desconectado",1500);
    logAddAssist("[WIFI] disconnected");
  }
}
// ==== END PATCH HELPERS ====

TFT_eSPI tft = TFT_eSPI();
Preferences prefs;
WebServer server(80);

// --- Wallpaper persistente (caminho completo com leading '/')
String currentWallpaperPath = "";

// NeoPixel config
#define LED_PIN 33
#define NUM_LEDS 7
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// SD / SPI pins (user request)
#define SD_CS_PIN 5
#define SD_MOSI_PIN 13
#define SD_SCK_PIN 14
#define SD_MISO_PIN 27
#define FREQ_SD 20000000U // 20 MHz — seguro para a maioria dos cartões

// --- Novo global para proteger decodificação JPEG concorrente
volatile bool jpgBusy = false;

SPIClass sdSPI = SPIClass(VSPI);
bool sdInitialized = false;
// servidor start marker (usado para o cliente detectar reinício)
unsigned long serverStartMillis = 0;

// App states
enum AppState {
  APP_CALIBRATE, APP_HOME, APP_MENU,
  APP_FILES, APP_FILES_TEXTOS, APP_FILES_IMAGENS,
  APP_BLUETOOTH,
  APP_LEDS, APP_LED_COLOR_SELECT,
  APP_SETTINGS,
  APP_WIFI_PORTAL
};
AppState appState = APP_CALIBRATE;

// Navigation stack (simple)
AppState navStack[12];
int navTop = 0;

// Geometry / rects
struct Rect { int x,y,w,h; };
Rect btnMenu, btnBack, btnNext;
Rect btnFiles, btnBT, btnLeds, btnSettings;
Rect btnFilesTextos, btnFilesImagens;
Rect btToggleRect;
Rect ledToggleRect, ledColorRect, ledSmoothRect, ledRainbowRect;
Rect brightRects[3];
Rect colorSquares[6];

// Image viewer specific rects
Rect topBackRect; // top-left back on image view
Rect imgPrevRect, imgNextRect, imgWallpaperRect;

// File-system UI state
std::vector<String> sdTxtFiles;
std::vector<String> sdImageFiles; // jpg/jpeg supported
std::vector<Rect> fileEntryRects; // visible rects for touch handling (shared)
int filesListPage = 0;
int filesPerPage = 0;

String currentFileName = "";
std::vector<String> currentFileLines;
std::vector<String> currentFileDisplayLines; // wrapped lines for display
int currentFilePage = 0;
int currentFilePagesTotal = 0;
int fileTextSize = 1; // font size for file content

// Image viewing state
String currentImageName = ""; // when non-empty we are viewing image
int currentImageIndex = -1; // index into sdImageFiles

// Button sizing (menu standard)
int MENU_BTN_W = 200;
int MENU_BTN_H = 36;
int MENU_BTN_GAP = 8;
int BACK_W = 72, BACK_H = 30;

int LED_PAGE = 0; // 0 = main led controls, 1 = brilho

// draw flags
bool homeDrawn=false, menuDrawn=false, filesDrawn=false, filesTextosDrawn=false, filesImagensDrawn=false;
bool btDrawn=false, ledsDrawn=false, ledColorDrawn=false, settingsDrawn=false, wifiPortalDrawn=false;

// NTP / timezone
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -3 * 3600;
const int daylightOffset_sec = 0;
// ===== NTP sync helpers =====
unsigned long lastNtpSyncMillis = 0;
const unsigned long NTP_SYNC_INTERVAL_MS = 6UL * 3600UL * 1000UL; // 6 horas padrão

// touch / mapping
const unsigned long TOUCH_POLL_MS = 40;
const unsigned long TOUCH_DEBOUNCE_MS = 220;
unsigned long lastTouchPoll = 0;
unsigned long lastTouchHandled = 0;
static inline int clampi(int v,int a,int b){ if(v<a) return a; if(v>b) return b; return v; }
static inline bool inRect(const Rect &r,int x,int y){ return x >= r.x && x <= (r.x + r.w) && y >= r.y && y <= (r.y + r.h); }

// calibration/mapping persisted
bool calibrated=false;
int cal_min_x = 0, cal_max_x = 4095, cal_min_y = 0, cal_max_y = 4095;
uint8_t mapXSource = 0, mapYSource = 1;
float mapXScale = 1.0f, mapXOffset = 0.0f;
float mapYScale = 1.0f, mapYOffset = 0.0f;

// hardware & app state vars
bool ledState = false;
int brightnessIndex = 1; // 0=min,1=med,2=max
int selectedColorIndex = -1;
uint8_t btHardwareState = 0; // pin 26
bool wifiConnected = false;

// BUTTON COLOR CONSISTENCY
int BTN_R = 80, BTN_G = 120, BTN_B = 160;
inline uint16_t buttonColor(){ return tft.color565(BTN_R, BTN_G, BTN_B); }

// LED effect state
enum LedEffect { EFFECT_NONE=0, EFFECT_SMOOTH=1, EFFECT_RAINBOW=2 };
volatile LedEffect currentEffect = EFFECT_NONE;
unsigned long lastEffectMillis = 0;
unsigned long rainbowInterval = 80; // ms per step
uint16_t rainbowPos = 0;
unsigned long smoothInterval = 80;
int smoothFromColor = 0, smoothToColor = 0;
int smoothStep = 0, smoothStepsTotal = 30;
uint8_t brightnessValue = 150; // 0..255

bool smoothLoop = false;

// WiFi / portal
bool portalActive = false;
String portalAPName = "DeviceSetup-AP";

// touch sampling
const int TOUCH_SAMPLES = 5;
bool readRawMedian(uint16_t &rx, uint16_t &ry){
  uint16_t sx[TOUCH_SAMPLES], sy[TOUCH_SAMPLES];
  int got=0;
  for(int i=0;i<TOUCH_SAMPLES;i++){
    uint16_t tx=0, ty=0;
    noInterrupts();
    bool ok = tft.getTouch(&tx, &ty);
    interrupts();
    if (ok){ sx[got]=tx; sy[got]=ty; got++; }
    delay(6);
  }
  if (!got) return false;
  for(int i=1;i<got;i++){
    uint16_t kx = sx[i], ky = sy[i]; int j = i-1;
    while(j>=0 && sx[j] > kx){ sx[j+1]=sx[j]; sy[j+1]=sy[j]; j--; }
    sx[j+1]=kx; sy[j+1]=ky;
  }
  rx = sx[got/2]; ry = sy[got/2];
  return true;
}

// Preferences persist/load
void saveCalibrationAndMapping(){
  prefs.begin("touch", false);
  prefs.putInt("minx", cal_min_x);
  prefs.putInt("maxx", cal_max_x);
  prefs.putInt("miny", cal_min_y);
  prefs.putInt("maxy", cal_max_y);
  prefs.putBool("cal", true);
  prefs.putUChar("mxs", mapXSource);
  prefs.putUChar("mys", mapYSource);
  prefs.putFloat("mxscl", mapXScale);
  prefs.putFloat("mxoff", mapXOffset);
  prefs.putFloat("myscl", mapYScale);
  prefs.putFloat("myoff", mapYOffset);
  prefs.end();
}

bool loadCalibrationAndMapping(){
  prefs.begin("touch", true);
  if (!prefs.isKey("cal")){ prefs.end(); return false; }
  calibrated = prefs.getBool("cal", false);
  if (!calibrated){ prefs.end(); return false; }
  cal_min_x = prefs.getInt("minx", cal_min_x);
  cal_max_x = prefs.getInt("maxx", cal_max_x);
  cal_min_y = prefs.getInt("miny", cal_min_y);
  cal_max_y = prefs.getInt("maxy", cal_max_y);
  mapXSource = prefs.getUChar("mxs", mapXSource);
  mapYSource = prefs.getUChar("mys", mapYSource);
  mapXScale  = prefs.getFloat("mxscl", mapXScale);
  mapXOffset = prefs.getFloat("mxoff", mapXOffset);
  mapYScale  = prefs.getFloat("myscl", mapYScale);
  mapYOffset = prefs.getFloat("myoff", mapYOffset);
  prefs.end();
  return true;
}

// Compute mapping (same as before)
void computeAndStoreMapping(uint16_t rawx[5], uint16_t rawy[5], int px[5], int py[5]){
  float mean_px=0, mean_py=0, mean_rx=0, mean_ry=0;
  for(int i=0;i<5;i++){ mean_px+=px[i]; mean_py+=py[i]; mean_rx+=rawx[i]; mean_ry+=rawy[i]; }
  mean_px/=5.0f; mean_py/=5.0f; mean_rx/=5.0f; mean_ry/=5.0f;
  float cov_px_rx=0, cov_px_ry=0, cov_py_rx=0, cov_py_ry=0;
  for(int i=0;i<5;i++){
    cov_px_rx += (rawx[i]-mean_rx)*(px[i]-mean_px);
    cov_px_ry += (rawy[i]-mean_ry)*(px[i]-mean_px);
    cov_py_rx += (rawx[i]-mean_rx)*(py[i]-mean_py);
    cov_py_ry += (rawy[i]-mean_ry)*(py[i]-mean_py);
  }
  mapXSource = (fabsf(cov_px_rx) >= fabsf(cov_px_ry)) ? 0 : 1;
  mapYSource = (fabsf(cov_py_ry) >= fabsf(cov_py_rx)) ? 1 : 0;

  auto computeParams = [&](uint8_t source, float &scale, float &offset, int screenVals[5]){
    float rmean=0, pmean=0;
    for(int i=0;i<5;i++){
      float r = (source==0) ? (float)rawx[i] : (float)rawy[i];
      rmean += r; pmean += screenVals[i];
    }
    rmean/=5.0f; pmean/=5.0f;
    float varr=0, cov=0;
    for(int i=0;i<5;i++){
      float r = (source==0) ? (float)rawx[i] : (float)rawy[i];
      float p = (float)screenVals[i];
      varr += (r-rmean)*(r-rmean);
      cov  += (r-rmean)*(p-pmean);
    }
    if (varr < 1e-6f){ scale = 1.0f; offset = pmean - scale * rmean; return; }
    scale = cov / varr;
    offset = pmean - scale * rmean;
  };

  int screenX[5], screenY[5];
  for(int i=0;i<5;i++){ screenX[i]=px[i]; screenY[i]=py[i]; }
  computeParams(mapXSource, mapXScale, mapXOffset, screenX);
  computeParams(mapYSource, mapYScale, mapYOffset, screenY);

  uint16_t minx=rawx[0], maxx=rawx[0], miny=rawy[0], maxy=rawy[0];
  for(int i=1;i<5;i++){
    if (rawx[i]<minx) minx=rawx[i];
    if (rawx[i]>maxx) maxx=rawx[i];
    if (rawy[i]<miny) miny=rawy[i];
    if (rawy[i]>maxy) maxy=rawy[i];
  }
  int padx = max(20, (int)((maxx - minx) * 0.04f));
  int pady = max(20, (int)((maxy - miny) * 0.04f));
  cal_min_x = clampi(minx - padx, 0, 4095);
  cal_max_x = clampi(maxx + padx, 0, 4095);
  cal_min_y = clampi(miny - pady, 0, 4095);
  cal_max_y = clampi(maxy + pady, 0, 4095);

  saveCalibrationAndMapping();
}

// Calibration UI (same)
void drawCross(int x,int y){
  noInterrupts();
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  tft.drawLine(x-12, y, x+12, y, TFT_WHITE);
  tft.drawLine(x, y-12, x, y+12, TFT_WHITE);
  tft.fillCircle(x, y, 2, TFT_WHITE);
  tft.endWrite();
  interrupts();
}
bool waitForTouchAndSample(uint16_t &rx, uint16_t &ry, unsigned long hold_ms=700){
  unsigned long t0 = millis();
  while(millis() - t0 < 5000){
    if (readRawMedian(rx, ry)){
      unsigned long tpress = millis();
      int cnt = 1; uint32_t ax = rx, ay = ry;
      while(millis() - tpress < hold_ms){
        uint16_t rx2=0, ry2=0;
        if (!readRawMedian(rx2, ry2)) break;
        ax += rx2; ay += ry2; cnt++;
        delay(30);
      }
      rx = (uint16_t)(ax/cnt); ry = (uint16_t)(ay/cnt);
      return true;
    }
    delay(20);
  }
  return false;
}
void doCalibration(){
  int W = tft.width(), H = tft.height();
  int margin = 18;
  int px[5] = { margin, W - margin - 1, W - margin - 1, margin, W/2 };
  int py[5] = { margin, margin, H - margin - 1, H - margin - 1, H/2 };
  uint16_t rawx[5], rawy[5];
  for(int i=0;i<5;i++){
    drawCross(px[i], py[i]);
    uint16_t rx=0, ry=0;
    bool ok = waitForTouchAndSample(rx, ry, 900);
    if (!ok){ calibrated = false; return; }
    rawx[i] = rx; rawy[i] = ry;
    delay(200);
  }
  computeAndStoreMapping(rawx, rawy, px, py);
  calibrated = true;
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM); tft.setTextSize(2); tft.setTextColor(TFT_GREEN);
  tft.drawString("Calibrado", tft.width()/2, tft.height()/2);
  tft.endWrite();
  delay(700);
}

// map raw -> screen
bool mapRawToScreen(uint16_t rawx, uint16_t rawy, int &sx, int &sy){
  if (!calibrated){
    if (rawx <= (uint16_t)tft.width() && rawy <= (uint16_t)tft.height()){
      sx = clampi(rawx, 0, tft.width()-1);
      sy = clampi(rawy, 0, tft.height()-1);
      return true;
    }
    return false;
  }
  float rx = (mapXSource==0) ? (float)rawx : (float)rawy;
  float ry = (mapYSource==0) ? (float)rawx : (float)rawy;
  float fx = mapXScale * rx + mapXOffset;
  float fy = mapYScale * ry + mapYOffset;
  sx = clampi((int)roundf(fx), 0, tft.width()-1);
  sy = clampi((int)roundf(fy), 0, tft.height()-1);
  return true;
}

// Compute nav button rects (positions at bottom) but do not necessarily draw
void computeNavRects(){
  BACK_W = min(72, max(56, tft.width()/6));
  BACK_H = 30;
  int bx = 4;
  int by = tft.height() - BACK_H - 6; // bottom with small margin
  btnBack = {bx, by, BACK_W, BACK_H};
  int nx = tft.width() - 4 - BACK_W;
  btnNext = {nx, by, BACK_W, BACK_H};
}

// Draw Bottom Back and Next buttons
void drawBackButton(){
  computeNavRects();
  int bx = btnBack.x, by = btnBack.y;
  tft.startWrite();
  tft.fillRoundRect(bx, by, btnBack.w, btnBack.h, 6, buttonColor());
  tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  tft.drawString("Voltar", bx + btnBack.w/2, by + btnBack.h/2);
  tft.endWrite();
}
void drawNextButton(){
  computeNavRects();
  int bx = btnNext.x, by = btnNext.y;
  tft.startWrite();
  tft.fillRoundRect(bx, by, btnNext.w, btnNext.h, 6, buttonColor());
  tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  tft.drawString("Prox", bx + btnNext.w/2, by + btnNext.h/2);
  tft.endWrite();
}

// Top-left Back button used on image viewer
void drawTopBackButton(){
  int bx = 6, by = 6, bw = 70, bh = 28;
  topBackRect = {bx, by, bw, bh};
  tft.startWrite();
  tft.fillRoundRect(bx, by, bw, bh, 6, buttonColor());
  tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  tft.drawString("Voltar", bx + bw/2, by + bh/2);
  tft.endWrite();
}

// Title drawing that avoids overlapping nav buttons
void drawCenteredTitleAvoidButtons(const char* title){
  computeNavRects();
  int leftLimit = btnBack.x + btnBack.w + 8;
  int rightLimit = btnNext.x - 8;
  int center = (leftLimit + rightLimit) / 2;
  tft.startWrite();
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(title, center, 24);
  tft.endWrite();
}

// Top bar (home only)
const char* wkpt[7] = {"Dom","Seg","Ter","Qua","Qui","Sex","Sab"};
int lastDayShown = -1, lastHourShown = -1, lastMinShown = -1, lastSecShown = -1;
const int TIME_AREA_W = 140;

void drawHomeTopBarInitial(){
  tft.startWrite();
  tft.fillRect(0, 0, tft.width(), 28, tft.color565(12,12,20));
  tft.endWrite();
  lastDayShown = lastHourShown = lastMinShown = lastSecShown = -1;
}
// --- drawHomeTopBarFull: versão sem retângulo de fundo, com sombra embaixo do texto
void drawHomeTopBarFull(struct tm *ti){
  // Se ti for NULL, desenha um cabeçalho genérico/placeholder sobre o wallpaper (sem background)
  if (ti == NULL) {
    tft.startWrite();
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);
    tft.drawString("Sem hora valida", tft.width()/2, 12);
    tft.endWrite();

    lastDayShown = -2; // marca para forçar full redraw quando hora for válida
    lastHourShown = lastMinShown = lastSecShown = -1;
    return;
  }

  // monta strings de data e hora
  char datebuf[32], timebuf[16];
  sprintf(datebuf, "%s %02d/%02d/%04d", wkpt[ti->tm_wday], ti->tm_mday, ti->tm_mon+1, ti->tm_year+1900);
  sprintf(timebuf, "%02d:%02d:%02d", ti->tm_hour, ti->tm_min, ti->tm_sec);

  int centerX = tft.width() / 2;
  int yTime = 12; // posição vertical do texto grande da hora
  int yDate = 28; // posição vertical do texto pequeno da data

  // Desenha hora e data diretamente por cima do papel de parede, com sombra preta abaixo
  tft.startWrite();

  // Sombra (preta) deslocada +1,+1 para dar efeito de relevo
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.setTextColor(TFT_BLACK);
  tft.drawString(timebuf, centerX + 1, yTime + 1);

  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.drawString(datebuf, centerX + 1, yDate + 1);

  // Texto principal (branco) na posição correta, sobre a sombra
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(timebuf, centerX, yTime);

  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(datebuf, centerX, yDate);

  tft.endWrite();

  // atualiza trackers
  lastDayShown = ti->tm_mday;
  lastHourShown = ti->tm_hour;
  lastMinShown = ti->tm_min;
  lastSecShown = ti->tm_sec;
}


void redrawHomeTime(struct tm *ti){
  char timebuf[16];
  sprintf(timebuf, "%02d:%02d:%02d", ti->tm_hour, ti->tm_min, ti->tm_sec);

  int centerX = tft.width() / 2;
  int yTime = 12;
  int textWApprox = 140; // largura aproximada do texto (ajuste se necessário)
  int textHApprox = 28;

  // Tenta amostrar cor de fundo; se não suportado, cai para preto
  uint16_t bgColor = TFT_BLACK;
  bool haveBgSample = false;
  // Tenta usar readPixel (nem todas as configs TFT_eSPI compilam com readPixel habilitado)
  #if defined(TFT_ESPI_VERSION) || defined(ILI9341_DRIVER) || defined(ST7735_DRIVER) || true
  // Protege leitura com startWrite/endWrite em drivers que suportam readPixel
  tft.startWrite();
  // tenta ler o pixel central do retângulo onde a hora será desenhada
  int sampleX = centerX;
  int sampleY = yTime + textHApprox/2;
  #if defined(TFT_eSPI) && defined(ESP32)
  // use readPixel se disponível
  // Nota: nem toda configuração do TFT_eSPI habilita readPixel; se der erro, comente este bloco.
  uint32_t pcol = tft.readPixel(sampleX, sampleY);
  bgColor = (uint16_t)pcol;
  haveBgSample = true;
  #endif
  tft.endWrite();
  #endif

  // Se não conseguimos amostrar, tentamos sample aproximado da cor média de tela (fallback)
  if (!haveBgSample) {
    // fallback rápido: pega pixel (centerX, yTime) via getPixelColor por desenho temporário (custoso) — aqui mantemos preto
    bgColor = TFT_BLACK;
  }

  // Agora desenhamos texto com background igual à cor amostrada (substitui área do texto, evitando rastro)
  tft.startWrite();
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  // desenha sombra: primeiro um texto com bgColor como fundo (limpa área), em preto deslocado
  // desenhamos a sombra usando (fg=BLACK, bg=bgColor) para garantir substituição dos pixels do fundo
  tft.setTextColor(TFT_BLACK, bgColor);
  tft.drawString(timebuf, centerX + 1, yTime + 1);

  // texto principal com fundo bgColor para evitar rastro (fg branco, bg amostrada)
  tft.setTextColor(TFT_WHITE, bgColor);
  tft.drawString(timebuf, centerX, yTime);
  tft.endWrite();

  lastHourShown = ti->tm_hour;
  lastMinShown = ti->tm_min;
  lastSecShown = ti->tm_sec;
}

void updateHomeTopBarMinuteHourDay(struct tm *ti){
  if (ti->tm_mday != lastDayShown){
    drawHomeTopBarFull(ti);
    return;
  }
  if (ti->tm_hour != lastHourShown){
    redrawHomeTime(ti);
    lastHourShown = ti->tm_hour;
    lastMinShown = ti->tm_min;
    lastSecShown = ti->tm_sec;
    return;
  }
  if (ti->tm_min != lastMinShown){
    redrawHomeTime(ti);
    lastMinShown = ti->tm_min;
    lastSecShown = ti->tm_sec;
    return;
  }
}

// Forward declarations for functions used before their definitions
void enterState(AppState s, bool pushToStack = true);
void drawFileContentPage();

// Helpers to redraw the currently active state
void drawHome();
void drawMenu();
void drawFiles();
void drawFilesTextos();
void drawFilesImagens();
void drawBluetooth();
void drawLeds();
void drawLedsPage1();
void drawLedColorSelect();
void drawSettings();

// JPEG helper and callback
bool showJpegFromSD(const char *path, uint8_t forceScale);
bool showJpegAlternative(const char *path, int16_t x, int16_t y, uint8_t scale);
bool jpgRender(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap);

// === Callback completo: substitua a sua versão anterior ===
// Callback jpgRender (mantenha como estava)
bool jpgRender(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  // Descarta blocos totalmente fora da tela
  if (y >= tft.height() || x >= tft.width()) return true;

  // Ajuste de clipping simples (não assume bitmap maior que tela)
  int16_t drawW = w;
  int16_t drawX = x;
  if (x < 0) { // caso improvável, mas cobre segurança
    int shift = -x;
    bitmap += shift; // avançar no buffer (cada pixel = 2 bytes na memória do TJpg_Decoder)
    drawW -= shift;
    drawX = 0;
  }
  if (drawX + drawW > tft.width()) drawW = tft.width() - drawX;
  if (drawW <= 0) return true;

  // Use startWrite/endWrite para proteger a transferência SPI da tela
  tft.startWrite();
  tft.pushImage(drawX, y, drawW, h, bitmap);
  tft.endWrite();

  return true; // continuar a decodificação
}

// Método alternativo para carregar JPEG
bool showJpegAlternative(const char *path, int16_t x, int16_t y, uint8_t scale) {
  File jpegFile = sdOpenAssist(path);
  if (!jpegFile) {
    Serial.println("Erro ao abrir arquivo");
    return false;
  }

  size_t fileSize = jpegFile.size();
  uint8_t *jpegBuffer = (uint8_t *)malloc(fileSize);
  if (!jpegBuffer) {
    Serial.println("Erro alocando buffer");
    jpegFile.close();
    return false;
  }

  // Ler arquivo para buffer
  if (jpegFile.read(jpegBuffer, fileSize) != fileSize) {
    Serial.println("Erro lendo arquivo");
    free(jpegBuffer);
    jpegFile.close();
    return false;
  }
  jpegFile.close();

  // Configurar decoder novamente
  TJpgDec.setJpgScale(scale);
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(jpgRender);

  // Usar drawJpg em vez de drawSdJpg
  JRESULT result = TJpgDec.drawJpg(x, y, jpegBuffer, fileSize);
  
  free(jpegBuffer);
  
  if (result != JDR_OK) {
    Serial.print("Erro drawJpg: ");
    Serial.println(result);
    return false;
  }

  return true;
}

bool showJpegFromSD(const char *path, uint8_t forceScale) {
  if (jpgBusy) {
    Serial.println("showJpegFromSD: decode em progresso, recusado");
    return false;
  }
  jpgBusy = true; // marcar início

  bool result_ok = false;

  if (!sdInitialized) {
    Serial.println("showJpegFromSD: SD não inicializado");
    jpgBusy = false;
    return false;
  }
  if (!SD.exists(path)) {
    Serial.print("showJpegFromSD: arquivo nao encontrado: ");
    Serial.println(path);
    jpgBusy = false;
    return false;
  }

  uint16_t imgW = 0, imgH = 0;
  JRESULT r = TJpgDec.getSdJpgSize(&imgW, &imgH, path);
  if (r != JDR_OK) {
    Serial.print("Erro getSdJpgSize, fallback: ");
    Serial.println(r);
  }

  Serial.printf("showJpegFromSD: %s -> %dx%d  (Tela: %dx%d)\n", path, imgW, imgH, tft.width(), tft.height());

  uint8_t candidates[3] = {1,2,4};
  uint8_t chosenScale = 1;
  if (forceScale == 1 || forceScale == 2 || forceScale == 4) {
    chosenScale = forceScale;
  } else {
    bool found = false;
    for (int i=0;i<3;i++){
      uint8_t s = candidates[i];
      uint16_t scaledW = (imgW + (s - 1)) / s;
      uint16_t scaledH = (imgH + (s - 1)) / s;
      if (scaledW <= (uint16_t)tft.width() && scaledH <= (uint16_t)tft.height()){
        chosenScale = s; found = true; break;
      }
    }
    if (!found) chosenScale = 4;
  }

  TJpgDec.setJpgScale(chosenScale);
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(jpgRender);

  // limpa tela antes do draw para evitar sobreposição
  //tft.startWrite(); tft.fillScreen(TFT_BLACK); tft.endWrite();

  // Ajuste seguro SPI para SD
  sdSPI.beginTransaction(SPISettings(FREQ_SD, MSBFIRST, SPI_MODE0));
  SPI.setFrequency(FREQ_SD);

  JRESULT jres = TJpgDec.drawSdJpg(0, 0, path);

  sdSPI.endTransaction();
  SPI.setFrequency(40000000);

  if (jres == JDR_OK) {
    Serial.println("Desenho via drawSdJpg OK");
    result_ok = true;
  } else {
    Serial.print("drawSdJpg falhou com codigo: ");
    Serial.println(jres);
    // Tentativa fallback carregando em RAM
    if (showJpegAlternative(path, 0, 0, chosenScale)) {
      Serial.println("Fallback drawJpg OK");
      result_ok = true;
    } else {
      Serial.println("Fallback também falhou");
      result_ok = false;
    }
  }

  jpgBusy = false; // liberar flag
  return result_ok;
}



// list TXT and JPG files in root
void listTxtFiles(){
  sdTxtFiles.clear();
  if (!sdInitialized) return;
  File root = sdOpenAssist("/");
  if (!root) return;
  File entry;
  while (true){
    entry = root.openNextFile();
    if (!entry) break;
    if (!entry.isDirectory()){
      String name = String(entry.name());
      String lower = name; lower.toLowerCase();
      if (lower.endsWith(".txt")) sdTxtFiles.push_back(name);
    }
    entry.close();
  }
  root.close();
  filesListPage = 0;
}

void listJpgFiles(){
  sdImageFiles.clear();
  if (!sdInitialized) return;
  File root = sdOpenAssist("/");
  if (!root) return;
  File entry;
  while ( (entry = root.openNextFile()) ){
    if (!entry.isDirectory()){
      String name = String(entry.name());
      String lower = name; lower.toLowerCase();
      if (lower.endsWith(".jpg") || lower.endsWith(".jpeg")) sdImageFiles.push_back(name);
    }
    entry.close();
  }
  root.close();
  filesListPage = 0;
}

// Draw file list page using same button style as menu.
void drawFilesTextos(){
  tft.startWrite();

  tft.fillScreen(TFT_BLACK);
  drawBackButton();
  drawCenteredTitleAvoidButtons("Textos SD");

  listTxtFiles();

  int topY = 48;
  computeNavRects();
  int bottom = btnBack.y - 12;
  int available = bottom - topY;
  int bh = MENU_BTN_H;
  int gap = MENU_BTN_GAP;
  filesPerPage = max(1, available / (bh + gap));
  int totalFiles = (int)sdTxtFiles.size();
  int totalPages = (totalFiles + filesPerPage - 1) / filesPerPage;
  if (filesListPage >= totalPages) filesListPage = max(0, totalPages - 1);

  int bw = MENU_BTN_W;
  int bx = (tft.width()-bw)/2;
  int by = topY;
  fileEntryRects.clear();
  int startIdx = filesListPage * filesPerPage;
  for (int i = 0; i < filesPerPage; ++i){
    int idx = startIdx + i;
    if (idx >= totalFiles) break;
    String fn = sdTxtFiles[idx];

    tft.fillRoundRect(bx, by, bw, bh, 6, buttonColor());
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);

    int maxChars = max(8, (bw - 12) / 6);
    String display = fn;
    if (display.length() > maxChars) display = display.substring(0, maxChars - 3) + "...";
    tft.drawString(display, bx + bw/2, by + bh/2);

    Rect r = { bx, by, bw, bh };
    fileEntryRects.push_back(r);

    by += bh + gap;
  }

  if (totalPages > 1) drawNextButton(); else { computeNavRects(); tft.fillRect(btnNext.x, btnNext.y, btnNext.w, btnNext.h, TFT_BLACK); }

  tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  char buf[32];
  sprintf(buf, "Pagina %d/%d", (totalPages==0?0:filesListPage+1), (totalPages==0?0:totalPages));
  tft.drawString(buf, tft.width()/2, btnBack.y + btnBack.h/2);

  tft.endWrite();
  currentFileName = "";
  filesTextosDrawn = true;
}

// Draw image list (same style). Only supported formats are listed.
void drawFilesImagens(){
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  drawBackButton();
  drawCenteredTitleAvoidButtons("Imagens SD");

  listJpgFiles();

  int topY = 48;
  computeNavRects();
  int bottom = btnBack.y - 12;
  int available = bottom - topY;
  int bh = MENU_BTN_H;
  int gap = MENU_BTN_GAP;
  filesPerPage = max(1, available / (bh + gap));
  int totalFiles = (int)sdImageFiles.size();
  int totalPages = (totalFiles + filesPerPage - 1) / filesPerPage;
  if (filesListPage >= totalPages) filesListPage = max(0, totalPages - 1);

  int bw = MENU_BTN_W;
  int bx = (tft.width()-bw)/2;
  int by = topY;
  fileEntryRects.clear();
  int startIdx = filesListPage * filesPerPage;
  for (int i = 0; i < filesPerPage; ++i){
    int idx = startIdx + i;
    if (idx >= totalFiles) break;
    String fn = sdImageFiles[idx];

    tft.fillRoundRect(bx, by, bw, bh, 6, buttonColor());
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);

    int maxChars = max(8, (bw - 12) / 6);
    String display = fn;
    if (display.length() > maxChars) display = display.substring(0, maxChars - 3) + "...";
    tft.drawString(display, bx + bw/2, by + bh/2);

    Rect r = { bx, by, bw, bh };
    fileEntryRects.push_back(r);

    by += bh + gap;
  }

  if (totalPages > 1) drawNextButton(); else { computeNavRects(); tft.fillRect(btnNext.x, btnNext.y, btnNext.w, btnNext.h, TFT_BLACK); }

  tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  char buf[32];
  sprintf(buf, "Pagina %d/%d", (totalPages==0?0:filesListPage+1), (totalPages==0?0:totalPages));
  tft.drawString(buf, tft.width()/2, btnBack.y + btnBack.h/2);

  tft.endWrite();
  currentImageName = ""; currentImageIndex = -1;
  filesImagensDrawn = true;
}

// Open image viewer for filename (assumes name in sdImageFiles)
void openImageViewer(const String &fname, int index){
  currentImageName = fname;
  currentImageIndex = index;
  // draw viewer (will call showJpegFromSD)
  // We'll draw UI first then decode image. If decode fails we show message and return to list.
}

void drawImageViewer() {
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  tft.endWrite();

  // top-left back
  drawTopBackButton();

  // small title with filename
  tft.startWrite();
  tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  String title = currentImageName;
  int maxTitleChars = max(8, (tft.width()-16) / 6);
  if (title.length() > maxTitleChars) title = title.substring(0, maxTitleChars - 3) + "...";
  tft.drawString(title, tft.width()/2, 20);
  tft.endWrite();

  // Tentar exibir a imagem
  String path = String("/") + currentImageName;
  bool ok = showJpegFromSD(path.c_str(), 1);

  if (!ok) {
    // show error message then return to list automatically
    tft.startWrite();
    tft.fillScreen(TFT_BLACK);
    drawBackButton();
    drawCenteredTitleAvoidButtons("Erro");
    tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
    tft.drawString("Formato não suportado ou arquivo corrompido", tft.width()/2, tft.height()/2);
    tft.endWrite();
    delay(900);
    // go back to list
    currentImageName = ""; currentImageIndex = -1;
    drawFilesImagens();
    return;
  }

  // draw bottom controls: Prev | Papel de Parede | Prox
  computeNavRects();
  int by = btnBack.y;
  int btnW = 84, btnH = 36;
  int gap = 12;
  int totalW = btnW*3 + gap*2;
  int startX = (tft.width() - totalW)/2;

  // Prev
  imgPrevRect = { startX, by, btnW, btnH };
  tft.startWrite();
  tft.fillRoundRect(imgPrevRect.x, imgPrevRect.y, imgPrevRect.w, imgPrevRect.h, 6, buttonColor());
  tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  tft.drawString("Ant.", imgPrevRect.x + imgPrevRect.w/2, imgPrevRect.y + imgPrevRect.h/2);

  // Wallpaper middle
  imgWallpaperRect = { startX + btnW + gap, by, btnW, btnH };
  tft.fillRoundRect(imgWallpaperRect.x, imgWallpaperRect.y, imgWallpaperRect.w, imgWallpaperRect.h, 6, buttonColor());
  tft.drawString("Papel de Parede", imgWallpaperRect.x + imgWallpaperRect.w/2, imgWallpaperRect.y + imgWallpaperRect.h/2);

  // Next
  imgNextRect = { startX + (btnW+gap)*2, by, btnW, btnH };
  tft.fillRoundRect(imgNextRect.x, imgNextRect.y, imgNextRect.w, imgNextRect.h, 6, buttonColor());
  tft.drawString("Prox", imgNextRect.x + imgNextRect.w/2, imgNextRect.y + imgNextRect.h/2);
  tft.endWrite();
}

// Set image as wallpaper placeholder (stores filename in prefs)
void setAsWallpaper(const String &fname){
  // grava caminho completo com leading '/'
  String full = fname;
  if (full.charAt(0) != '/') full = "/" + full;

  prefs.begin("app", false);
  prefs.putString("wallpaper", full);
  prefs.end();

  currentWallpaperPath = full;

  // confirmação rápida (com fundo preto para evitar sobreposição)
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  tft.fillRoundRect((tft.width()-260)/2, (tft.height()-40)/2, 260, 40, 6, buttonColor());
  tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  tft.drawString("Definido como papel de parede", tft.width()/2, tft.height()/2);
  tft.endWrite();
  delay(800);

  // Volta para HOME desenhando wallpaper por baixo
  enterState(APP_HOME, true);
  // desenha home (drawHome() usará showJpegFromSD protegido por jpgBusy)
  drawHome();
}

// helper: open image for given index and draw viewer
void openImageAndShowByIndex(int index){
  if (index < 0 || index >= (int)sdImageFiles.size()) return;
  currentImageIndex = index;
  currentImageName = sdImageFiles[index];
  drawImageViewer();
}

// Existing functions for files images (kept placeholder)
void drawBluetooth(){
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  drawBackButton();
  drawCenteredTitleAvoidButtons("Bluetooth");
  int bw = MENU_BTN_W, bh = MENU_BTN_H + 12;
  int bx = (tft.width()-bw)/2, by = 48;
  tft.fillRoundRect(bx,by,bw,bh,8,buttonColor());
  tft.setTextDatum(MC_DATUM); tft.setTextSize(1);

  prefs.begin("app", true);
  bool haveBt = prefs.isKey("bt_state");
  bool btShow = false;
  if (haveBt) btShow = prefs.getBool("bt_state", false);
  prefs.end();

  if (!btShow){
    tft.setTextColor(tft.color565(0,200,0));
    tft.drawString("Lig. Bluetooth", bx + bw/2, by + bh/2);
  } else {
    tft.setTextColor(tft.color565(200,0,0));
    tft.drawString("Desl. Bluetooth", bx + bw/2, by + bh/2);
  }
  tft.endWrite();
  btToggleRect = {bx,by,bw,bh};
  btDrawn = true;
}

// map selectedColorIndex to RGB
uint32_t colorForIndex(int idx){
  switch(idx){
    case 0: return strip.Color(200,20,20); // red
    case 1: return strip.Color(220,200,20); // yellow
    case 2: return strip.Color(240,120,200); // pink
    case 3: return strip.Color(40,120,220); // blue
    case 4: return strip.Color(20,160,80); // green
    case 5: return strip.Color(220,220,220); // white
    default: return strip.Color(200,200,200);
  }
}

void applyStripState(){
  strip.setBrightness(brightnessValue);
  if (!ledState){
    for(int i=0;i<NUM_LEDS;i++) strip.setPixelColor(i, 0);
    strip.show();
    return;
  }
  uint32_t c = (selectedColorIndex>=0) ? colorForIndex(selectedColorIndex) : strip.Color(200,200,200);
  for(int i=0;i<NUM_LEDS;i++) strip.setPixelColor(i, c);
  strip.show();
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void startSmoothTransition(int fromIdx, int toIdx, unsigned long duration_ms) {
  if (fromIdx < 0) fromIdx = 0;
  if (toIdx < 0) toIdx = 0;
  if (fromIdx > 5) fromIdx = fromIdx % 6;
  if (toIdx > 5) toIdx = toIdx % 6;
  if (duration_ms < 40) duration_ms = 40;

  smoothFromColor = fromIdx;
  smoothToColor   = toIdx;
  smoothStep      = 0;

  const unsigned long baseStepMs = 40UL;
  int steps = (int)(duration_ms / baseStepMs);
  if (steps < 1) steps = 1;
  smoothStepsTotal = steps;
  smoothInterval = duration_ms / (unsigned long)smoothStepsTotal;

  lastEffectMillis = millis();
  currentEffect = EFFECT_SMOOTH;
  ledState = true;

  uint32_t c = colorForIndex(smoothFromColor);
  strip.setBrightness(brightnessValue);
  for(int i=0;i<NUM_LEDS;i++) strip.setPixelColor(i, c);
  strip.show();
}

void startSmoothLoop(unsigned long duration_ms) {
  if (duration_ms < 40) duration_ms = 40;
  smoothLoop = true;

  smoothFromColor = (selectedColorIndex >= 0) ? selectedColorIndex : 0;
  smoothToColor   = (smoothFromColor + 1) % 6;
  smoothStep      = 0;

  const unsigned long baseStepMs = 40UL;
  int steps = (int)(duration_ms / baseStepMs);
  if (steps < 1) steps = 1;
  smoothStepsTotal = steps;
  smoothInterval = duration_ms / (unsigned long)smoothStepsTotal;

  lastEffectMillis = millis();
  currentEffect = EFFECT_SMOOTH;
  ledState = true;

  uint32_t c = colorForIndex(smoothFromColor);
  strip.setBrightness(brightnessValue);
  for(int i=0;i<NUM_LEDS;i++) strip.setPixelColor(i, c);
  strip.show();
}

void stopSmoothLoop() { smoothLoop = false; }

void updateEffects(){
  unsigned long now = millis();
  if (currentEffect == EFFECT_RAINBOW){
    if (now - lastEffectMillis >= rainbowInterval){
      lastEffectMillis = now;
      for(int i=0;i<NUM_LEDS;i++){
        uint8_t pos = (uint8_t)(rainbowPos + (i * (256 / NUM_LEDS)));
        strip.setPixelColor(i, Wheel(pos));
      }
      strip.show();
      rainbowPos++;
    }
  } else if (currentEffect == EFFECT_SMOOTH){
    if (now - lastEffectMillis >= smoothInterval){
      lastEffectMillis = now;
      uint32_t cfrom = colorForIndex(smoothFromColor);
      uint32_t cto   = colorForIndex(smoothToColor);
      uint8_t fr = (cfrom >> 16) & 0xFF, fg = (cfrom >> 8) & 0xFF, fb = cfrom & 0xFF;
      uint8_t tr = (cto >> 16) & 0xFF, tg = (cto >> 8) & 0xFF, tb = cto & 0xFF;
      float t = (float)smoothStep / (float)max(1, smoothStepsTotal);
      uint8_t rr = (uint8_t)round(fr + (tr - fr) * t);
      uint8_t rg = (uint8_t)round(fg + (tg - fg) * t);
      uint8_t rb = (uint8_t)round(fb + (tb - fb) * t);
      for(int i=0;i<NUM_LEDS;i++) strip.setPixelColor(i, strip.Color(rr, rg, rb));
      strip.show();
      smoothStep++;
      if (smoothStep > smoothStepsTotal){
        if (smoothLoop) {
          smoothFromColor = smoothToColor;
          smoothToColor = (smoothToColor + 1) % 6;
          smoothStep = 0;
        } else {
          currentEffect = EFFECT_NONE;
        }
      }
    }
  }
}

void drawLeds(){
  LED_PAGE = 0;
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  drawBackButton();
  drawNextButton();
  drawCenteredTitleAvoidButtons("Leds");
  int bw = MENU_BTN_W, bh = MENU_BTN_H;
  int gap = MENU_BTN_GAP;
  int bx = (tft.width()-bw)/2;
  int by = 48;

  tft.fillRoundRect(bx,by,bw,bh,6,buttonColor());
  tft.setTextDatum(MC_DATUM); tft.setTextSize(1);
  if (!ledState){
    tft.setTextColor(tft.color565(0,200,0));
    tft.drawString("Ligar", bx + bw/2, by + bh/2);
  } else {
    tft.setTextColor(tft.color565(200,0,0));
    tft.drawString("Desligar", bx + bw/2, by + bh/2);
  }
  ledToggleRect = {bx,by,bw,bh};
  by += bh + gap;

  tft.fillRoundRect(bx,by,bw,bh,6,buttonColor());
  tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  tft.drawString("Selecionar cor", bx + bw/2, by + bh/2); ledColorRect = {bx,by,bw,bh};
  by += bh + gap;
  tft.fillRoundRect(bx,by,bw,bh,6,buttonColor());
  tft.drawString("Troca suave", bx + bw/2, by + bh/2); ledSmoothRect = {bx,by,bw,bh};
  by += bh + gap;
  tft.fillRoundRect(bx,by,bw,bh,6,buttonColor());
  tft.drawString("Arco-iris", bx + bw/2, by + bh/2); ledRainbowRect = {bx,by,bw,bh};
  by += bh + gap;

  tft.endWrite();
  ledsDrawn = true;
}

void drawLedsPage1(){
  LED_PAGE = 1;
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  drawBackButton();
  drawNextButton();
  drawCenteredTitleAvoidButtons("Leds - Brilho");
  int bx = (tft.width()-MENU_BTN_W)/2;
  int by = 48;
  tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  tft.drawString("Ajuste de brilho", tft.width()/2, by);
  by += 28;

  int sq = 46;
  int totalW = sq*3 + 12*2;
  int sx = (tft.width() - totalW)/2;
  for(int i=0;i<3;i++){
    int rx = sx + i*(sq + 12);
    tft.fillRect(rx, by, sq, sq, (brightnessIndex==i) ? tft.color565(200,200,200) : tft.color565(80,80,80));
    tft.drawRect(rx, by, sq, sq, tft.color565(120,120,120));
    brightRects[i] = {rx,by,sq,sq};
    tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
    if (i==0) tft.drawString("Min", rx + sq/2, by + sq/2);
    if (i==1) tft.drawString("Med", rx + sq/2, by + sq/2);
    if (i==2) tft.drawString("Max", rx + sq/2, by + sq/2);
  }
  tft.endWrite();
  ledsDrawn = true;
}

void drawLedColorSelect(){
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  drawBackButton();
  drawCenteredTitleAvoidButtons("Selecionar Cor");
  int sq = min(48, (tft.width() - 60) / 3);
  int gap = 10;
  int startX = (tft.width() - (sq*3 + gap*2)) / 2;
  int y = 48;
  int idx = 0;

  uint32_t colors[6] = {
    strip.Color(200,20,20),
    strip.Color(220,200,20),
    strip.Color(240,120,200),
    strip.Color(40,120,220),
    strip.Color(20,160,80),
    strip.Color(220,220,220)
  };

  for(int row=0; row<2; row++){
    int x = startX;
    for(int col=0; col<3; col++){
      uint32_t c = colors[idx];
      uint8_t rr = (c >> 16) & 0xFF, gg = (c >> 8) & 0xFF, bb = c & 0xFF;
      tft.fillRect(x, y, sq, sq, tft.color565(rr, gg, bb));
      tft.drawRect(x, y, sq, sq, tft.color565(120,120,120));
      colorSquares[idx] = { x, y, sq, sq };
      idx++;
      x += sq + gap;
    }
    y += sq + gap;
  }

  if (selectedColorIndex >= 0 && selectedColorIndex < 6) {
    Rect r = colorSquares[selectedColorIndex];
    tft.drawRect(r.x - 2, r.y - 2, r.w + 4, r.h + 4, tft.color565(255,255,255));
    tft.drawRect(r.x - 3, r.y - 3, r.w + 6, r.h + 6, tft.color565(180,180,180));
  }

  tft.endWrite();
  ledColorDrawn = true;
}

void drawSettings(){
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  drawBackButton();
  drawCenteredTitleAvoidButtons("Configuracoes");
  int bw = MENU_BTN_W, bh = MENU_BTN_H, gap = MENU_BTN_GAP;
  int bx = (tft.width() - bw)/2, by = 48;
  tft.fillRoundRect(bx,by,bw,bh,6,buttonColor());
  tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  tft.drawString("Ajustar data e hora", bx + bw/2, by + bh/2); btnFilesTextos = {bx,by,bw,bh};
  by += bh + gap;
  tft.fillRoundRect(bx,by,bw,bh,6,buttonColor());
  tft.drawString("Definir lembretes", bx + bw/2, by + bh/2); btnFilesImagens = {bx,by,bw,bh};
  by += bh + gap;
  tft.fillRoundRect(bx,by,bw,bh,6,buttonColor());
  tft.drawString("Configurar WiFi", bx + bw/2, by + bh/2); btnSettings = {bx,by,bw,bh};
  tft.endWrite();
  settingsDrawn = true;
}

// WiFi portal functions omitted for brevity (kept same behavior)
void handleRoot(){
  String html = "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'/><title>WiFi Setup</title></head><body>";
  html += "<h3>Configurar WiFi</h3>";
  html += "<form action='/save' method='post'>";
  html += "SSID:<br><input name='ssid' maxlength='64'><br>";
  html += "Senha:<br><input name='pass' type='password' maxlength='64'><br><br>";
  html += "<input type='submit' value='Salvar'>";
  html += "</form><p>Ao salvar, o dispositivo tentara conectar.</p></body></html>";
  server.send(200, "text/html", html);
}

// forward declaration: garante que tryConnectFromPrefs possa ser chamado aqui mesmo que a definição venha depois no arquivo
bool tryConnectFromPrefs(unsigned long timeout_ms);

// connectFromPrefsTask: tarefa que tenta conectar usando as credenciais em prefs (evita bloquear handlers HTTP)
static void connectFromPrefsTask(void *pvParameters);

// ===== Substituir handleSave() =====
void handleSave(){
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  String response = "<!doctype html><html><body><h3>Salvando...</h3><p>Tentando conectar a rede: " + ssid + "</p></body></html>";
  server.send(200, "text/html", response);

  // grava credenciais de forma síncrona
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();

  // Parar o server do portal com segurança (libera portas/handlers)
  server.stop();

  // Não de-inicialize o driver WiFi aqui (use 'false'). Deinit é a principal causa dos erros.
  // Em vez disso, desligamos o AP e mudamos para STA depois de uma pequena pausa.
  WiFi.softAPdisconnect(false);
  portalActive = false;

  // Dar um pequeno tempo (hardware/network cleanup)
  delay(200);

  // Trocar para modo STA e iniciar conexão de forma assíncrona (task)
  WiFi.mode(WIFI_STA);
  // Não chamamos WiFi.begin() aqui no handler direto (evita bloquear o handler). Em vez disso criamos task.
  BaseType_t ok = xTaskCreatePinnedToCore(connectFromPrefsTask, "ap2sta", 4096, NULL, 1, NULL, 1);
  if (ok == pdPASS) {
    Serial.println("handleSave: connectFromPrefsTask criado com sucesso.");
  } else {
    Serial.println("handleSave: falha ao criar connectFromPrefsTask - tentando conectar no loop principal.");
    // fallback síncrono se task creation falhar
    tryConnectFromPrefs(10000);
  }
}

// ===== Substituir startPortal() com transição segura =====
void startPortal(){
  portalAPName = "DeviceSetup-AP";
  Serial.println("startPortal: preparando para iniciar AP (portal).");

  // Se estivermos como STA, desconecte gentileza do STA (sem deinit)
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("startPortal: desconectando STA antes de ativar AP.");
    WiFi.disconnect();            // desconecta da rede atual
    delay(150);
  }

  // Garantir que não haja AP ativo previamente (sem deinit do driver)
  WiFi.softAPdisconnect(false);
  delay(120);

  // Troca para modo AP e sobe o AP do portal
  WiFi.mode(WIFI_AP);
  WiFi.softAP(portalAPName.c_str());
  delay(120);

  // Reiniciar/limpar server antes de registrar handlers do portal
  server.stop();

  // Registra apenas os handlers mínimos para o portal
  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);

  // Sobe o servidor do portal
  server.begin();
  portalActive = true;

  // Desenha UI do portal
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  drawBackButton();
  drawCenteredTitleAvoidButtons("Configurar WiFi");
  tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  tft.drawString("Conecte seu celular ao AP:", tft.width()/2, 84);
  tft.setTextSize(2); tft.setTextColor(TFT_CYAN);
  tft.drawString(portalAPName, tft.width()/2, 110);
  tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  tft.drawString("Abra o navegador: http://192.168.4.1", tft.width()/2, 140);
  tft.endWrite();
  wifiPortalDrawn = true;

  Serial.println("startPortal: AP iniciado (portal) com sucesso.");
}


// Minimal implementations for drawHome, drawMenu, drawFiles to satisfy linker.
// These draw simple placeholders and initialize button rects used elsewhere.
void drawHome() {
  // desenha wallpaper se houver
  if (sdInitialized && currentWallpaperPath.length() > 0 && SD.exists(currentWallpaperPath.c_str())) {
    SPI.setFrequency(FREQ_SD);
    showJpegFromSD(currentWallpaperPath.c_str(), 1);
    SPI.setFrequency(40000000);
  } else {
    tft.startWrite();
    tft.fillScreen(TFT_BLACK);
    tft.endWrite();
  }

  // Top bar (horario)
  time_t now = time(NULL);
  struct tm ti;
  if (now > 1600000000UL) localtime_r(&now, &ti);
  drawHomeTopBarFull((now > 1600000000UL) ? &ti : NULL);

  // Lembrete centralizado (se houver e dentro do período programado)
  tft.startWrite();
  bool showReminder = false;
  if (reminderText.length() > 0) {
    // if both start and end are zero -> always show (compatibilidade)
    if (reminderStartEpoch == 0 && reminderEndEpoch == 0) {
      showReminder = true;
    } else {
      time_t now = time(NULL);
      if (now > 1600000000UL) { // hora válida
        if ((reminderStartEpoch == 0 || (unsigned long)now >= reminderStartEpoch) &&
            (reminderEndEpoch == 0   || (unsigned long)now <= reminderEndEpoch)) {
          showReminder = true;
        }
      }
    }
  }
  if (showReminder) {
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(2);
    tft.setTextColor(TFT_YELLOW);
    tft.drawString(reminderText, tft.width()/2, tft.height()/2 - 8);
  }
  tft.endWrite();

  // Botão Menu posicionado um pouco mais abaixo se houver lembrete
  int bw = 120, bh = 36;
  int menuY = (reminderText.length() > 0) ? (tft.height()/2 + 28) : 80;
  btnMenu = { (tft.width()-bw)/2, menuY, bw, bh };
  tft.fillRoundRect(btnMenu.x, btnMenu.y, btnMenu.w, btnMenu.h, 6, buttonColor());
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Menu", btnMenu.x + btnMenu.w/2, btnMenu.y + btnMenu.h/2);
  tft.endWrite();

  homeDrawn = true;
}

void drawMenu(){
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  drawBackButton();
  drawCenteredTitleAvoidButtons("Menu");
  int bw = MENU_BTN_W, bh = MENU_BTN_H, gap = MENU_BTN_GAP;
  int bx = (tft.width()-bw)/2, by = 48;
  tft.fillRoundRect(bx,by,bw,bh,6,buttonColor()); tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  tft.drawString("Arquivos", bx + bw/2, by + bh/2); btnFiles = {bx,by,bw,bh}; by += bh + gap;
  tft.fillRoundRect(bx,by,bw,bh,6,buttonColor()); tft.drawString("Bluetooth", bx + bw/2, by + bh/2); btnBT = {bx,by,bw,bh}; by += bh + gap;
  tft.fillRoundRect(bx,by,bw,bh,6,buttonColor()); tft.drawString("Leds", bx + bw/2, by + bh/2); btnLeds = {bx,by,bw,bh}; by += bh + gap;
  tft.fillRoundRect(bx,by,bw,bh,6,buttonColor()); tft.drawString("Configuracoes", bx + bw/2, by + bh/2); btnSettings = {bx,by,bw,bh};
  tft.endWrite();
  menuDrawn = true;
}

void drawFiles(){
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  drawBackButton();
  drawCenteredTitleAvoidButtons("Arquivos");
  int bw = MENU_BTN_W, bh = MENU_BTN_H, gap = MENU_BTN_GAP;
  int bx = (tft.width()-bw)/2, by = 48;
  tft.fillRoundRect(bx,by,bw,bh,6,buttonColor()); tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  tft.drawString("Textos", bx + bw/2, by + bh/2); btnFilesTextos = {bx,by,bw,bh}; by += bh + gap;
  tft.fillRoundRect(bx,by,bw,bh,6,buttonColor()); tft.drawString("Imagens", bx + bw/2, by + bh/2); btnFilesImagens = {bx,by,bw,bh};
  tft.endWrite();
  filesDrawn = true;
}
bool tryConnectFromPrefs(unsigned long timeout_ms = 10000) {
  prefs.begin("wifi", true);
  String ssid = prefs.getString("ssid", "");
  String pass = prefs.getString("pass", "");
  prefs.end();
  if (ssid.length() == 0) return false;

  Serial.printf("tryConnectFromPrefs: tentando '%s' por %lums\n", ssid.c_str(), timeout_ms);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  unsigned long start = millis();
  while (millis() - start < timeout_ms) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi conectado (prefs). IP: " + WiFi.localIP().toString());
      wifiConnected = true;
      return true;
    }
    delay(200);
  }
  Serial.println("tryConnectFromPrefs: falhou conectar WiFi");
  wifiConnected = false;
  return false;
}


// Tenta conectar WiFi a partir de prefs se necessário, sincroniza horário via NTP.
// Retorna true se conseguiu hora válida.
bool syncTimeWithNTP(unsigned long timeout_ms = 10000) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("syncTimeWithNTP: sem WiFi. Abortando.");
    return false;
  }

  Serial.println("syncTimeWithNTP: configTime() e aguardando NTP...");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  unsigned long start = millis();
  time_t now = time(nullptr);
  while ((now < 1600000000UL) && (millis() - start < timeout_ms)) {
    delay(200);
    now = time(nullptr);
  }
  if (now < 1600000000UL) {
    Serial.println("syncTimeWithNTP: falha ao obter hora NTP dentro do timeout.");
    return false;
  }
  lastNtpSyncMillis = millis();
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.printf("syncTimeWithNTP: sucesso. %02d:%02d:%02d %02d/%02d/%04d\n",
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
                timeinfo.tm_mday, timeinfo.tm_mon+1, timeinfo.tm_year+1900);
  return true;
}

// Handler acionado pelo botão "Ajustar data e hora"
void handleAdjustDateTimeButton() {
  // limpa tudo para preto
  int W = tft.width();
  int H = tft.height();
  tft.startWrite();
  tft.fillRect(0, 0, W, H, tft.color565(0,0,0));

  // mensagem inicial
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Sincronizando NTP...", W/2, H/2 - 12);
  tft.endWrite();

  // tentativa de sync
  bool ok = syncTimeWithNTP(12000); // 12s timeout

  // mostrar resultado em tela preta (substitui qualquer desenho anterior)
  tft.startWrite();
  tft.fillRect(0, 0, W, H, tft.color565(0,0,0));
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  if (ok) {
    tft.setTextColor(TFT_WHITE);
    tft.drawString("Hora ajustada via NTP", W/2, H/2);
  } else {
    tft.setTextColor(TFT_WHITE);
    tft.drawString("Falha NTP. Configure WiFi.", W/2, H/2);
  }
  tft.endWrite();

  // manter a mensagem por 2s
  delay(2000);

  // redesenha a tela de configurações inteira
  drawSettings();
}


bool robustSdInit(int retries = 4, unsigned long waitMs = 300) {
  // tentativa em frequência baixa para reduzir picos de corrente
  SPI.setFrequency(4000000UL);
  for (int i = 0; i < retries; ++i) {
    Serial.printf("SD try %d/%d\n", i+1, retries);
    if (sdInitAssist(SD_CS_PIN)) {
      Serial.println("SD montado via SD.begin");
      sdInitialized = true;
      SPI.setFrequency(20000000UL); // frequencia normal para SD ops
      return true;
    }
    delay(waitMs);
  }
  sdInitialized = false;
  Serial.println("SD.begin falhou após retries");
  SPI.setFrequency(20000000UL);
  return false;
}

// --- Logo simples: quadrado branco com G vermelho centrado
void drawLogo(){
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  int W = tft.width(), H = tft.height();
  int boxSize = min(W, H) * 3 / 5;
  int bx = (W - boxSize) / 2;
  int by = (H - boxSize) / 2;
  tft.fillRect(bx, by, boxSize, boxSize, TFT_WHITE);

  // letra G vermelha no centro. ajuste textSize se precisar
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(tft.color565(200,0,0)); // vermelho
  // calibrar tamanho conforme tela; 6 costuma ficar grande, 4-6 segure
  tft.setTextSize(6);
  tft.drawString("G", W/2, H/2 - (boxSize/20)); // pequeno ajuste vertical
  tft.endWrite();
}

// ------------------- Globals / helpers para inicialização segura -------------------
static bool webStarted = false;
static bool wifiAttemptedFromPrefs = false;

// Inicia serviços que dependem de rede (chama apenas uma vez)
static void startWebServices() {
  if (webStarted) return;
  webStarted = true;
  Serial.println("Iniciando serviços web...");
  // Carrega reminder e sobe o servidor (chama funções que existem no seu código)
  loadSavedReminder();
  initWebServer();
  // mDNS: opcional, tenta iniciar mas não falha o boot se der errado
  if (MDNS.begin("meudisplay")) Serial.println("mDNS iniciado: meudisplay.local");
  else Serial.println("mDNS: falha ao iniciar");
}


// ===== Substituir wifiMonitorTask() para evitar startWebServices durante portal =====
static void wifiMonitorTask(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    // Se portal ativo, não iniciamos os serviços web de STA (evita conflito)
    if (!portalActive && WiFi.status() == WL_CONNECTED && !webStarted) {
      Serial.println("wifiMonitorTask: WiFi conectado -> startWebServices()");
      startWebServices();
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // checa a cada 1s
  }
}

// cria a task (chame uma vez no setup)
static void startWifiMonitor() {
  // cria a tarefa se ainda não existir
  // stack 4096 costuma ser suficiente; prioridade 1
  xTaskCreatePinnedToCore(wifiMonitorTask, "wifiMon", 4096, NULL, 1, NULL, 1);
}
// -----------------------------------------------------------------------------------

// ===== Conectar a prefs em uma task separada (não-blocking para o handler) =====
static void connectFromPrefsTask(void *pvParameters) {
  (void) pvParameters;
  Serial.println("connectFromPrefsTask: iniciando tentativa de conectar a SSID salvo...");
  // tenta por 15s (sua tryConnectFromPrefs já faz WiFi.mode(WIFI_STA) internamente).
  tryConnectFromPrefs(15000);
  Serial.println("connectFromPrefsTask: fim da tentativa.");
  vTaskDelete(NULL); // encerra a própria task
}


void setup() {
  ensureInitPatches();
  Serial.begin(115200);

  // --- DISPLAY / SPI / TFT (rápido) ---
  SPI.begin();
  pinMode(TFT_CS, OUTPUT); digitalWrite(TFT_CS, HIGH);
  SPI.setFrequency(40000000);
  tft.init();
  tft.setRotation(1);
  tft.setTextWrap(false);
  tft.fillScreen(TFT_BLACK);
  drawLogo();

  // --- SD inicial (robusto) ---
  robustSdInit(); // se você utiliza outro nome, mantenha a sua rotina inicial de SD

  // NeoPixel etc (inalterado)
  pinMode(26, OUTPUT); digitalWrite(26, LOW);
  strip.begin(); strip.show(); strip.setBrightness(brightnessValue);

  // TJpg_Decoder callback
  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(jpgRender);

  // Inicia SD com frequência segura. Tentativas com validação FAT
  SPI.setFrequency(FREQ_SD);
  int tries = 4;
  sdInitialized = false;
  for (int i=1; i<=tries; ++i){
    Serial.printf("SD try %d/%d\n", i, tries);
    if (sdInitAssist(SD_CS_PIN)) {
      if (SD.exists("/")) {
        sdInitialized = true;
        Serial.println("SD montado via sdInitAssist(SD_CS_PIN)");
        break;
      } else {
        Serial.println("SD.begin OK mas sem FAT detectado (aguardando...)");
      }
    } else {
      Serial.println("SD.begin falhou");
    }
    delay(600);
  }
  // Voltar para SPI rápido para TFT
  SPI.setFrequency(40000000);

  // Carregar preferências (wallpaper) mesmo se SD não estiver — será validado ao desenhar
  prefs.begin("app", true);
  String wp = prefs.getString("wallpaper", "");
  prefs.end();
  if (wp.length() > 0) {
    if (wp.charAt(0) != '/') wp = "/" + wp;
    currentWallpaperPath = wp;
    Serial.printf("Wallpaper pref carregada: %s\n", currentWallpaperPath.c_str());
  } else currentWallpaperPath = "";

  // Registrar handler de evento WiFi para iniciar serviços quando pegar IP
  WiFi.mode(WIFI_STA); // garante driver inicializado
  // registrar evento WiFi para mostrar toasts/logs quando pegar IP ou desconectar
  WiFi.onEvent(wifiEventAssist);
  startWifiMonitor();  // cria a tarefa que monitora WiFi e iniciará web quando houver IP

  // Tente conectar utilizando credenciais gravadas (seus helpers)
  if (!wifiAttemptedFromPrefs) {
    wifiAttemptedFromPrefs = true;
    Serial.println("Tentando conectar usando credenciais salvas (tryConnectFromPrefs)...");
    bool wifiOk = tryConnectFromPrefs(10000); // função do seu projeto - tenta por 10s e retorna true/false
    if (wifiOk) {
      Serial.print("WiFi conectado (prefs). IP: ");
      Serial.println(WiFi.localIP());
      // startWebServices() será chamado pelo event handler SYSTEM_EVENT_STA_GOT_IP;
      // em alguns firmwares a conexão já possui IP imediatamente, então chamamos manualmente se necessário:
      if (WiFi.status() == WL_CONNECTED && !webStarted) startWebServices();
    } else {
      Serial.println("tryConnectFromPrefs falhou ou não havia credenciais. Continuando sem WiFi por enquanto.");
      // não iniciar serviços de rede agora; se WiFi conectar depois, event handler acionará startWebServices()
    }
  }

  // Se webStarted ainda não foi executado, podemos carregar o reminder diretamente (não depende de rede)
  // (loadSavedReminder também será chamado por startWebServices se a web for iniciada)
  if (!webStarted) {
    loadSavedReminder();  // garante que reminder esteja pronto antes de desenhar a tela inicial
  }

  // Sincronizar NTP apenas se tivermos WiFi
  bool ntpOk = false;
  if (WiFi.status() == WL_CONNECTED) {
    // 3 tentativas NTP curtas (8s cada)
    for (int i=0;i<3 && !ntpOk;i++){
      Serial.printf("NTP tentativa %d/3\n", i+1);
      ntpOk = syncTimeWithNTP(8000);
      if (!ntpOk) delay(800);
    }
    if (ntpOk) Serial.println("NTP sincronizado antes de ir para HOME.");
    else Serial.println("NTP não sincronizado (depois das tentativas). Prosseguindo sem hora válida.");
  } else {
    Serial.println("Sem WiFi -> pulando NTP por enquanto.");
  }

  // Calibração / UI inicial
  if (loadCalibrationAndMapping()) {
    calibrated = true;
  } else {
    calibrated = false;
    enterState(APP_CALIBRATE, false);
    // mostra a tela de calibração e sai do setup para deixar a UI de calibração ativa
    drawHome(); // se preferir drawCalibration() troque aqui
    return; // sai do setup
  }

  // chegar aqui = calibrado ou calibracao não necessária
  enterState(APP_HOME, false);
  drawHome(); // drawHome usa reminderText carregado por loadSavedReminder()

  if (ntpOk) {
    time_t now = time(NULL);
    struct tm ti; localtime_r(&now, &ti);
    drawHomeTopBarFull(&ti);
  } else {
    // sobrepõe aviso pequeno sem bloquear a UI
    tft.startWrite();
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Sem hora NTP. Configure WiFi.", tft.width()/2, tft.height()-20);
    tft.endWrite();
  }

  // NOTA: não chamamos tryConnectFromPrefs() novamente aqui para evitar duplicações.
  // Se você tiver um comportamento "background" diferente desejado, adicione lógica controlada por flags.
}

void redrawCurrentState(){
  switch(appState){
    case APP_HOME: drawHome(); break;
    case APP_MENU: drawMenu(); break;
    case APP_FILES: drawFiles(); break;
    case APP_FILES_TEXTOS: drawFilesTextos(); break;
    case APP_FILES_IMAGENS: drawFilesImagens(); break;
    case APP_BLUETOOTH: drawBluetooth(); break;
    case APP_LEDS: if (LED_PAGE==0) drawLeds(); else drawLedsPage1(); break;
    case APP_LED_COLOR_SELECT: drawLedColorSelect(); break;
    case APP_SETTINGS: drawSettings(); break;
    case APP_WIFI_PORTAL: startPortal(); break;
    default: drawHome(); break;
  }
}

void enterState(AppState s, bool pushToStack){
  if (s == appState) return;
  if (pushToStack && navTop < (int)sizeof(navStack)/sizeof(navStack[0])) navStack[navTop++] = appState;
  appState = s;
  homeDrawn = menuDrawn = filesDrawn = filesTextosDrawn = filesImagensDrawn = false;
  btDrawn = ledsDrawn = ledColorDrawn = settingsDrawn = wifiPortalDrawn = false;
  lastTouchHandled = millis();
}

void handleBackAction(){
  if (appState == APP_LEDS && LED_PAGE == 1){ drawLeds(); return; }
  if (appState == APP_FILES_TEXTOS && currentFileName.length()>0 && currentFilePage > 0){ currentFilePage--; drawFileContentPage(); return; }
  if (appState == APP_WIFI_PORTAL && portalActive){ server.stop(); WiFi.softAPdisconnect(true); portalActive = false; }

  if (appState == APP_FILES_IMAGENS && currentImageName.length() > 0){ // if viewing image, go back to list
    currentImageName = ""; currentImageIndex = -1; drawFilesImagens(); return;
  }

  if (navTop > 0){ AppState prev = navStack[--navTop]; enterState(prev, false); redrawCurrentState(); return; }
  if (appState != APP_HOME){ enterState(APP_HOME, false); drawHome(); }
}

// File text helpers (unchanged)
void wrapAndAppendLine(const String &line, int maxChars){
  int pos = 0; int len = line.length();
  while (pos < len){ int chunk = min(maxChars, len - pos); String piece = line.substring(pos, pos + chunk);
    if (chunk == maxChars && (pos + chunk) < len){ int lastSpace = piece.lastIndexOf(' '); if (lastSpace > 4){ piece = piece.substring(0, lastSpace); currentFileDisplayLines.push_back(piece); pos += lastSpace + 1; continue; } }
    currentFileDisplayLines.push_back(piece); pos += chunk;
  }
}

void openFileAndPrepareDisplay(const String &fname){
  currentFileLines.clear(); currentFileDisplayLines.clear(); currentFileName = fname; currentFilePage = 0; currentFilePagesTotal = 0;
  if (!sdInitialized) return;
  File f = sdOpenAssist(("/" + fname).c_str());
  if (!f){ tft.startWrite(); tft.fillScreen(TFT_BLACK); drawBackButton(); drawCenteredTitleAvoidButtons("Erro"); tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE); tft.drawString("Nao foi possivel abrir o arquivo", tft.width()/2, tft.height()/2); tft.endWrite(); return; }
  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (line.endsWith("\r")) line.remove(line.length()-1);
    currentFileLines.push_back(line);
  }
  f.close();
  int effectiveWidth = tft.width() - 16; int charWidth = 6 * fileTextSize; if (charWidth <= 0) charWidth = 6; int maxChars = max(10, effectiveWidth / charWidth);
  for (size_t i=0;i<currentFileLines.size();++i) wrapAndAppendLine(currentFileLines[i], maxChars);
  int topY = 48; computeNavRects(); int height = btnBack.y - topY - 12; int lineHeight = (8 * fileTextSize) + 2; int linesPerPage = max(1, height / lineHeight);
  currentFilePagesTotal = (currentFileDisplayLines.size() + linesPerPage - 1) / max(1, linesPerPage);
  drawFileContentPage();
}

void drawFileContentPage(){
  tft.startWrite(); tft.fillScreen(TFT_WHITE);
  drawBackButton(); tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_BLACK);
  int titleY = 20; String title = currentFileName; int maxTitleChars = max(8, (tft.width()-16) / 6); if (title.length() > maxTitleChars) title = title.substring(0, maxTitleChars - 3) + "..."; tft.drawString(title, tft.width()/2, titleY);
  int left = 8; int topY = 36; int width = tft.width() - 16; int height = btnBack.y - topY - 12; tft.setTextDatum(TL_DATUM); tft.setTextSize(fileTextSize); tft.setTextColor(TFT_BLACK);
  int lineHeight = (8 * fileTextSize) + 2; int linesPerPage = max(1, height / lineHeight); int startLine = currentFilePage * linesPerPage; int y = topY + 6;
  for (int i = 0; i < linesPerPage; ++i){ int idx = startLine + i; if (idx >= (int)currentFileDisplayLines.size()) break; String s = currentFileDisplayLines[idx]; tft.drawString(s, left + 6, y); y += lineHeight; }
  if (currentFilePagesTotal > 1) drawNextButton(); else { computeNavRects(); tft.fillRect(btnNext.x, btnNext.y, btnNext.w, btnNext.h, TFT_WHITE); }
  tft.setTextDatum(MC_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_BLACK); char buf[64]; sprintf(buf, "Pagina %d/%d", max(1, currentFilePage + 1), max(1, currentFilePagesTotal)); tft.drawString(buf, tft.width()/2, btnBack.y + btnBack.h/2);
  tft.endWrite();
}

// Main loop
// ===== SUBSTITUIR A FUNÇÃO void loop() EXISTENTE =====
void loop() {
  processToastAssist(tft);
  unsigned long now = millis();

  // Processa o servidor web tanto no portal (AP) quanto quando o server STA foi iniciado.
  if (portalActive || webStarted) {
    server.handleClient();
  }

  // Poll do touch
  if (millis() - lastTouchPoll >= TOUCH_POLL_MS) {
    lastTouchPoll = millis();
    uint16_t rx = 0, ry = 0;
    if (readRawMedian(rx, ry)) {
      Serial.printf("[TOUCH] raw: rx=%u ry=%u\n", rx, ry);
      int sx = 0, sy = 0;
      if (mapRawToScreen(rx, ry, sx, sy)) {
        Serial.printf("[TOUCH] mapped: sx=%d sy=%d  appState=%d\n", sx, sy, (int)appState);
        // Log de áreas importantes (caixas)
        Serial.printf("[BTN] menu:(%d,%d,%d,%d) back:(%d,%d,%d,%d)\n",
          btnMenu.x, btnMenu.y, btnMenu.w, btnMenu.h,
          btnBack.x, btnBack.y, btnBack.w, btnBack.h);

        if (millis() - lastTouchHandled > TOUCH_DEBOUNCE_MS) {
          lastTouchHandled = millis();

          // Back (quando não estiver no HOME)
          if (!(appState == APP_HOME) && inRect(btnBack, sx, sy)) {
            Serial.println("[ACTION] Back pressed");
            handleBackAction();
            goto touch_handled;
          }

          // Next/general
          if (inRect(btnNext, sx, sy)) {
            Serial.println("[ACTION] Next pressed");
            if (appState == APP_LEDS) {
              if (LED_PAGE == 0) drawLedsPage1(); else drawLeds();
            } else if (appState == APP_FILES) {
              listJpgFiles();
              int topY = 48; computeNavRects();
              int bottom = btnBack.y - 12;
              int available = bottom - topY;
              int bh = MENU_BTN_H; int gap = MENU_BTN_GAP;
              int curFilesPerPage = max(1, available / (bh + gap));
              int totalFiles = (int)sdImageFiles.size();
              int totalPages = (totalFiles + curFilesPerPage - 1) / curFilesPerPage;
              if (totalPages > 1 && filesListPage < totalPages - 1) { filesListPage++; drawFilesImagens(); }
            }
            goto touch_handled;
          }

          // --- Tela HOME: botão Menu central
          if (appState == APP_HOME) {
            if (inRect(btnMenu, sx, sy)) {
              Serial.println("[ACTION] Menu pressed (HOME)");
              enterState(APP_MENU);
              drawMenu();
            } else {
              Serial.println("[ACTION] HOME touched but not on Menu");
            }
            goto touch_handled;
          }

          // --- Tela MENU: entradas do menu
          else if (appState == APP_MENU) {
          // Reordered: check Bluetooth earlier to avoid false positives when areas overlap
          if (inRect(btnFiles, sx, sy)) {
            Serial.println("[ACTION] Menu->Files");
            enterState(APP_FILES);
            drawFiles();
          }
          else if (inRect(btnBT, sx, sy)) {
            Serial.println("[ACTION] Menu->Bluetooth");
            enterState(APP_BLUETOOTH);
            drawBluetooth();
          }
          else if (inRect(btnLeds, sx, sy)) {
            Serial.println("[ACTION] Menu->Leds");
            enterState(APP_LEDS);
            drawLeds();
          }
          else if (inRect(btnFilesTextos, sx, sy)) {
            Serial.println("[ACTION] Menu->FilesTextos");
            enterState(APP_FILES_TEXTOS);
            drawFilesTextos();
          }
          else if (inRect(btnFilesImagens, sx, sy)) {
            Serial.println("[ACTION] Menu->FilesImagens");
            enterState(APP_FILES_IMAGENS);
            drawFilesImagens();
          }
          else if (inRect(btnSettings, sx, sy)) {
            Serial.println("[ACTION] Menu->Settings");
            enterState(APP_SETTINGS);
            drawSettings();
          }
          goto touch_handled;
        }

          // --- Tela FILES: navegar entre submenus
          else if (appState == APP_FILES) {
            if (inRect(btnFilesTextos, sx, sy)) { Serial.println("[ACTION] Files->Textos"); enterState(APP_FILES_TEXTOS); drawFilesTextos(); }
            else if (inRect(btnFilesImagens, sx, sy)) { Serial.println("[ACTION] Files->Imagens"); enterState(APP_FILES_IMAGENS); drawFilesImagens(); }
            goto touch_handled;
          }

          // --- Tela BLUETOOTH: toggle local
          else if (appState == APP_BLUETOOTH) {
            if (inRect(btToggleRect, sx, sy)) {
              Serial.println("[ACTION] Bluetooth toggle (touch)");
              // comportamento equivalente ao handler HTTP handleBluetooth (faz persistência)
              btHardwareState = (btHardwareState == 0) ? 1 : 0;
              digitalWrite(26, btHardwareState ? HIGH : LOW);
              prefs.begin("app", false);
              prefs.putBool("bt_state", btHardwareState);
              prefs.end();
              drawBluetooth();
            }
            goto touch_handled;
          }

          // --- Tela LEDS: botões de ligar/desligar e cores
          else if (appState == APP_LEDS) {
                   if (LED_PAGE == 0) {
          // Toggle ligar/desligar
          if (inRect(ledToggleRect, sx, sy)) {
            Serial.println("[ACTION] LED Toggle (touch)");
            ledState = !ledState;
            if (!ledState) currentEffect = EFFECT_NONE;
            applyStripState();
            // redesenha a tela principal de LEDs (permanece na mesma página)
            drawLeds();
            goto touch_handled;
          }

          // Selecionar cor -> entra na tela de seleção de cor
          if (inRect(ledColorRect, sx, sy)) {
            Serial.println("[ACTION] LED -> Color select");
            enterState(APP_LED_COLOR_SELECT);
            drawLedColorSelect();
            goto touch_handled;
          }

          // Troca suave (start loop)
          if (inRect(ledSmoothRect, sx, sy)) {
            Serial.println("[ACTION] LED -> Start smooth loop");
            startSmoothLoop(2500); // 2.5s por transição (ajuste se quiser)
            // opcional: redesenhar a UI principal indicando o efeito
            drawLeds();
            goto touch_handled;
          }

          // Arco-íris (toggle)
          if (inRect(ledRainbowRect, sx, sy)) {
            Serial.println("[ACTION] LED -> Toggle rainbow");
            smoothLoop = false; // interrompe smooth se ativo
            currentEffect = (currentEffect == EFFECT_RAINBOW) ? EFFECT_NONE : EFFECT_RAINBOW;
            if (currentEffect == EFFECT_RAINBOW) rainbowPos = 0;
            goto touch_handled;
          }

        } else { // LED_PAGE == 1 -> Brilho
          // checar os três retângulos de brilho
          for (int i = 0; i < 3; ++i) {
            if (inRect(brightRects[i], sx, sy)) {
              Serial.printf("[ACTION] LED Brightness select %d\n", i);
              brightnessIndex = i;
              if (i == 0) brightnessValue = 50;
              else if (i == 1) brightnessValue = 150;
              else brightnessValue = 255;
              strip.setBrightness(brightnessValue);
              applyStripState();
              drawLedsPage1();
              break;
            }
          }
        }
        goto touch_handled;
      }

      // --- Tela de seleção de cor (tratamento separado)
      else if (appState == APP_LED_COLOR_SELECT) {
        for (int i = 0; i < 6; ++i) {
          if (inRect(colorSquares[i], sx, sy)) {
            Serial.printf("[ACTION] LED Color select idx=%d\n", i);
            if (selectedColorIndex != i) selectedColorIndex = i;
            smoothLoop = false;
            currentEffect = EFFECT_NONE;
            applyStripState();
            drawLedColorSelect();
            break;
          }
        }
        goto touch_handled;
      }
          // --- Tela SETTINGS: diagnóstico e ações (SUBSTITUIR)
          else if (appState == APP_SETTINGS) {
            // debug: print touch coords e rects candidatos
            Serial.printf("[SETTINGS] touch sx=%d sy=%d\n", sx, sy);
            Serial.printf("[SETTINGS-RECTS] filesTextos:(%d,%d,%d,%d) filesImagens:(%d,%d,%d,%d) settings:(%d,%d,%d,%d)\n",
              btnFilesTextos.x, btnFilesTextos.y, btnFilesTextos.w, btnFilesTextos.h,
              btnFilesImagens.x, btnFilesImagens.y, btnFilesImagens.w, btnFilesImagens.h,
              btnSettings.x, btnSettings.y, btnSettings.w, btnSettings.h);

            // Ação imediata mínima: ajustar data/hora quando tocar na área 'btnFilesTextos' (mapeada anteriormente)
            if (inRect(btnFilesTextos, sx, sy)) {
              Serial.println("[ACTION] Settings -> AdjustDateTime (via btnFilesTextos)");
              handleAdjustDateTimeButton();
            }
            // Placeholder para outras áreas que você poderá mapear posteriormente
            else if (inRect(btnFilesImagens, sx, sy)) {
              Serial.println("[ACTION] Settings -> (btnFilesImagens) - placeholder");
              // aqui você pode chamar a função correta quando souber o nome do rect
            }
            else if (inRect(btnSettings, sx, sy)) {
              Serial.println("[ACTION] Settings -> Start Portal");
              if (!portalActive) startPortal();
              enterState(APP_WIFI_PORTAL);
            }
            else {
              Serial.println("[ACTION] Settings -> no matching rect (touch ignored)");
            }
            goto touch_handled;
          }
          // --- Tela FILES_TEXTOS / FILES_IMAGENS / Image viewer handling
          else if (appState == APP_FILES_TEXTOS) {
            if (currentFileName.length() == 0) {
              listTxtFiles();
              int startIdx = filesListPage * filesPerPage;
              for (int i = 0; i < (int)fileEntryRects.size(); ++i) {
                if (inRect(fileEntryRects[i], sx, sy)) {
                  int idx = startIdx + i;
                  if (idx < (int)sdTxtFiles.size()) {
                    String fn = sdTxtFiles[idx];
                    openFileAndPrepareDisplay(fn);
                    goto touch_handled;
                  }
                }
              }
            } else {
              currentFilePage++;
              if (currentFilePage >= max(1, currentFilePagesTotal)) currentFilePage = max(0, currentFilePagesTotal - 1);
              drawFileContentPage();
            }
            goto touch_handled;
          }

          else if (appState == APP_FILES_IMAGENS) {
            if (currentImageName.length() == 0) {
              listJpgFiles();
              int startIdx = filesListPage * filesPerPage;
              for (int i = 0; i < (int)fileEntryRects.size(); ++i) {
                if (inRect(fileEntryRects[i], sx, sy)) {
                  int idx = startIdx + i;
                  if (idx < (int)sdImageFiles.size()) {
                    openImageAndShowByIndex(idx);
                    goto touch_handled;
                  }
                }
              }
            } else {
              if (inRect(topBackRect, sx, sy)) {
                currentImageName = ""; currentImageIndex = -1; drawFilesImagens(); goto touch_handled;
              }
              if (inRect(imgPrevRect, sx, sy)) { int ni = currentImageIndex - 1; if (ni < 0) ni = sdImageFiles.size() - 1; openImageAndShowByIndex(ni); goto touch_handled; }
              if (inRect(imgNextRect, sx, sy)) { int ni = currentImageIndex + 1; if (ni >= (int)sdImageFiles.size()) ni = 0; openImageAndShowByIndex(ni); goto touch_handled; }
              if (inRect(imgWallpaperRect, sx, sy)) { setAsWallpaper(currentImageName); goto touch_handled; }
            }
            goto touch_handled;
          }

        } // fim mapRawToScreen sucesso
        else {
          Serial.printf("[TOUCH] mapRawToScreen() returned false (raw rx=%u ry=%u)\n", rx, ry);
        }
      } // readRawMedian true
    }
  } // touch poll

touch_handled:

  // NTP periodic sync (mantive sua lógica)
  if (millis() - lastNtpSyncMillis >= NTP_SYNC_INTERVAL_MS) {
    if (WiFi.status() != WL_CONNECTED) tryConnectFromPrefs();
    if (WiFi.status() == WL_CONNECTED) {
      syncTimeWithNTP(10000);
    } else {
      lastNtpSyncMillis = millis();
    }
  }

  // TIME UPDATES (HOME) — atualiza mesmo que homeDrawn esteja false
  static unsigned long lastTopUpdate = 0;
  if (appState == APP_HOME) {
    if (millis() - lastTopUpdate >= 250) {
      lastTopUpdate = millis();
      time_t t = time(NULL);
      if (t < 1600000000L) {
        if (lastDayShown != -2) {
          tft.startWrite();
          tft.fillRect(0,0,tft.width(),28,tft.color565(12,12,20));
          tft.setTextDatum(TL_DATUM); tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
          tft.drawString("Sem horario NTP", 8, 8);
          tft.endWrite();
          lastDayShown = -2;
        }
      } else {
        struct tm ti; localtime_r(&t, &ti);
        if (lastDayShown == -1) { drawHomeTopBarFull(&ti); }
        else {
          if (ti.tm_mday != lastDayShown || ti.tm_hour != lastHourShown || ti.tm_min != lastMinShown) {
            updateHomeTopBarMinuteHourDay(&ti);
          }
          if (ti.tm_sec != lastSecShown) {
            redrawHomeTime(&ti);
            lastSecShown = ti.tm_sec;
          }
        }
      }
    }
  }

  updateEffects();

  static unsigned long lastWifiTry = 0;
  if (!portalActive && !wifiConnected && millis() - lastWifiTry > 10000) {
    tryConnectFromPrefs();
    lastWifiTry = millis();
    if (WiFi.status() == WL_CONNECTED) wifiConnected = true;
  }

  // pequena pausa pra dar chance a tarefas do RTOS
  delay(6);
}



// Escapa texto para inserir dentro de <textarea> / HTML (substitui & < > " ' )
String htmlEscape(const String &s){
  String out = "";
  for (size_t i=0;i<s.length();++i){
    char c = s.charAt(i);
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '"') out += "&quot;";
    else if (c == '\'') out += "&#39;";
    else out += c;
  }
  return out;
}

// Small JSON string escaper
static String jsEscape(const String &s){
  String out = "\"";
  for(size_t i=0;i<s.length();++i){
    char c = s.charAt(i);
    if (c == '"' ) out += "\\\"";
    else if (c == '\\') out += "\\\\";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else out += c;
  }
  out += "\"";
  return out;
}

// Ensure /images folder exists
static bool ensureImagesFolder(){
  if (!sdInitialized) return false;
  if (!xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000))) return false;
  bool ok = true;
  if (!SD.exists("/images")) {
    ok = SD.mkdir("/images");
  }
  xSemaphoreGive(sdMutex);
  return ok;
}

// Helpers: list files in a directory (optionally filter by ext)
static String listFilesJson(const char *folder, const char *extFilter = nullptr){
  String out = "[";
  if (!sdInitialized) return out + "]";
  if (!xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000))) return out + "]";
  File dir = SD.open(folder);
  if (!dir){
    xSemaphoreGive(sdMutex);
    return out + "]";
  }
  File entry;
  bool first=true;
  while ((entry = dir.openNextFile())) {
    if (!entry.isDirectory()) {
      String name = String(entry.name());
      String lower = name; lower.toLowerCase();
      if (!extFilter || lower.endsWith(extFilter)) {
        if (!first) out += ",";
        out += jsEscape(name);
        first = false;
      }
    }
    entry.close();
  }
  dir.close();
  xSemaphoreGive(sdMutex);
  out += "]";
  return out;
}

// Handler: serve HTML UI (simple)
static const char webpage_html[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="pt-BR">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <meta http-equiv="Content-Language" content="pt-BR">
  <title>Hub - Device</title>
  <style>
    :root{
      --bg:#0f1720;
      --card:#0f1a26;
      --muted:#9aa6b2;
      --accent:#4fc3f7;
      --accent-2:#60a5fa;
      --success:#7ee787;
      --danger:#ff7b7b;
      --glass: rgba(255,255,255,0.03);
      --card-radius:10px;
    }
    html,body{height:100%;margin:0;background:linear-gradient(180deg,var(--bg), #071021);font-family:system-ui,-apple-system,'Segoe UI',Roboto,Helvetica,Arial;}
    .wrap{max-width:980px;margin:18px auto;padding:18px;}
    header{display:flex;align-items:center;justify-content:space-between;color:var(--muted);margin-bottom:12px}
    header h1{color:white;font-size:18px;margin:0}
    .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:12px}
    .card{background:var(--card);padding:12px;border-radius:var(--card-radius);box-shadow:0 6px 18px rgba(0,0,0,0.5);color:var(--muted)}
    .card h3{margin:0 0 8px 0;color:white;font-size:14px}
    button, select, input[type="text"], input[type="datetime-local"]{background:var(--glass);border:1px solid rgba(255,255,255,0.04);padding:8px;border-radius:8px;color:white}
    .row{display:flex;gap:8px;flex-wrap:wrap}
    .muted{color:var(--muted);font-size:13px}
    .accent{color:var(--accent)}
    a.link{color:var(--accent-2);text-decoration:none}
    .small{font-size:12px;color:var(--muted)}
    .filelist{max-height:220px;overflow:auto;padding-top:6px}
    .fileitem{display:flex;justify-content:space-between;align-items:center;padding:6px;border-radius:6px;background:rgba(255,255,255,0.01);margin-bottom:6px}
    .controls{display:flex;flex-direction:column;gap:8px}
    label.small{display:block;font-size:12px;color:var(--muted);margin-top:6px}
  </style>
</head>
<body>
<div class="wrap">
  <header>
    <h1>Hub do dispositivo</h1>
    <div class="muted small" id="serverInfo">conectando...</div>
  </header>

  <div class="grid">
    <!-- Quick controls: LEDs -->
    <div class="card">
      <h3>Controles rápidos - LEDs</h3>
      <div class="controls">
        <div class="row">
          <button onclick="toggleLed()">Ligar / Desligar</button>
          <button onclick="setEffect('smooth')">Suave</button>
          <button onclick="setEffect('rainbow')">Arco-íris</button>
        </div>
        <div class="row">
          <select id="colorIdx">
            <option value="0">Vermelho</option>
            <option value="1">Amarelo</option>
            <option value="2">Magenta</option>
            <option value="3">Azul</option>
            <option value="4">Verde</option>
            <option value="5">Branco</option>
          </select>
          <button onclick="setColor()">Aplicar cor</button>
        </div>
        <div class="row">
          <label class="small">Brilho</label>
          <input id="bright" type="range" min="10" max="255" value="150" oninput="document.getElementById('brval').innerText=this.value">
          <span id="brval" class="small">150</span>
          <button onclick="setBright()">Aplicar</button>
        </div>
        <div class="small" id="ledState">LEDs: —</div>
      </div>
    </div>

    <!-- Quick controls: Bluetooth -->
    <div class="card">
      <h3>Controles rápidos - Bluetooth</h3>
      <div class="controls">
        <div class="row">
          <button onclick="toggleBT()">Liga / Desliga Bluetooth</button>
        </div>
        <div class="small" id="btstate">Bluetooth: —</div>
      </div>
    </div>

    <!-- Textos no SD -->
    <div class="card">
      <h3>Textos (SD)</h3>
      <div class="row">
        <input id="newtxtname" type="text" placeholder="novo-arquivo.txt">
        <button onclick="createText()">Criar</button>
      </div>
      <div class="filelist" id="textlist">carregando...</div>
    </div>

    <!-- Imagens no SD -->
    <div class="card">
      <h3>Imagens (SD)</h3>
      <div class="row">
        <input id="imgfile" type="file" accept=".jpg,.jpeg">
        <button onclick="uploadImage()">Enviar</button>
      </div>
      <div class="filelist" id="imagelist">carregando...</div>
    </div>

    <!-- Configurações e Ajuste de hora -->
    <div class="card">
      <h3>Configurações</h3>
      <div class="controls">
        <div class="row">
          <button onclick="adjustDateTime()">Ajustar data e hora (NTP)</button>
          <button onclick="openWifiPortal()">Configurar WiFi</button>
        </div>
        <div class="small">Remova ajuste manual de segundos (feito)</div>
      </div>
    </div>

    <!-- Lembrete com agendamento -->
    <div class="card">
      <h3>Lembrete</h3>
      <div class="controls">
        <label class="small">Texto do lembrete</label>
        <input id="reminderText" type="text" placeholder="Ex.: Ligar café">
        <label class="small">Início</label>
        <input id="reminderStart" type="datetime-local">
        <label class="small">Fim</label>
        <input id="reminderEnd" type="datetime-local">
        <div class="row">
          <button onclick="setReminder(false)">Salvar (ativo enquanto durar)</button>
          <button onclick="setReminder(true)">Programar (usar horário)</button>
          <button onclick="clearReminder()">Remover</button>
        </div>
        <div class="small" id="remInfo">—</div>
      </div>
    </div>

  </div>
</div>

<script>
const fetchJson = u => fetch(u).then(r => r.ok ? r.json() : Promise.reject('err'));
let serverStart = null;
let lastSdV = null;

function loadLedState(){
  fetchJson('/status').then(js=>{
    document.getElementById('ledState').innerText = 'LEDs: ' + (js.led? 'Ligados':'Desligados') + ' Cor:' + js.color + ' Br:' + js.brightness;
    document.getElementById('brval').innerText = js.brightness;
    document.getElementById('colorIdx').value = js.color;
  }).catch(()=>{});
}
function loadBtState(){
  fetchJson('/status').then(js=>{
    document.getElementById('btstate').innerText = 'Bluetooth: ' + (js.bt? 'Ligado':'Desligado');
  }).catch(()=>{});
}

function toggleLed(){ fetch('/led', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'action=toggle'}).then(()=>loadLedState()); }
function setEffect(e){ fetch('/led', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'effect='+encodeURIComponent(e)}).then(()=>loadLedState()); }
function setColor(){ const c = document.getElementById('colorIdx').value; fetch('/led', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'color='+encodeURIComponent(c)}).then(()=>loadLedState()); }
function setBright(){ const b = document.getElementById('bright').value; fetch('/led', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'brightness='+encodeURIComponent(b)}).then(()=>loadLedState()); }

function toggleBT(){ fetch('/bluetooth', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'action=toggle'}).then(()=>loadBtState()); }

function loadTextList(){
  fetchJson('/list-texts').then(list=>{
    let out = '';
    list.forEach(f => {
      out += '<div class="fileitem"><div>'+f+'</div><div><a class="link" href="/view-text?name='+encodeURIComponent(f)+'">Abrir</a> &nbsp; <button onclick="delTxt('+JSON.stringify(f)+')">Excluir</button></div></div>';
    });
    document.getElementById('textlist').innerHTML = out || '<div class="small muted">nenhum arquivo</div>';
  }).catch(e=>{ document.getElementById('textlist').innerText='erro'; });
}

function createText(){
  const name = document.getElementById('newtxtname').value.trim();
  if(!name) { alert('Informe nome do arquivo'); return; }
  const body = 'name='+encodeURIComponent(name)+'&content='+encodeURIComponent('');
  fetch('/save-text',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body}).then(r=>r.text()).then(t=>{
    alert(t);
    loadTextList();
  });
}

function delTxt(name){
  if(!confirm('Excluir '+name+'?')) return;
  fetch('/delete-text',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'name='+encodeURIComponent(name)})
  .then(()=>{ loadTextList(); });
}

function loadImageList(){
  fetchJson('/list-images').then(list=>{
    let out = '';
    list.forEach(f => {
      out += '<div class="fileitem"><div>'+f+'</div><div><a class="link" href="/download-image?name='+encodeURIComponent(f)+'" target="_blank">Exibir</a> &nbsp; <button onclick="delImg('+JSON.stringify(f)+')">Excluir</button></div></div>';
    });
    document.getElementById('imagelist').innerHTML = out || '<div class="small muted">nenhuma imagem</div>';
  }).catch(()=>{ document.getElementById('imagelist').innerText='erro'; });
}

function uploadImage(){
  const fi = document.getElementById('imgfile');
  if(!fi.files.length){ alert('Escolha um arquivo'); return; }
  const fd = new FormData();
  fd.append('image', fi.files[0]);
  fetch('/upload-image', { method:'POST', body: fd }).then(r=>r.text()).then(txt=>{ alert(txt); loadImageList(); });
}
function delImg(name){
  if(!confirm('Excluir '+name+'?')) return;
  fetch('/delete-image',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'name='+encodeURIComponent(name)}).then(()=>loadImageList());
}

/* time / wifi */
function adjustDateTime(){ fetch('/set-time',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'ntp=1'}).then(r=>r.text()).then(t=>alert(t)); }
function openWifiPortal(){ alert('Abra o portal WiFi via tela do dispositivo.'); }

/* reminder */
function setReminder(programmed){
  const text = document.getElementById('reminderText').value || '';
  let start = 0, end = 0;
  if(programmed){
    const s = document.getElementById('reminderStart').value;
    const e = document.getElementById('reminderEnd').value;
    if(s) start = Math.floor(new Date(s).getTime()/1000);
    if(e) end = Math.floor(new Date(e).getTime()/1000);
  }
  const body = 'reminder='+encodeURIComponent(text)+'&start='+encodeURIComponent(start)+'&end='+encodeURIComponent(end);
  fetch('/set-reminder',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body}).then(()=>{ alert('Salvo'); });
}
function clearReminder(){
  fetch('/set-reminder',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'reminder='}).then(()=>{ alert('Removido'); });
}

/* status polling + auto-refresh lists on SD changes / restart */
async function pollStatus(){
  try{
    const st = await fetchJson('/status');
    if(serverStart === null) serverStart = st.start;
    document.getElementById('serverInfo').innerText = 'uptime: ' + Math.floor((Date.now() - st.start)/1000) + 's';
    // restart detection
    if(st.start != serverStart){ location.reload(true); return; }
    // sd changes detection
    if(typeof st.sdv !== 'undefined'){
      if(lastSdV === null) lastSdV = st.sdv;
      else if(st.sdv != lastSdV){
        lastSdV = st.sdv;
        // recarregar listas
        loadTextList(); loadImageList();
      }
    }
  }catch(e){}
}

/* init */
window.addEventListener('load', ()=>{
  loadTextList(); loadImageList();
  loadLedState(); loadBtState();
  pollStatus();
  setInterval(pollStatus, 4000);
  setInterval(loadLedState, 5000);
  setInterval(loadBtState, 7000);
});
</script>
</body>
</html>
)rawliteral";

// Handler: serve page
void handleWebUI(){
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  // Força charset UTF-8 para evitar caracteres estranhos (acentos PT-BR)
  server.sendHeader("Content-Type", "text/html; charset=utf-8");
  server.send(200, "text/html; charset=utf-8", String(webpage_html));
}

// Handler: status (JSON)
void handleStatus(){
  String s = "{";
  s += "\"led\":" + String(ledState ? "true":"false") + ",";
  s += "\"color\":" + String(selectedColorIndex) + ",";
  s += "\"brightness\":" + String(brightnessValue) + ",";
  s += "\"bt\":" + String((btHardwareState)? "true":"false") + ",";
  s += "\"wallpaper\":\"" + jsEscape(currentWallpaperPath) + "\",";
  s += "\"start\":" + String(serverStartMillis) + ",";
  s += "\"sdv\":" + String(sdChangeCounter) + ",";
  s += "\"statev\":" + String(serverStateVersion);
  s += "}";
  server.sendHeader("Content-Type", "application/json; charset=utf-8");
  server.send(200, "application/json; charset=utf-8", s);
}

// LEDs control
void handleLed(){
  if (server.method() != HTTP_POST){
    server.send(405,"text/plain","Use POST");
    return;
  }
  String action = server.arg("action");
  String color = server.arg("color");
  String brightness = server.arg("brightness");
  String effect = server.arg("effect");
  if (action == "toggle"){
    ledState = !ledState;
    if (!ledState) currentEffect = EFFECT_NONE;
    applyStripState();
  }
  if (color.length()){
    int idx = color.toInt();
    selectedColorIndex = clampi(idx, 0, 5);
    currentEffect = EFFECT_NONE;
    applyStripState();
  }
  if (brightness.length()){
    int b = clampi(brightness.toInt(), 0, 255);
    brightnessValue = b;
    strip.setBrightness(brightnessValue);
    applyStripState();
  }
  if (effect.length()){
    if (effect == "rainbow") { currentEffect = EFFECT_RAINBOW; rainbowPos = 0; }
    else if (effect == "smooth") { startSmoothLoop(2500); }
    else { currentEffect = EFFECT_NONE; }
  }
  handleStatus();
}

// Bluetooth toggle
void handleBluetooth(){
  if (server.method() != HTTP_POST){ server.send(405,"text/plain","Use POST"); return; }
  String action = server.arg("action");
  if (action == "toggle"){
    btHardwareState = (btHardwareState == 0) ? 1 : 0;
    digitalWrite(26, btHardwareState ? HIGH : LOW);
    prefs.begin("app", false);
    prefs.putBool("bt_state", btHardwareState);
    prefs.end();
  }
  server.send(200,"text/plain", btHardwareState ? "1":"0");
}

// TEXT file handlers
void handleListTexts(){
  String out = listFilesJson("/", ".txt");
  server.send(200, "application/json", out);
}

// Handler: view/edit text file via web page
void handleViewText(){
  if (!server.hasArg("name")) { server.send(400,"text/plain","missing name"); return; }
  String name = server.arg("name");
  if (name.charAt(0) == '/') name = name.substring(1);
  String path = "/" + name;
  if (!sdInitialized){ server.send(500,"text/plain","SD not mounted"); return; }
  if (!xSemaphoreTake(sdMutex, pdMS_TO_TICKS(3000))) { server.send(503,"text/plain","SD busy"); return; }
  if (!SD.exists(path.c_str())) { xSemaphoreGive(sdMutex); server.send(404,"text/plain","Not found"); return; }
  File f = SD.open(path.c_str(), FILE_READ);
  if (!f){ xSemaphoreGive(sdMutex); server.send(500,"text/plain","open failed"); return; }
  String content = "";
  while (f.available()) content += (char)f.read();
  f.close();
  xSemaphoreGive(sdMutex);

  String esc = htmlEscape(content);
  String qname = jsEscape(name); // jsEscape já existe no seu arquivo e devolve a string entre aspas

  String page = "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'><title>Editar "+name+"</title>";
  page += "<style>body{font-family:Arial;background:#071426;color:#eaf6ff;padding:12px}textarea{width:100%;height:60vh;background:#021821;color:#eaf6ff;border-radius:8px;padding:8px;border:none}input,button{padding:8px;border-radius:6px} .top{display:flex;gap:8px;align-items:center} .link{color:#10b981;text-decoration:none}</style></head><body>";
  page += "<div class='top'><a class='link' href='/'>&larr; Voltar</a><h2 style='margin-left:8px'>"+name+"</h2></div>";
  page += "<div><textarea id='txtcontent'>";
  page += esc;
  page += "</textarea></div>";
  page += "<div style='margin-top:8px'><button onclick='save()' class='btn'>Salvar</button> <button onclick='del()' class='btn'>Excluir</button></div>";
  page += "<script>";
  page += "function save(){ const content = document.getElementById('txtcontent').value; fetch('/save-text', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'name='+encodeURIComponent(" + qname + ") + '&content='+encodeURIComponent(content)}).then(()=>{ alert('Salvo'); location.href='/'; }); }";
  page += "function del(){ if(!confirm('Excluir '+ " + qname + " + ' ?')) return; fetch('/delete-text', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'name='+encodeURIComponent(" + qname + ")}).then(()=>{ alert('Excluído'); location.href='/'; }); }";
  page += "</script></body></html>";

  server.send(200, "text/html", page);
}

void handleReadText(){
  if (!server.hasArg("name")) { server.send(400,"text/plain","missing name"); return; }
  String name = server.arg("name");
  // sanitize: remove leading '/'
  if (name.charAt(0) == '/') name = name.substring(1);
  String path = "/" + name;
  if (!sdInitialized) { server.send(500,"text/plain","SD not mounted"); return; }
  if (!xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000))) { server.send(503,"text/plain","SD busy"); return; }
  File f = SD.open(path.c_str(), FILE_READ);
  if (!f){ xSemaphoreGive(sdMutex); server.send(404,"text/plain","Not found"); return; }
  String content = "";
  while (f.available()) content += (char)f.read();
  f.close();
  xSemaphoreGive(sdMutex);
  server.send(200, "text/plain", content);
}

void handleSaveText(){
  if (server.method() != HTTP_POST){ server.send(405,"text/plain","Use POST"); return; }
  String name = server.arg("name");
  String content = server.arg("content");
  if (name.length() == 0){ server.send(400,"text/plain","missing name"); return; }
  if (name.charAt(0) == '/') name = name.substring(1);
  String path = "/" + name;
  if (!sdInitialized) { server.send(500,"text/plain","SD not mounted"); return; }
  if (!xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000))) { server.send(503,"text/plain","SD busy"); return; }

  // overwrite safely
  if (SD.exists(path.c_str())) {
    SD.remove(path.c_str());
  }

  File f = SD.open(path.c_str(), FILE_WRITE);
  if (!f){ xSemaphoreGive(sdMutex); server.send(500,"text/plain","open failed"); return; }
  f.print(content);
  f.close();
  xSemaphoreGive(sdMutex);

  // notify web clients
  sdChangeCounter++;
  serverStateVersion++;

  server.send(200,"text/plain","OK");
}

void handleDeleteText(){
  if (server.method() != HTTP_POST){ server.send(405,"text/plain","Use POST"); return; }
  String name = server.arg("name");
  if (name.length() == 0){ server.send(400,"text/plain","missing name"); return; }
  if (name.charAt(0) == '/') name = name.substring(1);
  String path = "/" + name;
  if (!sdInitialized){ server.send(500,"text/plain","SD not mounted"); return; }
  if (!xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000))) { server.send(503,"text/plain","SD busy"); return; }
  bool ok = SD.remove(path.c_str());
  xSemaphoreGive(sdMutex);

  if (ok) {
    sdChangeCounter++;
    serverStateVersion++;
  }
  server.send(ok?200:500,"text/plain", ok? "OK":"Fail");
}

// IMAGES handlers
void handleListImages(){
  // ensure folder exists listing /images
  if (!ensureImagesFolder()){ server.send(200,"application/json","[]"); return; }
  String out = listFilesJson("/images", ".jpg");
  // include .jpeg too: quick append of .jpeg entries
  // (listFilesJson only filters by single extension; for brevity we return jpg only - your images root can include .jpeg as .jpg)
  server.send(200,"application/json", out);
}

void handleDownloadImage(){
  if (!server.hasArg("name")) { server.send(400,"text/plain","missing name"); return; }
  String name = server.arg("name");
  String path = "/images/" + name;
  if (!sdInitialized){ server.send(500,"text/plain","SD not mounted"); return; }
  if (!xSemaphoreTake(sdMutex, pdMS_TO_TICKS(3000))) { server.send(503,"text/plain","SD busy"); return; }
  if (!SD.exists(path.c_str())) { xSemaphoreGive(sdMutex); server.send(404,"text/plain","Not found"); return; }
  File f = SD.open(path.c_str(), FILE_READ);
  if (!f){ xSemaphoreGive(sdMutex); server.send(500,"text/plain","open failed"); return; }
  server.streamFile(f, "image/jpeg");
  f.close();
  xSemaphoreGive(sdMutex);
}

// Upload image (multipart)
void handleImageUploadMultipart(){
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    // sanitize
    filename.replace("/",""); filename.replace("\\","");
    if (filename.length() == 0) return;
    ensureImagesFolder();
    String path = "/images/" + filename;
    if (!xSemaphoreTake(sdMutex, pdMS_TO_TICKS(3000))) {
      webUploadOpen = false; return;
    }
    webUploadFile = SD.open(path.c_str(), FILE_WRITE);
    webUploadOpen = (bool)webUploadFile;
    if (!webUploadOpen) { xSemaphoreGive(sdMutex); }
  } else if (upload.status == UPLOAD_FILE_WRITE){
    if (webUploadOpen) {
      webUploadFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END){
    if (webUploadOpen) {
      webUploadFile.close();
      xSemaphoreGive(sdMutex);
      webUploadOpen = false;
      // notify web clients
      sdChangeCounter++;
      serverStateVersion++;
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED){
    if (webUploadOpen){
      webUploadFile.close();
      xSemaphoreGive(sdMutex);
      webUploadOpen = false;
    }
  }
}

// Delete image
void handleDeleteImage(){
  if (server.method() != HTTP_POST){ server.send(405,"text/plain","Use POST"); return; }
  String name = server.arg("name");
  if (name.length() == 0) { server.send(400,"text/plain","missing name"); return; }
  String path = "/images/" + name;
  if (!sdInitialized){ server.send(500,"text/plain","SD not mounted"); return; }
  if (!xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000))) { server.send(503,"text/plain","SD busy"); return; }
  bool ok = SD.remove(path.c_str());
  xSemaphoreGive(sdMutex);

  if (ok) {
    sdChangeCounter++;
    serverStateVersion++;
  }

  server.send(ok?200:500,"text/plain", ok? "OK":"Fail");
}

// Time handlers: ntp or epoch set
void handleSetTime(){
  if (server.method() != HTTP_POST){ server.send(405,"text/plain","Use POST"); return; }
  if (server.hasArg("ntp") && server.arg("ntp") == "1"){
    bool ok = syncTimeWithNTP(10000);
    server.send(200, "text/plain", ok? "NTP OK":"NTP falhou");
    return;
  }
  if (server.hasArg("epoch")){
    long e = server.arg("epoch").toInt();
    struct timeval tv;
    tv.tv_sec = e;
    tv.tv_usec = 0;
    if (settimeofday(&tv, NULL) == 0){
      server.send(200,"text/plain","Hora definida");
    } else server.send(500,"text/plain","Falha settimeofday");
    return;
  }
  server.send(400,"text/plain","missing ntp or epoch");
}

// Reminder handler: saves to prefs and redraws home
void handleSetReminder(){
  if (server.method() != HTTP_POST){ server.send(405,"text/plain","Use POST"); return; }
  String r = server.arg("reminder");
  String sstart = server.arg("start");
  String send = server.arg("end");
  unsigned long startEpoch = 0;
  unsigned long endEpoch = 0;
  if (sstart.length()) startEpoch = (unsigned long) sstart.toInt();
  if (send.length())   endEpoch   = (unsigned long) send.toInt();

  prefs.begin("app", false);
  prefs.putString("reminder", r);
  prefs.putString("reminder_start", String(startEpoch));
  prefs.putString("reminder_end", String(endEpoch));
  prefs.end();

  reminderText = r;
  reminderStartEpoch = startEpoch;
  reminderEndEpoch = endEpoch;

  // redraw home on screen thread-safe
  if (xSemaphoreTake(tftMutex, pdMS_TO_TICKS(200))){
    drawHome();
    xSemaphoreGive(tftMutex);
  }

  // bump server state so clients know something mudou (opcional)
  serverStateVersion++;

  server.send(200,"text/plain","OK");
}

// Handler de debug: retorna info do lembrete e hora atual
void handleReminderStatus(){
  time_t now = time(NULL);
  String s = "{";
  s += "\"now\":" + String((unsigned long)now) + ",";
  s += "\"reminder\":\"" + jsEscape(reminderText) + "\",";
  s += "\"start\":" + String(reminderStartEpoch) + ",";
  s += "\"end\":" + String(reminderEndEpoch);
  s += "}";
  server.sendHeader("Content-Type", "application/json; charset=utf-8");
  server.send(200, "application/json; charset=utf-8", s);
}

// Substituir initWebServer() por este bloco (adiciona /view-text e seta serverStartMillis)
void initWebServer(){
  server.on("/", HTTP_GET, handleWebUI);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/led", HTTP_POST, handleLed);
  server.on("/bluetooth", HTTP_POST, handleBluetooth);

  // text files
  server.on("/list-texts", HTTP_GET, handleListTexts);
  server.on("/read-text", HTTP_GET, handleReadText);
  server.on("/save-text", HTTP_POST, handleSaveText);
  server.on("/delete-text", HTTP_POST, handleDeleteText);
  server.on("/view-text", HTTP_GET, handleViewText); // nova rota para visualização/edição

  // images
  server.on("/list-images", HTTP_GET, handleListImages);
  server.on("/download-image", HTTP_GET, handleDownloadImage);
  server.on("/delete-image", HTTP_POST, handleDeleteImage);
  // upload with multipart: finalizer + upload handler
  server.on("/upload-image", HTTP_POST,
    [](){ server.send(200, "text/plain", "Upload recebido"); },
    handleImageUploadMultipart
  );

  // time & reminder
  server.on("/set-time", HTTP_POST, handleSetTime);
  server.on("/set-reminder", HTTP_POST, handleSetReminder);

  server.on("/reminder-status", HTTP_GET, handleReminderStatus);

  server.begin();

  // marca quando o servidor subiu (usado pelo cliente para detectar reinício)
  serverStartMillis = millis();
}


// Load saved reminder at boot (call in setup after prefs begin)
void loadSavedReminder(){
  prefs.begin("app", true);
  reminderText = prefs.getString("reminder", "");
  String sstart = prefs.getString("reminder_start", "0");
  String send   = prefs.getString("reminder_end", "0");
  prefs.end();
  reminderStartEpoch = (unsigned long) sstart.toInt();
  reminderEndEpoch   = (unsigned long) send.toInt();
}


// ------------------------ END: Web UI module ------------------------1