/*======================================================================
 *
 *      ESP8266  HAND CONTROLLER  --  THE WEARABLE TRANSMITTER
 *      ==================================================
 *
 *      >>> THIS FILE GOES ON THE ESP8266 YOU WEAR ON YOUR HAND. <<<
 *      >>> The other file, ESP32_Humanoid_Robot_RECEIVER.ino,   <<<
 *      >>> goes on the ESP32 inside the robot.                  <<<
 *
 *      Board to select in the IDE:  NodeMCU 1.0 (ESP-12E) , or
 *                                   LOLIN(WEMOS) D1 mini
 *      Libraries needed:            none beyond the ESP8266 core
 *
 *----------------------------------------------------------------------
 *  WHAT DO I HAVE TO CONFIGURE?
 *----------------------------------------------------------------------
 *  Nothing, as long as you did not change the robot's hotspot.
 *
 *      SSID / password   already filled in below, copied from the robot
 *                        sketch ("Humanoid" / "12345678"). If you ever
 *                        change AP_SSID or AP_PASS in the robot sketch,
 *                        change the two lines below to match. That is
 *                        the ONE setting, and it is already done.
 *      IP address        NOT needed. The glove uses WiFi.gatewayIP(),
 *                        which IS the robot (192.168.4.1). If you ever
 *                        change the robot's AP address it still works.
 *      Pairing           none. Power it on, it joins, it starts sending.
 *
 *  So: flash it, wear it, switch it on. No app, no pairing, no internet.
 *
 *----------------------------------------------------------------------
 *  WIRING  --  THE GLOVE (TRANSMITTER)
 *----------------------------------------------------------------------
 *  The ESP8266 has only ONE analog pin (A0), so the three flex sensors
 *  go through a CD4051 / 74HC4051 8-channel analog multiplexer.
 *
 *    CD4051 pin      goes to
 *    ----------      -------------------------------------------------
 *    VDD  (16)       3.3 V
 *    VSS  (8)        GND
 *    VEE  (7)        GND        <-- must be GND, not left floating
 *    INH  (6)        GND        <-- enable, active low
 *    S0   (11)       D5  (GPIO14)
 *    S1   (10)       D6  (GPIO12)
 *    S2   (9)        D7  (GPIO13)
 *    COM/Z (3)       A0 on the ESP8266
 *    Y0   (13)       flex sensor 1 sensing node   -> joint 1
 *    Y1   (14)       flex sensor 2 sensing node   -> joint 2
 *    Y2   (15)       flex sensor 3 sensing node   -> joint 3
 *    Y3              leave unconnected
 *    Y4   (pin 1)     RIGHT/LEFT push button input
 *    Y5..Y7           leave unconnected
 *
 *  EACH FLEX SENSOR IS A VOLTAGE DIVIDER. One 47k resistor per sensor:
 *
 *      3.3 V ----[ FLEX SENSOR ]----+----[ 47k ]---- GND
 *                                   |
 *                                   +--> CD4051 Yn   (the sensing node)
 *
 *    Resistor value:  47 kOhm, 1/4 W, one per sensor.
 *                     A typical 2.2in flex sensor is about 25k straight
 *                     and 45k-125k bent, so 47k puts the swing right in
 *                     the middle of the ADC range. If your swing looks
 *                     small on the robot's web page, try 22k (for low
 *                     resistance sensors) or 100k (for high ones).
 *    Bending it makes its resistance RISE, so the reading FALLS. That is
 *    fine -- calibration on the web page handles either direction.
 *
 *  HOW MANY WIRES?  Count them once and it stops being confusing.
 *
 *    Per flex sensor: the sensor has 2 legs.
 *        leg A  -> 3.3 V rail                                (1 wire)
 *        leg B  -> the sensing node                          (1 wire)
 *        sensing node -> one leg of the 47k                  (junction)
 *        other leg of the 47k -> GND rail                    (1 wire)
 *        sensing node -> CD4051 Yn                           (1 wire)
 *      = 2 wires from the sensor, 1 wire from that node into the
 *        CD4051, plus the resistor to GND. Three sensors, so THREE
 *        wires total from the sensors into the CD4051 (Y0, Y1, Y2),
 *        and they share the one 3.3 V rail and the one GND rail.
 *
 *    CD4051 -> ESP8266: SIX wires.
 *        COM/Z -> A0,  S0 -> D5,  S1 -> D6,  S2 -> D7,
 *        VDD -> 3.3 V, VSS -> GND
 *        (INH and VEE also go to GND, but to the GND rail, so if you
 *        count those as their own wires it is eight.)
 *
 *----------------------------------------------------------------------
 *  THE RIGHT / LEFT ARM PUSH BUTTON (THROUGH CD4051 Y4)
 *----------------------------------------------------------------------
 *
 *        3.3V ----[10k resistor]----+---- CD4051 Y4 (pin 1)
 *                                   |
 *                               PUSH BUTTON
 *                                   |
 *                                  GND
 *
 *    Released = high A0 reading. Pressed = low A0 reading.
 *    Every press flips between RIGHT and LEFT arms.
 *
 *  POWER FOR THE GLOVE
 *    Easiest:   USB power bank into the ESP8266 micro-USB socket.
 *    Battery:   1S LiPo 3.7 V into a TP4056 charger + a 3.3 V regulator
 *               (or an MT3608 to 5 V into VIN). Do NOT put 3.7 V on the
 *               3V3 pin through nothing -- use VIN/5V with a regulator,
 *               or a board with a battery input.
 *    3.3 V and GND for the sensors and the CD4051 come from the ESP8266
 *               3V3 and G pins. The whole glove draws well under 100 mA.
 *
 *    !! Sensors MUST be fed from the SAME 3.3 V the ESP8266 ADC uses,
 *       or the readings drift with the supply.
 *
 *  STATUS LED (the small blue one on the board, D4 / GPIO2)
 *    fast blink  = looking for the robot's hotspot
 *    slow blink  = connected, but Flex Control is OFF on the website
 *    solid on    = connected and Flex Control is ON: you are driving it
 *
 *  The arm the switch has selected is shown on the robot's web page and
 *  printed on this board's serial monitor, so you can always tell.
 *
 *----------------------------------------------------------------------
 *  WIRING  --  THE ROBOT (RECEIVER), unchanged from your existing build
 *----------------------------------------------------------------------
 *    ESP32 3V3/GND  -> PCA9685 VCC/GND        (logic only)
 *    ESP32 GPIO 21  -> PCA9685 SDA
 *    ESP32 GPIO 22  -> PCA9685 SCL
 *    PCA9685 V+     -> 6 V servo supply +     (5-6 V, 10 A or more)
 *    PCA9685 GND    -> servo supply -  AND  ESP32 GND  (common ground,
 *                      this is not optional)
 *    PCA9685 ch0-15 -> the 16 body servos
 *    ESP32 GPIO 25  -> head servo signal
 *    OLED           -> SDA 32, SCL 33, 3V3, GND
 *    MAX98357A      -> DIN 14, BCLK 26, LRC 27, 5 V, GND
 *    Wireless link  -> none to wire: the glove joins the ESP32 hotspot
 *
 *----------------------------------------------------------------------
 *  WHICH SENSOR DRIVES WHICH SERVO  (defaults -- change on the website)
 *----------------------------------------------------------------------
 *   The SAME three sensors drive whichever arm the switch selects.
 *
 *    Flex  CD4051   switch = RIGHT         switch = LEFT
 *    ----  ------   ------------------     ------------------
 *     1     Y0      rh1  ch 8   0->130     lh1  ch 0  180-> 50
 *     2     Y1      rh2  ch 9   0->120     lh2  ch 1  180-> 60
 *     3     Y2      rh3  ch 10 20->120     lh3  ch 2  160-> 60
 *
 *   The left angles run the other way because the left arm mirrors the
 *   right: lh1 rests at 180 where rh1 rests at 0.
 *
 *   All six angle numbers, and the straight/bent calibration, are on
 *   the robot's web page under "Hand controller", saved in the ESP32's
 *   permanent memory. This board holds no calibration at all -- it only
 *   ever sends three raw numbers and which way the switch is set.
 *
 *====================================================================*/

/*  This sketch is for the ESP8266 (D1 mini / NodeMCU), NOT the ESP32.
 *  If you see "ESP8266WiFi.h: No such file or directory", the IDE has an
 *  ESP32 board selected -- pick  LOLIN(WEMOS) D1 R2 & mini  or
 *  NodeMCU 1.0 (ESP-12E)  and compile again. Nothing in the code is
 *  wrong when that error appears.                                    */
#if !defined(ESP8266) && !defined(ARDUINO_ARCH_ESP8266)
#error "Wrong board selected: choose an ESP8266 board (LOLIN D1 mini / NodeMCU 1.0) for the TRANSMITTER sketch."
#endif

#include <ESP8266WiFi.h>

/* ---- the ONE setting, already matching the robot sketch ---------- */
#define ROBOT_SSID   "Humanoid"
#define ROBOT_PASS   "12345678"
#define ROBOT_OPEN      0     // must match AP_OPEN in the robot sketch
#define ROBOT_CHANNEL   1     // must match AP_CHANNEL in the robot sketch
#define USE_STATIC_IP   1     // 1 = skip DHCP entirely. The robot's hotspot
                              // is always 192.168.4.1, so there is nothing
                              // to negotiate -- and DHCP is the single most
                              // common cause of "connection attempt timed
                              // out" on a board that can see the network.
#define GLOVE_IP_LAST  77     // this board becomes 192.168.4.77

/* ---- how many flex sensors ---------------------------------------
 *  Three: Flex 1 -> joint 1, Flex 2 -> joint 2, Flex 3 -> joint 3,
 *  on CD4051 channels Y0, Y1, Y2.
 *  (With 1 sensor you could skip the multiplexer: set USE_MUX to 0 and
 *  wire that one divider straight to A0.)                            */
#define NFLEX          3
#define USE_MUX        1

/* ---- multiplexer select pins (CD4051 / 74HC4051) ----------------- */
#define MUX_S0        D5      // GPIO14
#define MUX_S1        D6      // GPIO12
#define MUX_S2        D7      // GPIO13

/* ---- RIGHT / LEFT arm push button through CD4051 -----------------
 *  Wiring:
 *      3.3V --[10k]--+-- CD4051 Y4 (pin 1)
 *                    |
 *                 BUTTON
 *                    |
 *                   GND
 *
 *  Released reads about 1024. Pressed reads about 0.
 *  Every press flips RIGHT <-> LEFT.
 */
#define SIDE_MUX_CHANNEL    4
#define SIDE_ADC_THRESHOLD 500
#define SIDE_INVERT          0
#define SIDE_DEBOUNCE_MS    35
#define SIDE_RELEASE_MS     25
#define SIDE_MODE            1
#define SIDE_LED_BLINK       1
#define SIDE_USE_ISR         0   // Y4 is analog via A0, so no GPIO interrupt

/* ---- timing ------------------------------------------------------ */
#define TX_MS         60      // fastest send rate: about 16 packets/s
#define KEEPALIVE_MS 400      // send even when nothing moved, so the
                              // robot knows the glove is still there
#define MOVE_MIN       6      // ADC counts of change worth sending
#define SMOOTH         4      // moving-average length per sensor
#define HTTP_MS      250      // give up on a packet after this long

/* ---- status LED -------------------------------------------------- */
#define LED_PIN       LED_BUILTIN     // D4 / GPIO2, active LOW

/* =================================================================== */

int      raw[NFLEX];                  // smoothed reading, 0..1023
int      hist[NFLEX][SMOOTH];         // ring buffer per sensor
uint8_t  hpos = 0;
int      lastSent[NFLEX];
uint32_t lastTx = 0, lastAny = 0, lastPrint = 0;
uint8_t  side   = 0;                  // 0 = right arm, 1 = left arm
uint8_t  sideRawLast = 0;
uint32_t sideChangeMs = 0;
/*  PART 3 state. volatile because the interrupt writes them. */
volatile uint8_t  sidePress   = 0;    // set by the ISR, cleared by sideTick
volatile uint32_t sidePressMs = 0;    // when the ISR saw it
uint8_t  sideArmed  = 1;              // 0 until a clean release is seen
uint32_t sideRelMs  = 0;              // when the release started
uint32_t sideFlips  = 0;              // how many times the arm has flipped
uint8_t  winkLeft   = 0;              // LED winks still owed
uint32_t winkNext   = 0;              // when the LED may change again
bool     flexOn = false;              // what the robot last told us
bool     linked = false;              // last packet got through
uint32_t okCount = 0, failCount = 0;

/*  Read one channel. With the mux: set the three select lines, wait a
 *  moment for the analog switch to settle, then sample A0.           */
int readSensor(uint8_t ch) {
#if USE_MUX
  digitalWrite(MUX_S0, (ch & 1) ? HIGH : LOW);
  digitalWrite(MUX_S1, (ch & 2) ? HIGH : LOW);
  digitalWrite(MUX_S2, (ch & 4) ? HIGH : LOW);
  delayMicroseconds(60);              // settling. 60 us is plenty.
#else
  (void)ch;
#endif
  return analogRead(A0);              // 0..1023 on a NodeMCU / D1 mini
}

/*  Sample every sensor once and keep a short moving average. Flex
 *  sensors are noisy and a servo hears every count of it.            */
void sampleAll() {
  for (int i = 0; i < NFLEX; i++) {
    hist[i][hpos] = readSensor(i);
    long sum = 0;
    for (int k = 0; k < SMOOTH; k++) sum += hist[i][k];
    raw[i] = (int)(sum / SMOOTH);
  }
  hpos = (hpos + 1) % SMOOTH;
}

/*====================================================================
 *  PART 3: THE RIGHT / LEFT ARM BUTTON, FIXED
 *
 *  sideRead()  -- CD4051 Y4 analog button, with SIDE_INVERT applied.
 *                 1 = pressed / closed, 0 = released / open.
 *  sideIsr()   -- tiny, IRAM, latches a press. Nothing else.
 *  winkTick()  -- the two confirmation blinks, without ever blocking.
 *  sideTick()  -- the decision, called several times per loop.
 *===================================================================*/
static inline uint8_t sideRead() {
  // Read CD4051 Y4 through the existing COM -> A0 connection.
  // Released is high (~1024); pressed pulls Y4 low (~0).
  int v = readSensor(SIDE_MUX_CHANNEL);
  uint8_t r = (v < SIDE_ADC_THRESHOLD) ? 1 : 0;
#if SIDE_INVERT
  r = r ? 0 : 1;
#endif
  return r;
}

/*  Some ESP8266 core versions call this attribute IRAM_ATTR and older
 *  ones ICACHE_RAM_ATTR. Accept whichever the installed core provides,
 *  so this sketch compiles on both.                                   */
#ifndef ICACHE_RAM_ATTR
  #define ICACHE_RAM_ATTR IRAM_ATTR
#endif

#if SIDE_MODE && SIDE_USE_ISR
void ICACHE_RAM_ATTR sideIsr() {
  uint32_t now = millis();
  /*  One press per SIDE_DEBOUNCE_MS. Contact chatter arrives as a burst
   *  of edges inside a few milliseconds; only the first one counts.   */
  if (now - sidePressMs < (uint32_t)SIDE_DEBOUNCE_MS) return;
  sidePressMs = now;
  sidePress   = 1;
}
#endif

/*  Two winks of the D4 LED, one state change at a time. ledTick() is
 *  told to leave the LED alone while winkLeft is non-zero, and takes it
 *  straight back afterwards, so the status blink pattern is unchanged. */
void winkTick() {
  if (!winkLeft) return;
  if ((int32_t)(millis() - winkNext) < 0) return;
  winkNext = millis() + 70;
  winkLeft--;
  digitalWrite(LED_PIN, (winkLeft & 1) ? LOW : HIGH);   // active LOW
}

/*  Actually change arms. One place, so every path agrees.             */
void sideFlip(uint8_t to) {
  side = to ? 1 : 0;
  sideFlips++;
  Serial.printf("arm select -> %s ARM   (press %lu)\n",
                side ? "LEFT" : "RIGHT", (unsigned long)sideFlips);
#if SIDE_LED_BLINK
  winkLeft = 4;                          /* two winks, non-blocking */
  winkNext = 0;
#endif
  lastAny = 0;                           /* make the next packet go at once */
}

/*  Called from several places in loop(), including on both sides of the
 *  HTTP request, so a press is acted on within a few milliseconds no
 *  matter what else the sketch is doing.                              */
void sideTick() {
  uint8_t  r   = sideRead();
  uint32_t now = millis();

#if SIDE_MODE
  /*  ---- momentary push button ------------------------------------ */
  /*  A press is accepted only when the button was previously seen
   *  cleanly released; that is what stops release-bounce counting as a
   *  second press and flipping the arm straight back.                */
  if (!r) {                                  /* released */
    if (sideRelMs == 0) sideRelMs = now;
    if (!sideArmed && now - sideRelMs >= (uint32_t)SIDE_RELEASE_MS)
      sideArmed = 1;                         /* ready for the next press */
    sidePress = 0;                           /* nothing to act on */
    return;
  }
  sideRelMs = 0;                             /* it is held down now */

  /*  Two ways to notice the press: the interrupt latched it, or we can
   *  see it held for longer than the debounce time. Either is enough. */
  uint8_t latched = 0;  // Y4 button uses polling, not a GPIO interrupt
  if (sideRawLast != r) { sideRawLast = r; sideChangeMs = now; }
  bool held = (now - sideChangeMs >= (uint32_t)SIDE_DEBOUNCE_MS);

  if (!sideArmed)        return;             /* waiting for a release  */
  if (!latched && !held) return;             /* still bouncing         */

  sideArmed = 0;                             /* one flip per press     */
  sideFlip(side ? 0 : 1);
#else
  /*  ---- latching slide switch: closed = LEFT, open = RIGHT ------- */
  if (r != sideRawLast) { sideRawLast = r; sideChangeMs = now; return; }
  if (now - sideChangeMs < (uint32_t)SIDE_DEBOUNCE_MS) return;
  if (r == side) return;
  sideFlip(r);
#endif
}

/*  Anything actually moved?                                          */
bool worthSending() {
  for (int i = 0; i < NFLEX; i++)
    if (abs(raw[i] - lastSent[i]) >= MOVE_MIN) return true;
  return false;
}

/*  PART 4 reporting state. Declared here, above the first function
 *  that touches it, because a .ino only gets automatic prototypes for
 *  functions -- never for variables.                                  */
bool     wasUp    = false;      // last known Wi-Fi state
uint32_t nextSay  = 0;          // throttle for the "sending" lines
uint32_t nextFail = 0;          // throttle for the failure lines

/*  One short HTTP GET to the robot:  /flex?v=512,430,610&s=0
 *  The robot answers "1" if Flex Control is ON, "0" if it is OFF, and
 *  that is all we need for the status LED.
 *
 *  Deliberately a plain socket rather than HTTPClient: fewer layers,
 *  no keep-alive assumptions, and a hard time budget so a missing
 *  robot can never lock the glove up.                                */
void sendPacket() {
  WiFiClient c;
  c.setTimeout(HTTP_MS);
  IPAddress robot = WiFi.gatewayIP();       // the hotspot IS the robot
  uint32_t now = millis();
  bool say = (nextSay == 0) || (int32_t)(now - nextSay) >= 0;
  if (say) {
    nextSay = now + 1000;
    Serial.println(F("Sending flex data to ESP32..."));
    for (int i = 0; i < NFLEX; i++) Serial.printf("Flex%d: %d\n", i + 1, raw[i]);
  }
  if (!c.connect(robot, 80)) {
    linked = false;
    failCount++;
    if ((int32_t)(now - nextFail) >= 0) {
      nextFail = now + 800;
      Serial.println(F("ESP32 communication failed"));
      Serial.println(F("Retrying..."));
    }
    return;
  }

  String url = "/flex?v=";
  for (int i = 0; i < NFLEX; i++) { if (i) url += ','; url += raw[i]; }
  url += "&s="; url += side;          // 0 = right arm, 1 = left arm

  c.print(String("GET ") + url + " HTTP/1.1\r\n" +
          "Host: " + robot.toString() + "\r\n" +
          "Connection: close\r\n\r\n");

  /*  Read the reply, but never for longer than HTTP_MS. We only care
   *  about the very last character of the body.                      */
  uint32_t t0 = millis();
  String body;
  while (c.connected() && millis() - t0 < HTTP_MS) {
    while (c.available()) body += (char)c.read();
    if (body.indexOf("\r\n\r\n") >= 0 && !c.available()) break;
    delay(1);
  }
  c.stop();

  int sep = body.indexOf("\r\n\r\n");
  if (sep >= 0) {
    String payload = body.substring(sep + 4);
    payload.trim();
    if (payload.length()) flexOn = (payload[payload.length() - 1] == '1');
    if (say || !linked) Serial.println(F("ESP32 communication successful"));
    linked = true;
    okCount++;
    for (int i = 0; i < NFLEX; i++) lastSent[i] = raw[i];
  } else {
    linked = false;
    failCount++;
    if ((int32_t)(millis() - nextFail) >= 0) {
      nextFail = millis() + 800;
      Serial.println(F("ESP32 communication failed"));
      Serial.println(F("Retrying..."));
    }
  }
}

/*  fast blink = no wifi, slow blink = connected but OFF,
 *  solid = connected and Flex Control ON                             */
void ledTick() {
  if (winkLeft) return;        /* PART 3: the arm-change wink has the LED */
  bool on;
  uint32_t m = millis();
  if (WiFi.status() != WL_CONNECTED || !linked) on = (m % 200) < 100;
  else if (!flexOn)                             on = (m % 1200) < 120;
  else                                          on = true;
  digitalWrite(LED_PIN, on ? LOW : HIGH);       // active LOW
}

/*  PART 4.  Exactly the messages you asked for, and no lies: the glove
 *  only says it is connected when the Wi-Fi layer says so, and only
 *  says the robot answered when a real reply came back.
 *
 *  Everything is edge triggered or throttled, so the serial monitor
 *  stays readable instead of scrolling 16 times a second.            */
/*  Puts the Wi-Fi status code into English, so a failure tells you
 *  WHICH failure it is instead of just "not connected".              */
const char* wlWhy() {
  switch (WiFi.status()) {
    case WL_NO_SSID_AVAIL:   return "hotspot not on the air";
    case WL_CONNECT_FAILED:  return "refused -- password or channel";
    case WL_CONNECTION_LOST: return "connection lost";
    case WL_IDLE_STATUS:     return "radio idle";
    case WL_DISCONNECTED:    return "disconnected";
    default:                 return "still trying";
  }
}

/*  PART 4, the real fix.  Your glove could SEE the robot at -54 dBm and
 *  still timed out, which means the name and the range were never the
 *  problem. Three things are done differently here:
 *
 *    * NO SLEEP on the client either. A dozing ESP8266 misses the reply
 *      to its own handshake.
 *    * A FIXED IP by default, so DHCP cannot time out. The robot is
 *      always 192.168.4.1; this board simply takes 192.168.4.77.
 *    * JOIN BY BSSID AND CHANNEL. One quick scan finds the robot's
 *      actual radio, then WiFi.begin() is told exactly where to go
 *      instead of probing all thirteen channels on every retry.
 *
 *  Every third failed attempt the radio is switched fully off and on,
 *  which clears the stuck state the SDK gets into after a timeout, and
 *  every fourth attempt falls back to DHCP just in case the fixed
 *  address is the thing being objected to.                            */
void wifiJoin(bool useStatic) {
  Serial.println(F("Trying to connect to ESP32..."));
  Serial.printf ("SSID: %s\n", ROBOT_SSID);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);      /* do not doze mid-handshake */
  WiFi.setPhyMode(WIFI_PHY_MODE_11N);
  WiFi.setOutputPower(20.5);
  WiFi.setAutoReconnect(true);

  if (useStatic) {
    IPAddress ip(192, 168, 4, GLOVE_IP_LAST), gw(192, 168, 4, 1),
              mask(255, 255, 255, 0);
    WiFi.config(ip, gw, mask, gw);
    Serial.printf("fixed IP 192.168.4.%d, gateway 192.168.4.1 (no DHCP)\n",
                  GLOVE_IP_LAST);
  } else {
    WiFi.config(0U, 0U, 0U);               /* hand it back to DHCP */
    Serial.println(F("asking for a DHCP address this time"));
  }

  int  n = WiFi.scanNetworks();
  int  best = -1;
  int32_t bestRssi = -999;
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) != String(ROBOT_SSID)) continue;
    if (WiFi.RSSI(i) > bestRssi) { bestRssi = WiFi.RSSI(i); best = i; }
  }
  if (best >= 0) {
    Serial.printf("%s found: RSSI %d dBm, channel %d\n",
                  ROBOT_SSID, (int)WiFi.RSSI(best), WiFi.channel(best));
    uint8_t bss[6];
    memcpy(bss, WiFi.BSSID(best), 6);
    WiFi.begin(ROBOT_SSID, ROBOT_OPEN ? (const char*)NULL : (const char*)ROBOT_PASS,
               WiFi.channel(best), bss, true);
  } else {
    Serial.println(F("that hotspot is not in the scan -- is the robot powered up?"));
    WiFi.begin(ROBOT_SSID, ROBOT_OPEN ? (const char*)NULL : (const char*)ROBOT_PASS,
               ROBOT_CHANNEL);
  }
  WiFi.scanDelete();
}

void wifiTick() {
  static uint32_t nextTry = 0;
  static uint16_t tries   = 0;
  bool up = (WiFi.status() == WL_CONNECTED);

  if (up != wasUp) {                            /* the state changed */
    wasUp = up;
    if (up) {
      Serial.println(F("Wi-Fi connected to ESP32 hotspot"));
      Serial.print(F("ESP8266 IP: "));
      Serial.println(WiFi.localIP());
      Serial.print(F("Gateway: "));
      Serial.println(WiFi.gatewayIP());         /* this IS the robot */
      nextSay = 0;
    } else {
      linked = false;
      Serial.println(F("Wi-Fi connection to ESP32 lost"));
      Serial.println(F("Retrying..."));
      nextTry = 0;                              /* try again at once */
    }
  }
  if (up) { tries = 0; return; }

  if (millis() < nextTry) return;
  nextTry = millis() + 4000;
  tries++;
  Serial.printf("attempt %u: %s\n", (unsigned)tries, wlWhy());

  if (tries % 3 == 0) {          /* clear a stuck radio, properly */
    Serial.println(F("resetting the ESP8266 radio..."));
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(250);
    WiFi.mode(WIFI_STA);
    delay(50);
  }
  /*  Mostly fixed IP, every fourth go with DHCP, so neither choice can
   *  lock you out on its own.                                        */
  wifiJoin(USE_STATIC_IP && (tries % 4 != 0));
}

void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.println();
  Serial.println(F("Starting ESP8266..."));
  Serial.println(F("=================================================="));
  Serial.println(F(" ESP8266 HAND CONTROLLER  --  the wearable glove"));
  Serial.printf (" %d flex sensors, %s\n", NFLEX,
                 USE_MUX ? "CD4051 on D5/D6/D7, COM -> A0"
                         : "single sensor straight into A0");
  Serial.println(F(" push button on CD4051 Y4 switches RIGHT / LEFT arm"));
  Serial.printf (" looking for the robot hotspot \"%s\"\n", ROBOT_SSID);
  Serial.println(F(" nothing to configure -- no IP, no pairing"));
  Serial.println(F("=================================================="));

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);          // off
#if USE_MUX
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
#endif

  /*  Read the switch straight away so the very first packet already
   *  carries the right arm, not a default.                           */
#if SIDE_MODE
  /*  PART 3: with a momentary BUTTON the resting state means nothing --
   *  it is simply not pressed at power-up. Start on the RIGHT arm and
   *  let the first press flip it; that is what "every press switches
   *  arms" means.                                                     */
  side        = 0;
  sideRawLast = sideRead();
  sideArmed   = sideRawLast ? 0 : 1;   /* held at boot: wait for release */
  sideRelMs   = sideRawLast ? 0 : millis();
  Serial.println(F(" Y4 is a push BUTTON: every press switches arms"));
  Serial.println(F(" starting on the RIGHT ARM"));
#else
  side = sideRead();
  sideRawLast = side;
  Serial.printf(" switch on D2 says %s ARM\n", side ? "LEFT" : "RIGHT");
#endif
  sideChangeMs = millis();
  sidePress    = 0;
  sidePressMs  = 0;


  /*  Prime the moving average so the first packet is already sane.   */
  for (int k = 0; k < SMOOTH; k++) sampleAll();
  for (int i = 0; i < NFLEX; i++) lastSent[i] = -999;

  /*  Start from a clean radio: a persisted half-finished join from a
   *  previous upload is enough to make every later attempt time out. */
  WiFi.persistent(false);
  WiFi.setAutoConnect(false);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(150);
  WiFi.mode(WIFI_STA);
  delay(50);
  wifiJoin(USE_STATIC_IP);
}

void loop() {
  wifiTick();
  sideTick();
  winkTick();
  sampleAll();
  ledTick();

  uint32_t now = millis();
  if (WiFi.status() == WL_CONNECTED && now - lastTx >= TX_MS) {
    /*  A flick of the switch is sent at once, so the arm changes over
     *  without waiting for the keepalive.                            */
    static uint8_t sentSide = 255;
    if (worthSending() || side != sentSide || now - lastAny >= KEEPALIVE_MS) {
      sentSide = side;
      lastTx = now;
      lastAny = now;
      sideTick();          /* PART 3: look at the button on both sides of */
      sendPacket();        /* the HTTP request, which may block for up to */
      sideTick();          /* HTTP_MS. A fast tap can no longer fall down */
      winkTick();          /* the gap.                                   */
    } else {
      lastTx = now;
    }
  }

  /*  A readable line twice a second, so you can watch the numbers
   *  while you bend a finger and see whether your divider resistor is
   *  a good match before you even open the web page.                 */
  if (now - lastPrint >= 500) {
    lastPrint = now;
    Serial.printf("%s  flex %s  %-5s ",
                  WiFi.status() == WL_CONNECTED
                    ? (linked ? "robot OK " : "wifi only")
                    : "no wifi  ",
                  flexOn ? "ON " : "OFF",
                  side ? "LEFT" : "RIGHT");
    for (int i = 0; i < NFLEX; i++) Serial.printf("F%d=%4d ", i + 1, raw[i]);
    Serial.printf(" sent %lu  lost %lu\n",
                  (unsigned long)okCount, (unsigned long)failCount);
  }

  delay(2);          // the ESP8266 wifi stack needs the breathing room
}
