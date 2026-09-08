/*======================================================================
 *
 *      ESP32  HUMANOID ROBOT CONTROLLER  --  THE RECEIVER
 *      ==================================================
 *
 *      >>> THIS FILE GOES ON THE ESP32 INSIDE THE ROBOT.        <<<
 *      >>> The other file, ESP8266_Hand_Controller_TRANSMITTER   <<<
 *      >>> .ino, goes on the ESP8266 you wear on your hand.      <<<
 *
 *      This is your original "Humanoid ESP32 WiFi.ino" with ONE thing
 *      added: the flex glove layer. Every existing feature is exactly
 *      as it was -- the website, all the movements, the saved poses,
 *      the OLED faces, the sounds, the head gestures, the balance
 *      assist, the serial commands, the hotspot. Nothing was rewritten
 *      and nothing was removed.
 *
 *----------------------------------------------------------------------
 *  WHAT WAS ADDED, AND WHY IT CANNOT GET IN THE ROBOT'S WAY
 *----------------------------------------------------------------------
 *    - the glove sends readings to  GET /flex?v=...  which the web
 *      server you already run answers inside pumpIO()
 *    - flexTick(), also called from pumpIO(), applies them -- and its
 *      FIRST LINE is  if (!gFlexOn || gBusy || gFreeze) return;
 *
 *      That single line is the whole design. gBusy is true for the
 *      entire length of any movement, saved pose or website command, so
 *      while the robot is walking the glove simply stands aside. Press
 *      STOP or start another movement and that wins immediately. When
 *      the movement finishes gBusy clears and the glove is live again,
 *      with no handover code and nothing to reset.
 *
 *      Turn it off and the sketch behaves exactly like your original:
 *      one boolean, one early return, no servo writes.
 *
 *      Nothing anywhere waits for the ESP8266. If the glove is off, out
 *      of range or never built, flexTick() returns on a stale timestamp
 *      and the robot does not notice.
 *
 *----------------------------------------------------------------------
 *  DO I HAVE TO CONFIGURE ANYTHING?   No.
 *----------------------------------------------------------------------
 *      IP address    no. The glove uses its wifi gateway, which is this
 *                    ESP32 (192.168.4.1).
 *      SSID / pass   no, as long as you leave AP_SSID / AP_PASS below
 *                    alone ("Humanoid" / "12345678"). They are already
 *                    written into the ESP8266 sketch. If you change them
 *                    here, change the two matching lines there.
 *      Pairing       none. Power the glove on and it appears.
 *
 *----------------------------------------------------------------------
 *  WIRING FOR THIS BOARD (unchanged from your build)
 *----------------------------------------------------------------------
 *      ESP32 GPIO 21 -> PCA9685 SDA        ESP32 GPIO 25 -> head servo
 *      ESP32 GPIO 22 -> PCA9685 SCL        OLED  -> SDA 32, SCL 33
 *      ESP32 3V3/GND -> PCA9685 VCC/GND    MAX98357A -> DIN 14,
 *      PCA9685 V+    -> 6 V servo supply +   BCLK 26, LRC 27
 *      PCA9685 GND   -> servo supply -  AND ESP32 GND  (one common
 *                       ground -- this is not optional)
 *      PCA9685 ch0..15 -> the 16 body servos
 *      ESP32 power   -> 5 V 2 A into VIN/5V, or USB. Do NOT run the
 *                       servos off the ESP32's regulator.
 *      Glove link    -> nothing to wire. It is wifi.
 *
 *      Full glove wiring, the CD4051 pinout and the resistor values are
 *      in the header of ESP8266_Hand_Controller_TRANSMITTER.ino.
 *
 *----------------------------------------------------------------------
 *  THREE SENSORS, EITHER ARM  (defaults; all editable on the web page,
 *  stored in NVS so they survive a power cut)
 *----------------------------------------------------------------------
 *  The switch on the glove decides which arm the SAME three sensors
 *  drive. Nothing else changes -- one calibration per finger serves
 *  both arms, because it is the same finger either way.
 *
 *    Flex   switch = RIGHT            switch = LEFT
 *    ----   -------------------       ------------------------
 *     1     rh1  ch 8    0 -> 130     lh1  ch 0   180 ->  50
 *     2     rh2  ch 9    0 -> 120     lh2  ch 1   180 ->  60
 *     3     rh3  ch 10  20 -> 120     lh3  ch 2   160 ->  60
 *
 *  The left angles run the other way because the left arm is the mirror
 *  of the right: lh1 rests at 180 where rh1 rests at 0.
 *
 *  CALIBRATION, on the web page: hold the finger straight, press
 *  "Straight"; bend it fully, press "Bent". Then set the two servo
 *  angles for the right arm and for the left arm and press Save.
 *
 *  Serial monitor also has:  flex on | flex off | flex   (live report)
 *
 *====================================================================*/

/**********************************************************************
 *  HUMANOID ROBOT  --  ESP32 + PCA9685  --  v6  "ALIVE"
 *
 *  All 25 routines from your v3 sketch are here, byte-for-byte
 *  identical, with the same Mega angles.  What is new:
 *
 *   0. IT HAS A VOICE, A FACE AND A NECK, AND THEY MOVE TOGETHER.
 *      Every command carries four things in ONE table row (section
 *      17): the routine, the face, the sound and the head movement.
 *      They cannot drift apart, because there is only one place to
 *      change any of them.  Say hi and it chirps, grins and nods at
 *      the same instant.  Twenty faces, twenty-six sounds, eleven
 *      head movements, all built from maths -- there is no SD card
 *      and there are no files to upload.
 *
 *      The sound and the face each run in their own task on core 0,
 *      so neither of them can ever delay a servo on core 1.  Pushing
 *      a full OLED frame takes about 25 ms; on core 1 that would show
 *      up as a stutter in the legs, and this is why it does not.
 *
 *      Amplifier: MAX98357A, three wires.  Wiring in section 1f.
 *      Head servo: the 17th joint, straight onto GPIO 25, because the
 *      PCA9685 has exactly 16 outputs.  Wiring in section 2.
 *
 *   1. THE TWITCHING IS FIXED IN SOFTWARE AS FAR AS IT CAN BE.
 *      Three separate software causes were found and removed:
 *        (a) the sketch re-sent the SAME pulse value to servos that
 *            were not moving -- about 850 pointless I2C writes every
 *            second during a glide.  Each one restarts that channel's
 *            PWM phase mid-cycle, which a servo feels as a nudge.
 *            Now a write that would not change the pulse is dropped.
 *        (b) I2C ran at 400 kHz on long unshielded servo wiring.  A
 *            corrupted setPWM is a servo jumping somewhere random and
 *            snapping back -- the "glitch" sound.  Now 100 kHz.
 *        (c) all 17 servos were energised in the same millisecond at
 *            power-up.  Now they come up one at a time, 120 ms apart.
 *      When the robot is standing still this sketch sends NOTHING at
 *      all -- zero bus traffic.  Type  quiet  to have it prove that.
 *
 *   2. WALKING IS REWRITTEN.  Your move_forward() had a real bug and
 *      a real omission (both measured, see below).  turn left/right
 *      had four 20-degree slams per cycle.  All three are replaced
 *      with one drift-free parametric gait.  Your originals are still
 *      here as  old walk / old turn left / old turn right  so you can
 *      compare on the actual robot.
 *
 *   3. MPU6050 balance assist + a 128x64 OLED face.  Both optional,
 *      both explained below.
 *
 *  ==================================================================
 *  READ THIS FIRST -- IT MATTERS MORE THAN ANY OF THE CODE
 *
 *  You said the servos run from 6 V 2 A.  That is the main reason the
 *  robot buzzes, will not hold its pose stiffly, and falls over.
 *
 *  One MG996R draws roughly 0.5-0.9 A just HOLDING a loaded joint,
 *  and about 2.5 A stalled.  Seventeen of them standing up need
 *  something like 6-10 A.  On a 2 A supply the rail collapses, every
 *  servo loses position, they all re-seek, that draws even more
 *  current, and the rail sags again.  That loop IS the noise you are
 *  hearing, and it is why the joints feel soft.  No firmware can add
 *  amps.
 *
 *      What you need:  6 V, 10 A or more.
 *      Good options:   2S LiPo (7.4 V) + a 6 V 10 A UBEC,
 *                      or a 6 V 15 A switching supply,
 *                      or 5 NiMH D cells.
 *      Also:           a 2200-4700 uF capacitor across V+ and GND
 *                      right at the PCA9685 terminals, thick wire
 *                      (18 AWG or better) for V+ and GND, and the
 *                      ESP32 powered separately from USB.
 *      Never:          power servos from the ESP32's 5V pin.
 *
 *  Until the supply is fixed, everything below will help but will not
 *  fully cure it.  After it is fixed, this sketch should be silent
 *  when idle.
 *
 *  ==================================================================
 *  WHAT WAS MEASURED IN THE OLD WALK  (replayed frame by frame)
 *
 *   * The knees never moved.  ll3 and rl3 sat at 150 and 30 for the
 *     entire walk, so each leg swung as one rigid stick with the sole
 *     held flat -- the foot scraped the floor the whole way.  That is
 *     your "the leg doesn't come completely up" and "it gets stuck".
 *
 *   * The right leg drifted.  The return loops for rl2 and rl4 ran
 *     i = 0..30 while the outward loops ran i = 0..20, so every cycle
 *     ended 10 degrees past where it started:
 *         rl2  150 -> 160      rl4  30 -> 40
 *     The next cycle then began with an absolute write back to 150
 *     and 30, i.e. a 10 degree snap, once per step, forever.  lh2
 *     drifted 20 degrees the same way.
 *
 *   * The weight shift was only 5 degrees of roll.  That is not
 *     enough to actually unload a foot, so the "swinging" leg was
 *     still carrying the robot and had to drag itself round.
 *
 *   The new gait holds three geometric rules in every single frame:
 *       flat sole      d(ankle) = d(hip) + d(knee)   on both legs
 *       roll coupling  ll1 = rl1 = ll5 = rl5         always
 *       knee mirror    left knee bends by decreasing its angle,
 *                      right knee by increasing its own
 *   and it returns to exactly the standing pose, so it cannot drift.
 *
 *  ==================================================================
 *  IS THE MPU6050 WORTH IT?   Short answer: yes, add it -- but know
 *  what it does and does not do.
 *
 *  Your worry was "will all the servos then be moving every second?"
 *  That is a fair worry and it is exactly what a naive IMU loop does.
 *  This one will not, because of three deliberate limits:
 *
 *      DEADBAND    Inside +/- 3 degrees of level it sends NOTHING.
 *                  Not a small correction -- nothing at all.  So a
 *                  robot that is standing acceptably straight is
 *                  completely silent.
 *      SLOW        Outside the deadband it moves at most 1 degree,
 *                  and at most once every 150 ms.  It leans, it does
 *                  not twitch.
 *      SMALL       It may only trim 8 degrees total, and only the
 *                  four roll joints and the two ankles.  It can lean
 *                  the robot; it can never throw it.
 *
 *  What it WILL do for you: hold a straight stand instead of slowly
 *  leaning over, take up the slop in the ankles, stop a routine and
 *  warn you when the robot is actually falling, and tell you whether
 *  it is upright, sitting or on its face.
 *
 *  What it will NOT do: catch a real fall, or make a bad gait good.
 *  Genuine dynamic balancing needs the ankles corrected 100+ times a
 *  second, and an MG996R updated at 50 Hz through a geartrain with
 *  visible backlash cannot close that loop.  Sit, stand and walk have
 *  to work open-loop first; the MPU then keeps them tidy.
 *
 *  Turn it off any time with:  balance off      (no reflash needed)
 *  IMPORTANT: leave balance OFF until the power supply is upgraded.
 *  On a sagging 2 A rail the corrections cause the sag that triggers
 *  more corrections, and it will oscillate.
 *
 *  ==================================================================
 *  WIRING
 *
 *  SERVO BUS  (I2C bus 0, 100 kHz)
 *      ESP32 GPIO 21 -> PCA9685 SDA
 *      ESP32 GPIO 22 -> PCA9685 SCL
 *      ESP32 GND     -> PCA9685 GND      <-- must be common
 *      ESP32 GPIO 25 -> head servo signal
 *      PCA9685 V+    -> 6 V 10 A+ supply, NOT the ESP32 5V pin
 *      PCA9685 VCC   -> ESP32 3V3 (logic only)
 *      2200 uF+ capacitor across V+/GND at the PCA9685
 *
 *      PCA9685:  ch0 lh1  ch1 lh2  ch2 lh3  ch3 ll1  ch4 ll2
 *                ch5 ll3  ch6 ll4  ch7 ll5  ch8 rh1  ch9 rh2
 *                ch10 rh3 ch11 rl1 ch12 rl2 ch13 rl3 ch14 rl4
 *                ch15 rl5      head = GPIO 25
 *
 *  MPU6050  (same bus as the servos -- address 0x68 vs PCA 0x40)
 *      VCC -> 3V3      GND -> GND      SDA -> 21      SCL -> 22
 *      AD0 -> leave unconnected
 *      Mount it on the torso, as flat and as central as you can, with
 *      the chip's X axis pointing forward.  Exact alignment does not
 *      matter -- run  bal zero  while the robot is standing straight
 *      and it stores the offset in flash.
 *
 *  OLED 128x64 SSD1306  (its OWN I2C bus, bus 1 -- deliberately)
 *      VCC -> 3V3      GND -> GND
 *      SDA -> GPIO 32
 *      SCL -> GPIO 33
 *      Address 0x3C (a few modules are 0x3D -- see OLED_ADDR below).
 *
 *      Why a second bus: pushing a full screen is over a thousand
 *      bytes.  On the servo bus that would sit in front of servo
 *      updates and cause exactly the stutter we are trying to remove.
 *      On its own bus it cannot interfere.  The sketch also never
 *      sends more than one 128-byte strip per pass for the same
 *      reason.  No library needed -- the driver is in this file.
 *
 *  ==================================================================
 *  CONTROL
 *      Serial monitor  115200, any line ending.  Type  help
 *      Web page        connect to hotspot "Humanoid" / "12345678"
 *                      then open  http://192.168.4.1
 *      Blynk / Sinric  behind USE_BLYNK / USE_SINRIC compile flags
 *
 *  FIRST RUN, IN ORDER
 *      1.  power up with the robot held or hung so it cannot fall
 *      2.  sweep ll2 30 90    then   sweep rl2 150 90
 *          both legs must swing the SAME way.  If not:  flip ll2  save
 *      3.  stand          4.  bal zero        5.  sit down / stand up
 *      6.  walk           7.  balance on   (only after the new supply)
 *********************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <FS.h>
#include <LittleFS.h>
#include "Adafruit_PWMServoDriver.h"

/*====================================================================
 *  1. THINGS YOU MIGHT WANT TO CHANGE
 *===================================================================*/

/* ---- motion feel ------------------------------------------------ */
#define MAX_STEP_DEG    6     // hard ceiling: degrees a servo may be
                              // ordered to move in one go. Lower =
                              // gentler and slower. 4-8 is sensible.
#define SUBSTEP_MS      6     // pause between rate-limiter sub-steps

#define GLIDE_STEP_MS   20    // frame time while gliding between poses
#define GLIDE_MS_PER_DEG 18   // glide speed: 18 ms per degree of the
                              // furthest-travelling joint
#define GLIDE_MIN_MS    700
#define GLIDE_MAX_MS    5000

#define SETTLE_MS       2000  // pause after reaching the start pose

/* Speed as a percentage of the original Mega timing.
 * 100 = exactly your Mega speed.  Bigger number = SLOWER.        */
#define SPEED_DEFAULT   100
#define SPEED_SIT       160   // sit down / stand up: slow
#define SPEED_PUSHUP    110
#define SPEED_WALK      100

#define RETURN_TO_STAND 1

#define SIT_SPREAD_DEG  0     // widen the hips before sitting. 0 = off,
                              // try 10-15 if the thighs collide.

#define PUSHUP_REPS     10
/*  ---- PART 5: why the push-up looked like it "stopped halfway" -------
 *  It did not stop. After the six set-up phases the routine deliberately
 *  sat still for PUSHUP_HOLD_MS, and that used to be 10000 -- a full ten
 *  seconds (eleven at SPEED_PUSHUP 110) of absolutely no movement, right
 *  in the middle of the sequence, before the ten reps began. There is no
 *  way to tell that apart from a crash by watching the robot, so it read
 *  as "it stops halfway". It is now a short, obvious brace-yourself
 *  pause, and every phase prints its own progress line so you can see
 *  the sequence marching through the whole thing on the serial monitor.
 *
 *  The other half of the fix is in push_ups() itself, which now glides
 *  every joint of a phase together in lock step instead of writing them
 *  one after another, and glides INTO the first phase from wherever the
 *  robot happens to be standing, so there is no lurch at the start.
 *
 *  Set PUSHUP_HOLD_MS back to a larger number if you liked the long
 *  pause; nothing else depends on it.                                 */
#define PUSHUP_HOLD_MS  1200
#define PUSHUP_TRACE    1     // 1 = print every phase as it happens, so
                              // a real stall is obvious on the monitor
#define SEG_STEP_MS     14    // one interpolation step of a glide. Small
                              // enough to look continuous, large enough
                              // that the I2C bus is never saturated.
#define TURN_STEPS      30    // cycles for the OLD turn routines
#define WALK_STEPS      10    // cycles for the OLD walk routine

#define MOVE_ON_BOOT    0     // 0 = hold still at power-up and wait

/*--------------------------------------------------------------------
 *  ANTI-JITTER.  These remove every twitch the FIRMWARE can cause.
 *  Read section 1 above: a 6 V 2 A supply will still buzz, and no
 *  setting here can fix that.  Type  quiet  to tell the two apart.
 *-------------------------------------------------------------------*/
#define DEDUPE_WRITES   1     // 1 = never re-send a pulse a servo is
                              // already holding.  Measured effect: it
                              // makes a repeated command (e.g. "stand
                              // up" while already standing) send 0
                              // writes instead of 12.  It is NOT what
                              // makes idle standing silent -- nothing
                              // writes at all while idle.  Leave on.
#define I2C_SERVO_HZ    100000  // 100 kHz. Raise to 400000 only if
                              // your servo wiring is short + shielded.
#define BOOT_STAGGER_MS 120   // gap between energising each servo at
                              // power-up, so 17 inrush surges do not
                              // land on the supply together.
#define QUIET_REPORT_MS 3000  // sample window for the  quiet  command

/*--------------------------------------------------------------------
 *  THE NEW WALK.  Angles are degrees.
 *-------------------------------------------------------------------*/
#define GAIT_HIP_AMP     14   // hip swing either side of centre.
                              // Bigger = longer stride, less stable.
#define GAIT_KNEE_LIFT   26   // knee bend while a leg is swinging.
                              // THIS is what lifts the foot off the
                              // floor. Raise it if the foot still
                              // scuffs; lower it if the robot bobs.
#define GAIT_ROLL        12   // weight shift onto the standing foot.
                              // Your old walk used 5, which was not
                              // enough to unload the other foot.
#define GAIT_LEAN_SIGN   1    // +1 or -1.  Which way positive roll
                              // leans. If the robot leans onto the
                              // foot it is trying to LIFT, use -1.
#define GAIT_FRAME_MS    20
#define GAIT_SHIFT_MS    420  // weight-shift phase
#define GAIT_LIFT_MS     260  // knee lift phase
#define GAIT_SWING_MS    440  // leg swing phase
#define GAIT_LOWER_MS    300  // put the foot back down
#define GAIT_PAUSE_MS    90   // small settle between phases
#define WALK_HALFSTEPS   8    // one half-step = one foot forward
#define GAIT_ARM_SWING   0    // 1 = swing the arms while walking.
                              // Off by default: it costs current and
                              // adds wobble you do not need yet.
#define TURN_HIP         16   // stepping-leg hip swing when turning
#define TURN_CYCLES      6    // how many shuffle cycles per command

/* ---- hardware --------------------------------------------------- */
#define BAUD            115200
#define I2C_SDA         21
#define I2C_SCL         22
#define PCA_ADDR        0x40
#define PCA_FREQ        50    // 50 Hz is MG996R spec (v3 used 60)
#define PCA_OSC_HZ      25000000UL
/*  These MUST match the Arduino Servo library your working Mega sketch
 *  used, or every angle comes out wrong and the joints you send to 180
 *  jam past their mechanical stop, stall, and drag the supply down.
 *    Servo library: 0 deg = 544 us, 180 deg = 2400 us.
 *    At 50 Hz one count = 20000/4096 = 4.883 us, so:
 *      544 us  -> 111 counts        2400 us -> 492 counts
 *  Do not "widen" these for more travel -- you will hit the stops.   */
#define PCA_MIN         111   // count at 0 deg   (544 us, = Servo lib)
#define PCA_MAX         492   // count at 180 deg (2400 us, = Servo lib)
#define HEAD_PIN        25
#define HEAD_LEDC_CH    0
#define HEAD_US_MIN     509
#define HEAD_US_MAX     2340

/* ---- wifi / web ------------------------------------------------- */
#define USE_WEB         1
#define HOME_SSID       ""
#define HOME_PASS       ""
#define AP_SSID         "Humanoid"
#define AP_PASS         "12345678"
#define AP_CHANNEL      1     // 1, 6 or 11. The glove is told to look here.
#define AP_OPEN         0     // 1 = hotspot with NO password at all. If the
                              // WPA2 handshake still refuses to finish, set
                              // this to 1 here AND set ROBOT_OPEN to 1 in the
                              // glove sketch. Everything else is unchanged.
#define AP_MAX_CLIENTS  8     // the glove plus your phone, with room spare

/* ---- v8 additions: OLED fix, phone drawing, voice clip, auto stand-up --
 *  OLED_COL_OFFSET removes the bright stripe down the left edge of the
 *  1.3in SH1106 panels: their RAM is 132 columns wide, the visible glass
 *  is 128, so column 0 of the panel is really column 2 of the RAM.     */
#define OLED_COL_OFFSET  2
#define NDRAW            3      // permanent drawing slots kept in flash
#define VREC_MAX     24000      // recorded clip, bytes (11025 Hz, 8-bit)
#define VNVS_MAX     12000      // how much of it we try to keep in flash
#define AUP_TRIES        3      // auto stand-up attempts before giving up
#define AUP_GAP_MS    5000      // wait between attempts

extern bool gFlexOn;            // defined with the glove layer, further down
bool flexLive();
void oledShowUser();
void drawSlideTick();
void autoUpTick();
void idleFaceTick();

/*====================================================================
 *  FLEX GLOVE -- the ESP8266 hand controller (THIS BOARD IS THE
 *  RECEIVER).  Nothing here changes how the robot already behaves.
 *
 *  The glove joins this hotspot and does a plain HTTP GET a few times
 *  a second:   GET /flex?v=512,430,610&s=0
 *  s=0 means the switch on the glove is set to the RIGHT arm, s=1 the
 *  LEFT arm. The same three sensors drive whichever side is selected --
 *  Flex 1 -> joint 1, Flex 2 -> joint 2, Flex 3 -> joint 3.
 *  That is handled by the web server the robot already runs, inside
 *  pumpIO(), so it costs nothing when the glove is switched off and it
 *  NEVER blocks the robot waiting for the glove.
 *
 *  Flex Control is an EXTRA layer, not a mode: flexTick() gives up the
 *  moment gBusy is set, so a walk, a saved pose, a website button or
 *  STOP always wins. When the movement ends, gBusy clears and the
 *  glove is live again with no extra work.                           */
#define NFLEX            3     // three flex sensors: joint1, joint2, joint3
#define FLEX_STALE_MS 1500     // no packet for this long = link lost
#define FLEX_APPLY_MS   40     // how often the glove angles are applied
#define FLEX_MAX_STEP    4     // degrees per tick: 4 deg / 40 ms = 100 deg/s
/*  ---- PART 1: ONE SPEED LIMITER PER MOTOR ------------------------
 *  The three flex-driven motors are called, everywhere in this sketch
 *  and everywhere on the web page:
 *
 *        motor 0   Wrist Motor
 *        motor 1   Elbow Motor
 *        motor 2   Inside Elbow Motor
 *
 *  Each one now has its OWN speed limiter and its OWN deadband, its
 *  own smoothing accumulator and its own committed target, all held in
 *  arrays indexed by the motor number. Nothing is shared between them,
 *  which is exactly why working one finger -- or dragging one slider --
 *  can no longer nudge the other two motors a single degree.
 *
 *  Speed is in degrees per FLEX_APPLY_MS tick, so with the 40 ms tick:
 *        1 =  25 deg/s (very slow, very smooth)
 *        4 = 100 deg/s (the old fixed value, a good default)
 *       12 = 300 deg/s (as fast as the servo will follow)
 *  Both numbers have a slider on the dashboard and are saved in flash. */
#define FLEX_STEP_DEF  FLEX_MAX_STEP   // startup speed limit, per motor
#define FLEX_STEP_MAX   12             // slider ceiling for the speed
#define FLEX_DEAD_DEF    5             // startup deadband, degrees. Raised
                                       // from 3: a flex sensor on a mux
                                       // wanders about 2 deg by itself, and
                                       // 3 was inside that wander, so the
                                       // motor chased the noise. 5 is
                                       // outside it -- this is the flex
                                       // half of the anti-jitter fix.
#define FLEX_DEAD_MAX   20             // slider ceiling for the deadband
#define FLEX_SMOOTH      5             // 1..8, per-motor extra smoothing.
                                       // 1 = none, 5 = calm, 8 = syrup.
void flexTick();
#define WIFI_LOW_POWER  1     // trims WiFi transmit power. WiFi bursts
                              // pull current spikes; on a weak supply
                              // that shows up as a servo twitch.

/* ---- MPU6050 balance assist -------------------------------------
 *  ANTI-JITTER, the second thing you asked for.
 *
 *  A balance loop is the classic cause of "every motor keeps twitching
 *  by a degree, over and over, and the robot never settles". The MPU6050
 *  is a noisy part: sitting perfectly still it still wanders by one to
 *  two degrees, and every wander used to buy a 1 degree trim step on
 *  eight or more joints. That is what you were watching.
 *
 *  Four things now stand between sensor noise and a servo, and none of
 *  them weakens a REAL correction:
 *
 *    1. HEAVY LOW-PASS on pitch and roll (BAL_LP). Noise is averaged
 *       away; a genuine lean still arrives, just a fraction later.
 *    2. A WIDE DEADBAND (BAL_DEADBAND) that is far outside that noise.
 *    3. CONFIRMATION (BAL_CONFIRM): the lean must be past the deadband,
 *       in the SAME direction, on that many consecutive samples before
 *       one single trim step is allowed. Random noise never manages it;
 *       a hand pushing the robot manages it immediately.
 *    4. HYSTERESIS on the way back (BAL_RELEASE) plus a longer minimum
 *       gap between steps (BAL_STEP_MS), so the trim cannot oscillate.
 *
 *  Net effect: standing still, the balancer issues ZERO servo writes.
 *  Push the robot and it corrects exactly as before.
 *===================================================================*/
#define USE_MPU          1    // compile the sensor in
#define BAL_ENABLE       0    // start with balancing OFF. Turn it on
                              // with  balance on  AFTER you have the
                              // bigger power supply.
#define BAL_LP           12   // 1..30 low-pass strength on pitch/roll.
                              // Higher = smoother and calmer, slower to
                              // notice a lean. 12 is a good compromise.
#define BAL_DEADBAND     6.0f // degrees of tilt before ANY correction.
                              // Deliberately well outside the MPU's own
                              // noise: this is the single number that
                              // decides whether a robot standing still
                              // holds still or hunts back and forth.
#define BAL_CONFIRM      3    // consecutive samples past the deadband,
                              // all leaning the same way, before one
                              // trim step is allowed. Kills noise dead.
#define BAL_RELEASE      2.0f // once back inside this, the trim is walked
                              // gently to zero and then stops dead.
#define BAL_HOLD_MS      900  // ignore tilt shorter than this (a bump)
#define BAL_MAX_DEG      8    // total authority, degrees
#define BAL_STEP_DEG     1    // degrees per correction
#define BAL_STEP_MS      260  // minimum gap between corrections
#define BAL_GAIN         0.35f
#define BAL_ONLY_IDLE    1    // freeze the trim while a routine runs
#define BAL_SETTLE_MS    600  // must be idle this long before trimming
#define BAL_GIVEUP_MS    6000 // if saturated this long, switch off and
                              // say so, rather than fight forever
#define FALL_DEG         45   // a genuine fall, while the robot is IDLE
#define FALL_HOLD_MS     250
/*  Intentional movement is NOT a fall. Push-ups reach -88 deg of pitch
 *  on purpose, and the old code read that as falling and stopped the
 *  routine. While a command is running the limit becomes
 *  FALL_DEG_MOVING and, unless you switch TILT_GUARD_MOVING on, the
 *  robot is never stopped by tilt at all. Idle protection is unchanged,
 *  so a real fall while standing is still caught.                    */
#define FALL_DEG_MOVING  100
#define FALL_HOLD_MOVING_MS 900
#define TILT_GUARD_MOVING  0
#define BAL_RESUME_MS    1200 // settle time before balance switches back on

/* ---- OLED face -------------------------------------------------- */
#define USE_OLED         1
#define OLED_SDA         32
#define OLED_SCL         33
#define OLED_ADDR        0x3C  // some modules are 0x3D
#define OLED_I2C_HZ      400000
#define OLED_FPS_MS      110   // face refresh interval
#define OLED_BLINK_MS    4200  // how often the idle face blinks

/*====================================================================
 *  1f. SOUND  (MAX98357A I2S amplifier -- see the wiring note below)
 *
 *  WHY THIS CHIP: the MAX98357A is the smallest thing that sounds
 *  good on an ESP32. It is a class-D amplifier with an I2S input, so
 *  it contains its own DAC -- you feed it three digital wires and it
 *  drives a speaker directly. No analogue output pin is needed, which
 *  matters here because GPIO25 (the ESP32's own DAC pin) is already
 *  running your head servo. The board is about 15 x 18 mm.
 *
 *  WIRING  (MAX98357A  ->  ESP32)
 *    VIN    -> 5 V        (the servo 5-6 V rail is fine, or ESP32 VIN)
 *    GND    -> GND        (must share ground with the ESP32)
 *    DIN    -> GPIO 14
 *    BCLK   -> GPIO 26
 *    LRC    -> GPIO 27
 *    GAIN   -> leave unconnected for 9 dB, or GND for 12 dB (louder),
 *              or to VIN through 100k for 6 dB (quieter).
 *    SD     -> leave unconnected. Tying it to GND mutes the amp; the
 *              board pulls it up on its own, so do not touch it.
 *    Speaker: 4 ohm or 8 ohm, 1-3 W, to the + and - screw terminals.
 *             A 8 ohm 0.5 W speaker is plenty -- you said quiet is
 *             fine, and an 8 ohm load makes the amp run cooler.
 *
 *  Put a 100 uF capacitor across the amp's VIN and GND. The amp draws
 *  current in bursts and your servo rail is already sagging; without
 *  the capacitor the sound crackles when the legs move.
 *
 *  NO SD CARD AND NO FILE UPLOADS. Every sound is generated by maths
 *  at run time (section 10), so there is nothing to copy anywhere.
 *===================================================================*/
#define USE_AUDIO        1     // 0 = compile the sound out completely
#define I2S_DIN         14     // amp DIN
/*  Optional: the MAX98357A's SD / shutdown pin. Leave at -1 if you have
 *  not wired it (nothing changes, no hardware needed). If you do wire
 *  it to a spare GPIO, put the number here and the amplifier is shut
 *  down completely whenever the robot is not making a sound.        */
#define I2S_SD_PIN      -1
#define I2S_BCLK        26     // amp BCLK
#define I2S_LRC         27     // amp LRC
#define AUDIO_SR     22050     // sample rate. 22050 is plenty for this
/* ---- future on-robot microphone reservation ---------------------
 * Leave this at 0 for the current build. Phone speech recognition is
 * the practical voice-input path today; it sends recognised text to
 * /run and never consumes ESP32 audio input resources.
 *
 * These pins are deliberately separate from the MAX98357A output.
 * A future I2S microphone (for example INMP441) needs I2S RX plus a
 * wake-word/speech-recognition stage; the microphone alone is not
 * speech recognition. GPIO34 is input-only, ideal for microphone DIN.
 */
#define FUTURE_MIC_ENABLE  0
#define FUTURE_MIC_DIN    34
#define FUTURE_MIC_BCLK   18
#define FUTURE_MIC_LRC    19
/*  ---- PART 4: LOUDER --------------------------------------------------
 *  Two things were holding the volume down, and neither of them was the
 *  MAX98357A:
 *
 *    1. The sketch started at 45 %, so out of the box you were hearing
 *       LESS THAN HALF of what the amplifier can already do. Startup is
 *       now 100 %.
 *    2. The slider stopped at 100 %, and 100 % meant "exactly full
 *       scale", so there was no headroom to ask for more. The slider now
 *       goes to VOL_MAX (200 %). Above 100 % the samples are pushed past
 *       full scale and then rounded back down by softClip() below, which
 *       is how every guitar amp and every mastering limiter in the world
 *       gets louder without turning into a buzz: quiet parts get pulled
 *       up a lot, loud peaks get squeezed gently instead of being
 *       chopped square. Hard clipping is what actually sounds like
 *       broken distortion, and softClip() is exactly what avoids it.
 *
 *  Practical guide:  100 = clean and already much louder than before.
 *  130..150 = noticeably louder still, effectively no distortion.
 *  200 = as loud as this hardware will go, deliberately compressed.
 *  The MAX98357A has a fixed 9 dB (or strapped 12/15 dB) gain and its
 *  own limiter, so nothing here can damage the amplifier -- but a very
 *  small speaker on a weak 5 V supply may rattle at 200. If it does,
 *  slide back to about 150; the setting is live and takes effect at
 *  once.                                                              */
#define AUDIO_VOL      100     // startup volume 0..VOL_MAX. Change live
                               // with  vol 150  or the dashboard slider
#define VOL_MAX        200     // slider / command ceiling. 100 = full
                               // scale, above that softClip() limits.
#define SOFT_KNEE    22000     // below this, samples pass through
                               // untouched; above it they are rounded
                               // off smoothly instead of clipped square
#define SOFT_CEIL    32200     // never send more than this to the amp
#define AUDIO_CORE       0     // sound runs on core 0 so it plays
                               // WHILE the servos move on core 1

/*====================================================================
 *  1g. HEAD GESTURES
 *
 *  The head is the one servo wired straight to the ESP32 rather than
 *  to the PCA9685, because the PCA9685 has exactly 16 outputs and this
 *  robot has 17 joints. See section 2 for the wiring.
 *
 *  A gesture is a short list of head angles that the sketch walks
 *  through in the background, so the head can nod while the arms are
 *  busy. Gestures never block.
 *===================================================================*/
#define HEAD_CENTRE     90     // straight ahead
#define HEAD_MIN        45     // do not drive the head past these --
#define HEAD_MAX       135     // widen only if your neck can take it
#define HEAD_TICK_MS    22     // how often the gesture engine steps
#define HEAD_MAX_STEP    5     // degrees per tick, so it glides


/* ---- optional cloud --------------------------------------------- */
#define USE_BLYNK       0
#define USE_SINRIC      0

#if USE_WEB || USE_BLYNK || USE_SINRIC
  #include <WiFi.h>
  #define NEED_WIFI 1
#else
  #define NEED_WIFI 0
#endif
#if USE_WEB
  #include <WebServer.h>
#endif

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);
Preferences prefs;
#if USE_WEB
  WebServer server(80);
#endif
#if USE_OLED
  TwoWire OledBus(1);
#endif

/*  Counts every pulse actually put on the wire. The whole point of
 *  this version is that this number stops rising when the robot is
 *  standing still.                                                  */
volatile uint32_t gWrites  = 0;
volatile uint32_t gSkipped = 0;

/*====================================================================
 *  TYPES THAT APPEAR IN FUNCTION SIGNATURES
 *
 *  These two have to be declared up here, above the first function in
 *  the sketch, and NOT next to the code that uses them.
 *
 *  Why: the Arduino IDE silently generates a prototype for every
 *  function in a .ino and pastes them all in just above your first
 *  function. If a prototype mentions a type that is still defined
 *  further down the file, you get errors that point at lines you never
 *  wrote -- "'Face' was not declared in this scope". Defining the
 *  types first makes that impossible. Do not move them lower.
 *===================================================================*/

/*  Which picture the OLED is showing. */
enum Face {
  F_BOOT, F_IDLE, F_HAPPY, F_BUSY, F_WALK, F_SIT, F_SLEEP, F_ALERT,
  /*  moods added in v6 -- the originals keep their positions so
   *  nothing that referred to them by name had to change.           */
  F_ANGRY, F_SAD, F_SURPRISE, F_LOVE, F_WINK, F_DIZZY, F_COOL,
  F_DETERMINED, F_PROUD, F_CURIOUS, F_BYE, F_MUSIC
};

/*  One instant of the walk, as five numbers. See section 11.
 *    hl/hr  hip pitch, degrees forward from standing (left/right)
 *    kl/kr  knee bend, degrees (0 = straight)
 *    r      roll: + leans onto the RIGHT foot, - onto the left        */
struct GaitPose { float hl, hr, kl, kr, r; };

/*  Which sound to play. Kept up here with the other types because the
 *  IDE hoists prototypes that mention it. See the note above.        */
enum Snd {
  S_NONE = 0,
  S_BOOT,        // power-up chime
  S_READY,       // finished booting, standing
  S_BEEP,        // plain acknowledge
  S_HAPPY,       // bright little rise
  S_GIGGLE,      // playful stutter
  S_HI,          // "hello!" two-tone wave
  S_BYE,         // falling goodbye
  S_ANGRY,       // harsh buzzing growl
  S_SAD,         // slow droop
  S_SURPRISE,    // fast upward chirp
  S_LOVE,        // soft warble with a heartbeat
  S_SLEEPY,      // yawn, dropping away
  S_WINK,        // single cheeky blip
  S_DIZZY,       // wobbling siren
  S_COOL,        // smooth confident two-note
  S_GRUNT,       // low effort noise, for push-ups
  S_WALK,        // footstep tick
  S_SIT,         // descending settle
  S_STAND,       // ascending lift
  S_KARATE,      // sharp attack shout
  S_BOW,         // polite descending pair
  S_FLEX,        // proud fanfare
  S_MUSIC,       // dance loop riff
  S_WIN,         // victory arpeggio
  S_CURIOUS,     // questioning up-tick
  S_ERROR,       // "I did not understand that"
  S_COUNT
};

/*  A background head movement. Never blocks; see section 12b.        */
enum HeadGest {
  HG_NONE = 0,
  HG_NOD,        // yes
  HG_SHAKE,      // no
  HG_TILT,       // curious lean
  HG_LOOK_L,     // glance left
  HG_LOOK_R,     // glance right
  HG_SCAN,       // slow sweep, both ways
  HG_BOW_H,      // head drops and comes back up
  HG_PERK,       // snaps up alert
  HG_DROOP,      // sinks down sadly
  HG_BEAT,       // bobs to a beat, for dancing
  HG_SHIVER,     // fast small shake, for anger
  HG_COUNT
};


/*====================================================================
 *  2. HEAD SERVO  (LEDC -- works on ESP32 core 2.x and 3.x)
 *===================================================================*/
void headInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(HEAD_PIN, 50, 16);
#else
  ledcSetup(HEAD_LEDC_CH, 50, 16);
  ledcAttachPin(HEAD_PIN, HEAD_LEDC_CH);
#endif
}
static inline void headWriteDuty(uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(HEAD_PIN, duty);
#else
  ledcWrite(HEAD_LEDC_CH, duty);
#endif
}
static inline int headDutyFor(int a) {
  long us = HEAD_US_MIN + (long)a * (HEAD_US_MAX - HEAD_US_MIN) / 180;
  return (int)((us * 65536UL) / 20000UL);
}
void headOff() { headWriteDuty(0); }

/*====================================================================
 *  2b. SOUND ENGINE
 *
 *  Every sound is built out of "notes". A note is a frequency that
 *  slides from f1 to f2 over ms milliseconds using one of six
 *  waveforms. That is enough to make chimes, growls, sirens, footsteps
 *  and fanfares without storing a single audio file.
 *
 *  It runs in its own task pinned to core 0. The task blocks inside
 *  i2s write() waiting for the amplifier to want more samples, which
 *  is a free timer -- it costs no CPU on core 1 where your servos are.
 *  So soundPlay() returns instantly and the robot keeps moving.
 *
 *  Works on both ESP32 core versions: core 3.x ships the new I2S
 *  driver, core 2.x the old one, and __has_include picks the right
 *  one for you.
 *===================================================================*/

#if USE_AUDIO

#if __has_include(<driver/i2s_std.h>)
  #define AUDIO_NEW_API 1
  #include <driver/i2s_std.h>
  static i2s_chan_handle_t gTx = NULL;
#else
  #define AUDIO_NEW_API 0
  #include <driver/i2s.h>
#endif

/*  waveforms */
#define W_SQ   0    // square: bright, chiptune
#define W_SIN  1    // sine: soft, round
#define W_SAW  2    // saw: buzzy, harsh
#define W_NOI  3    // noise: air, footsteps, effort
#define W_WBL  4    // sine with vibrato: cute, wobbly
#define W_TRI  5    // triangle: mellow flute

struct Note {
  uint16_t f1;      // start frequency, Hz (0 = silence)
  uint16_t f2;      // end frequency, Hz (slides there)
  uint16_t ms;      // length
  uint8_t  wave;    // W_*
  uint8_t  vol;     // 0..255, relative
};

/* ---- the sounds -------------------------------------------------- */
/*  Musical reference, so the tables read sensibly:
 *  C4 262  D4 294  E4 330  F4 349  G4 392  A4 440  B4 494
 *  C5 523  E5 659  G5 784  C6 1047  E6 1319  G6 1568           */

const Note N_BOOT[] = {
  {  262,  262,  70, W_SQ,  170 }, {  392,  392,  70, W_SQ,  185 },
  {  523,  523,  80, W_SIN, 200 }, {  784,  784,  90, W_SIN, 205 },
  { 1047, 1047, 110, W_SIN, 190 }, {    0,    0,  50, W_SQ,    0 },
  { 1047, 1568, 200, W_WBL, 175 },
};
const Note N_READY[] = {
  {  784,  784,  70, W_TRI, 190 }, { 1047, 1047,  70, W_TRI, 195 },
  { 1319, 1319, 130, W_SIN, 190 },
};
const Note N_BEEP[]  = { { 880, 880, 55, W_SQ, 165 } };
const Note N_HAPPY[] = {
  {  523,  659,  80, W_SIN, 195 }, {  784,  784,  70, W_SIN, 195 },
  { 1047, 1047, 120, W_WBL, 185 },
};
const Note N_GIGGLE[] = {
  { 900, 1150, 55, W_SIN, 180 }, {   0,   0, 35, W_SQ,   0 },
  { 950, 1250, 55, W_SIN, 180 }, {   0,   0, 35, W_SQ,   0 },
  { 1000,1350, 60, W_SIN, 180 }, {   0,   0, 30, W_SQ,   0 },
  { 1100,1500, 80, W_WBL, 170 },
};
const Note N_HI[] = {
  {  659,  988, 110, W_WBL, 195 }, {  988,  784,  90, W_WBL, 190 },
  {  988, 1319, 160, W_SIN, 195 },
};
const Note N_BYE[] = {
  { 1047, 1047,  90, W_SIN, 195 }, {  784,  784,  90, W_SIN, 190 },
  {  659,  440, 260, W_WBL, 180 },
};
const Note N_ANGRY[] = {
  {  150,  110, 180, W_SAW, 225 }, {  190,  120, 150, W_SAW, 225 },
  {  120,   80, 260, W_SAW, 235 }, {   90,   60, 200, W_SAW, 215 },
};
const Note N_SAD[] = {
  {  494,  440, 200, W_TRI, 180 }, {  392,  349, 220, W_TRI, 175 },
  {  330,  247, 420, W_WBL, 165 },
};
const Note N_SURPRISE[] = {
  {  440, 1568,  90, W_SQ,  210 }, { 1568, 1568,  70, W_SIN, 200 },
  { 1319, 1760, 130, W_WBL, 190 },
};
const Note N_LOVE[] = {
  {  659,  659, 120, W_WBL, 185 }, {  784,  784, 120, W_WBL, 185 },
  {  988,  988, 100, W_SIN, 190 }, {  784,  784,  90, W_SIN, 180 },
  {  659,  880, 300, W_WBL, 175 },
};
const Note N_SLEEPY[] = {
  {  392,  523, 260, W_TRI, 175 }, {  523,  330, 380, W_WBL, 165 },
  {  294,  196, 460, W_SIN, 150 },
};
const Note N_WINK[] = {
  { 1319, 1760,  60, W_SQ,  185 }, { 1760, 1319,  70, W_SIN, 175 },
};
const Note N_DIZZY[] = {
  {  400,  900, 150, W_WBL, 195 }, {  900,  400, 150, W_WBL, 195 },
  {  420,  950, 140, W_WBL, 190 }, {  950,  380, 160, W_WBL, 190 },
  {  380,  600, 220, W_SIN, 175 },
};
const Note N_COOL[] = {
  {  330,  330, 120, W_TRI, 190 }, {  440,  440, 110, W_TRI, 190 },
  {  554,  554, 220, W_SIN, 185 },
};
const Note N_GRUNT[] = {
  {  110,   80, 150, W_NOI, 200 }, {   90,  130, 130, W_SAW, 185 },
};
const Note N_WALK[]  = { { 200, 90, 45, W_NOI, 175 } };
const Note N_SIT[]   = {
  {  659,  494, 130, W_TRI, 185 }, {  440,  330, 150, W_TRI, 180 },
  {  294,  247, 220, W_SIN, 170 },
};
const Note N_STAND[] = {
  {  247,  330, 130, W_TRI, 180 }, {  392,  523, 140, W_TRI, 190 },
  {  659,  784, 200, W_SIN, 190 },
};
const Note N_KARATE[] = {
  { 1400,  300,  90, W_SAW, 230 }, {  200,  200, 110, W_NOI, 215 },
  {  700, 1500, 120, W_SQ,  205 },
};
const Note N_BOW[] = {
  {  523,  392, 180, W_TRI, 185 }, {  330,  262, 260, W_SIN, 175 },
};
const Note N_FLEX[] = {
  {  523,  523,  90, W_SQ,  200 }, {  659,  659,  90, W_SQ,  200 },
  {  784,  784,  90, W_SQ,  205 }, { 1047, 1047, 240, W_SIN, 205 },
};
const Note N_MUSIC[] = {
  {  523,  523, 110, W_SQ,  195 }, {  784,  784, 110, W_SQ,  195 },
  {  659,  659, 110, W_SQ,  190 }, {  988,  988, 110, W_SQ,  195 },
  {  880,  880, 110, W_SQ,  190 }, {  659,  659, 110, W_SQ,  185 },
  {  784,  784, 110, W_SQ,  190 }, {  523,  523, 160, W_SIN, 195 },
};
const Note N_WIN[] = {
  {  523,  523,  90, W_SQ,  200 }, {  659,  659,  90, W_SQ,  200 },
  {  784,  784,  90, W_SQ,  200 }, { 1047, 1047, 110, W_SIN, 210 },
  {  784,  784,  80, W_SIN, 190 }, { 1047, 1568, 280, W_WBL, 200 },
};
const Note N_CURIOUS[] = {
  {  494,  494,  90, W_TRI, 180 }, {  587,  784, 170, W_WBL, 185 },
};
const Note N_ERROR[] = {
  {  330,  330, 110, W_SAW, 195 }, {    0,    0,  50, W_SQ,    0 },
  {  247,  185, 200, W_SAW, 195 },
};

struct SoundDef { const Note* n; uint8_t count; };
#define SD(x) { x, (uint8_t)(sizeof(x) / sizeof(Note)) }
const SoundDef SOUNDS[S_COUNT] = {
  { NULL, 0 },     // S_NONE
  SD(N_BOOT),     SD(N_READY),   SD(N_BEEP),   SD(N_HAPPY),
  SD(N_GIGGLE),   SD(N_HI),      SD(N_BYE),    SD(N_ANGRY),
  SD(N_SAD),      SD(N_SURPRISE),SD(N_LOVE),   SD(N_SLEEPY),
  SD(N_WINK),     SD(N_DIZZY),   SD(N_COOL),   SD(N_GRUNT),
  SD(N_WALK),     SD(N_SIT),     SD(N_STAND),  SD(N_KARATE),
  SD(N_BOW),      SD(N_FLEX),    SD(N_MUSIC),  SD(N_WIN),
  SD(N_CURIOUS),  SD(N_ERROR),
};
#undef SD

/* ---- state shared with the audio task ---------------------------- */
volatile uint8_t  gSndReq   = S_NONE;   // written by anyone, read by task
volatile uint8_t  gSndNow   = S_NONE;   // what is actually sounding
volatile uint8_t  gVol      = AUDIO_VOL;
volatile bool     gMute     = false;
volatile bool     gSndLoop  = false;    // keep repeating (dance, walk)
volatile uint32_t gSndCount = 0;        // sounds started, for the tests
bool              gAudioOk  = false;

int16_t  gSine[256];                    // one quarter-cycle would do,
                                        // but 256 entries is only 512 B

/*  ---- PART 4: the soft limiter --------------------------------------
 *  This is the whole reason the volume can now go above "full scale"
 *  without turning into a buzz. Anything quieter than SOFT_KNEE is left
 *  completely alone, so normal playback is bit-for-bit what it always
 *  was. Only the peaks that would have run off the end of a 16-bit
 *  sample get bent back, and they are bent along a curve that flattens
 *  out towards SOFT_CEIL instead of being sliced off square. A sliced
 *  peak is a step, a step is full of harmonics, and that is what a
 *  cheap speaker reproduces as crackle. A rounded peak is not.        */
static inline int16_t softClip(int32_t s) {
  int32_t a = s < 0 ? -s : s;
  if (a <= SOFT_KNEE) return (int16_t)s;          // untouched
  int32_t over = a - SOFT_KNEE;                   // how far past the knee
  int32_t room = SOFT_CEIL - SOFT_KNEE;           // what is left above it
  /*  A simple compressive curve: over/(1+over/room)*... -- in integer
   *  maths, over*room/(over+room), which approaches  room  but never
   *  reaches it, no matter how hard it is driven.                     */
  int32_t sq = SOFT_KNEE + (over * room) / (over + room);
  if (sq > SOFT_CEIL) sq = SOFT_CEIL;
  return (int16_t)(s < 0 ? -sq : sq);
}

/*  Ask for a sound. Safe to call from anywhere, including inside a
 *  servo routine -- it only sets a byte, it never waits.             */
void soundPlay(Snd s) {
  if (!gAudioOk || gMute) return;
  gSndLoop = false;
  gSndReq  = (uint8_t)s;
  gSndCount++;
}
void soundLoop(Snd s) {           // repeat until soundStop()
  if (!gAudioOk || gMute) return;
  gSndReq  = (uint8_t)s;
  gSndLoop = true;
  gSndCount++;
}
void soundStop() { gSndLoop = false; gSndReq = S_NONE; }

/* ---- the synthesiser --------------------------------------------- */
#define ABLK 128                        // samples generated per pass

static uint32_t nzState = 0x13579BDF;
static inline int16_t noiseSample() {
  nzState ^= nzState << 13; nzState ^= nzState >> 17; nzState ^= nzState << 5;
  return (int16_t)(nzState >> 16);
}

/*  Fill one block. Returns false when the current sound has finished
 *  and nothing is queued, so the caller can idle.                    */
bool audioFill(int16_t* out, int n) {
  static uint8_t  sid  = S_NONE;   // sound being played
  static uint8_t  ni   = 0;        // note index within it
  static uint32_t np   = 0;        // samples done in this note
  static uint32_t nlen = 0;        // samples this note lasts
  static uint32_t ph   = 0;        // phase accumulator

  /*  a new request pre-empts whatever is sounding */
  uint8_t req = gSndReq;
  if (req != sid) {
    sid = req; ni = 0; np = 0; nlen = 0; ph = 0;
    gSndNow = sid;
  }
  if (sid == S_NONE || sid >= S_COUNT || SOUNDS[sid].count == 0) {
    memset(out, 0, n * sizeof(int16_t));
    return false;
  }

  const SoundDef& sd = SOUNDS[sid];
  /*  PART 4: the ceiling is VOL_MAX, not 100. Everything above 100 is
   *  extra gain that softClip() keeps under control.                  */
  int vol = gVol; if (vol > VOL_MAX) vol = VOL_MAX;

  for (int i = 0; i < n; i++) {
    if (np >= nlen) {                       // advance to the next note
      if (nlen) ni++;
      if (ni >= sd.count) {                 // sound finished
        if (gSndLoop) { ni = 0; }
        else {
          gSndReq = S_NONE; gSndNow = S_NONE; sid = S_NONE;
          for (; i < n; i++) out[i] = 0;
          return false;
        }
      }
      nlen = (uint32_t)sd.n[ni].ms * AUDIO_SR / 1000;
      if (!nlen) nlen = 1;
      np = 0;
    }

    const Note& nt = sd.n[ni];
    float t = (float)np / (float)nlen;                 // 0..1 in note
    float f = (float)nt.f1 + ((float)nt.f2 - (float)nt.f1) * t;

    int16_t s = 0;
    if (f > 1.0f && nt.vol) {
      if (nt.wave == W_WBL)                            // gentle vibrato
        f *= 1.0f + 0.045f * gSine[(uint8_t)((np * 24 * 256 / AUDIO_SR) & 0xFF)] / 32767.0f;
      ph += (uint32_t)(f * (4294967296.0f / AUDIO_SR));
      switch (nt.wave) {
        case W_SQ:  s = (ph & 0x80000000UL) ? 26000 : -26000;            break;
        case W_SAW: s = (int16_t)((int32_t)(ph >> 16) - 32768);           break;
        case W_NOI: s = noiseSample();                                   break;
        case W_TRI: {
          int32_t u = (int32_t)(ph >> 16);                    // 0..65535
          s = (int16_t)((u < 32768 ? u * 2 : (65535 - u) * 2) - 32768);
          break;
        }
        default:    s = gSine[(uint8_t)(ph >> 24)];                      break;
      }
      /*  envelope: short attack and release so nothing clicks */
      float env = 1.0f;
      uint32_t atk = AUDIO_SR / 400;                   // 2.5 ms
      uint32_t rel = nlen / 5; if (rel > AUDIO_SR / 40) rel = AUDIO_SR / 40;
      if (np < atk)             env = (float)np / (float)atk;
      if (nlen - np < rel && rel) env *= (float)(nlen - np) / (float)rel;
      s = softClip((int32_t)((float)s * env * (nt.vol / 255.0f)
                                       * (vol / 100.0f)));
    }
    out[i] = s;
    np++;
  }
  return true;
}

/* ---- I2S plumbing ------------------------------------------------ */
bool audioInit() {
  for (int i = 0; i < 256; i++)
    gSine[i] = (int16_t)(30000.0f * sinf(2.0f * (float)PI * i / 256.0f));

#if AUDIO_NEW_API
  i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  cc.dma_desc_num  = 6;
  cc.dma_frame_num = 240;
  if (i2s_new_channel(&cc, &gTx, NULL) != ESP_OK) return false;
  i2s_std_config_t sc = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SR),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                    I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_BCLK,
      .ws   = (gpio_num_t)I2S_LRC,
      .dout = (gpio_num_t)I2S_DIN,
      .din  = I2S_GPIO_UNUSED,
      .invert_flags = { false, false, false },
    },
  };
  if (i2s_channel_init_std_mode(gTx, &sc) != ESP_OK) return false;
  if (i2s_channel_enable(gTx) != ESP_OK) return false;
#else
  i2s_config_t cfg = {};
  cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate          = AUDIO_SR;
  cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags     = 0;
  cfg.dma_buf_count        = 6;
  cfg.dma_buf_len          = 240;
  cfg.use_apll             = false;
  cfg.tx_desc_auto_clear   = true;
  if (i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL) != ESP_OK) return false;
  i2s_pin_config_t pins = {};
  pins.bck_io_num   = I2S_BCLK;
  pins.ws_io_num    = I2S_LRC;
  pins.data_out_num = I2S_DIN;
  pins.data_in_num  = -1;          // I2S_PIN_NO_CHANGE
  if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) return false;
#endif
  return true;
}

/*  Zero the DMA ring as well as writing silence. Without this the
 *  MAX98357A keeps re-playing whatever was left in the buffers, which
 *  is exactly the random idle beeping.                               */
/*  THE IDLE-BEEPING FIX.
 *  Leaving I2S clocked with nothing to send makes the MAX98357A chew on
 *  whatever is left in the DMA ring, which is where the beeping and the
 *  buzzing came from about a second after a clip ended. So: flush a
 *  whole ring of real silence, clear the DMA, then STOP the peripheral.
 *  It is started again automatically the moment anything wants to play.
 *  If you have wired the amplifier's SD pin, set I2S_SD_PIN and it is
 *  held low while parked as well.                                    */
volatile bool gI2sUp = false;

void audioWake() {
  if (!gAudioOk || gI2sUp) return;
#if AUDIO_NEW_API
  i2s_channel_enable(gTx);
#else
  i2s_start(I2S_NUM_0);
#endif
#if I2S_SD_PIN >= 0
  digitalWrite(I2S_SD_PIN, HIGH);
#endif
  gI2sUp = true;
}

void audioPark() {
  if (!gAudioOk || !gI2sUp) return;
  static int16_t z[240];
  memset(z, 0, sizeof(z));
  size_t w = 0;
  for (int k = 0; k < 7; k++) {          /* flush the whole DMA ring */
#if AUDIO_NEW_API
    i2s_channel_write(gTx, z, sizeof(z), &w, 40 / portTICK_PERIOD_MS);
#else
    i2s_write(I2S_NUM_0, z, sizeof(z), &w, 40 / portTICK_PERIOD_MS);
#endif
  }
#if AUDIO_NEW_API
  i2s_channel_disable(gTx);
#else
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_stop(I2S_NUM_0);
#endif
#if I2S_SD_PIN >= 0
  digitalWrite(I2S_SD_PIN, LOW);         /* amplifier shut down: dead quiet */
#endif
  gI2sUp = false;
}

static inline void audioSilence() {
  audioPark();
}

static inline void audioWrite(const int16_t* buf, size_t samples) {
  size_t wrote = 0;
  audioWake();                 /* no clocks unless there is something to say */
#if AUDIO_NEW_API
  i2s_channel_write(gTx, buf, samples * sizeof(int16_t), &wrote, portMAX_DELAY);
#else
  i2s_write(I2S_NUM_0, buf, samples * sizeof(int16_t), &wrote, portMAX_DELAY);
#endif
}

/* ---- recorded voice clip ------------------------------------------
 *  8-bit unsigned mono at 11025 Hz, which is exactly half of AUDIO_SR,
 *  so playback just emits every byte twice and needs no resampling.
 *  The buffer is malloc'd only when you actually record something.   */
uint8_t*      gVBuf = NULL;
uint32_t      gVCap = 0, gVLen = 0;
volatile bool gVOn  = false;
volatile uint32_t gVPos = 0;

bool vAlloc(uint32_t want) {
  if (gVBuf && gVCap >= want) return true;
  if (gVBuf) { free(gVBuf); gVBuf = NULL; gVCap = 0; }
  uint32_t n = want;
  while (n >= 4096) {
    gVBuf = (uint8_t*)malloc(n);
    if (gVBuf) { gVCap = n; return true; }
    n /= 2;
  }
  return false;
}

void vSave() {
  if (!gVBuf || !gVLen) return;
  uint32_t nkeep = gVLen > VNVS_MAX ? VNVS_MAX : gVLen;
  prefs.putBytes("vclip", gVBuf, nkeep);
}
void vLoad() {
  size_t have = prefs.getBytesLength("vclip");
  if (have < 512) return;
  if (!vAlloc((uint32_t)have)) return;
  prefs.getBytes("vclip", gVBuf, have);
  gVLen = (uint32_t)have;
  Serial.printf("voice: %u bytes of recording restored from flash\n", (unsigned)have);
}
void vClear() {
  gVOn = false; gVLen = 0; gVPos = 0;
  prefs.remove("vclip");
}

/*  The task. When nothing is playing it sleeps instead of pushing
 *  silence, so an idle robot is a silent robot and the amplifier is
 *  not being fed at all.                                             */
void audioTask(void*) {
  int16_t buf[ABLK];
  for (;;) {
    if (gVOn && gVBuf && gVLen) {
      /*  PART 4: your recorded clip is 8-bit, so it only ever swings
       *  +/-128 counts. The old scale of 180 left it about 3 dB below
       *  full scale even at "100 %"; 250 uses the whole 16-bit range and
       *  softClip() catches the peaks, so the clip is now clearly louder
       *  without the crackle that a hard clip would add.               */
      int vol = gVol; if (vol > VOL_MAX) vol = VOL_MAX;
      if (gMute) vol = 0;
      for (int i = 0; i < ABLK; i++) {
        uint32_t pp = gVPos + (uint32_t)(i >> 1);
        uint8_t  sm = (pp < gVLen) ? gVBuf[pp] : 128;
        buf[i] = softClip(((int32_t)sm - 128) * 250 * vol / 100);
      }
      gVPos += (uint32_t)(ABLK >> 1);
      audioWrite(buf, ABLK);
      if (gVPos >= gVLen) {
        gVOn = false;
        memset(buf, 0, sizeof(buf));
        audioWrite(buf, ABLK);
        audioSilence();
      }
      continue;
    }
    if (gSndReq == S_NONE && gSndNow == S_NONE) {
      if (gI2sUp) audioPark();          /* silence means silence */
      vTaskDelay(pdMS_TO_TICKS(12));
      continue;
    }
    bool more = audioFill(buf, ABLK);
    audioWrite(buf, ABLK);
    if (!more) {
      /*  flush a little silence so the last note does not repeat out
       *  of the DMA buffer, then go quiet                            */
      memset(buf, 0, sizeof(buf));
      audioWrite(buf, ABLK);
      audioWrite(buf, ABLK);
      audioSilence();            // and leave the amplifier fed with nothing
    }
  }
}

/*  ---- AUDIO SELF-TEST -------------------------------------------------
 *  "The audio is not working" has two completely different causes and
 *  only one of them is in this sketch, so the sketch now tells you which
 *  one you have instead of leaving you to guess.
 *
 *  It writes a plain 660 Hz sine straight into the I2S DMA -- no note
 *  engine, no envelope, no queue, no task -- and prints how many bytes
 *  the driver accepted. It runs BEFORE the audio task is created, so
 *  nothing can be competing for the peripheral while it does.
 *
 *      bytes = 0   the I2S driver itself is refusing samples. Nothing
 *                  downstream can possibly make a sound; look for
 *                  another library holding I2S_NUM_0.
 *      bytes > 0   the ESP32 is genuinely clocking audio out of GPIO 14
 *                  and the fault is after the pins: wiring, GND, the
 *                  amplifier's SD pin pulled low, VIN on 3V3 instead of
 *                  5 V, or the speaker.
 *
 *  A short beep at boot is also the fastest possible confirmation that
 *  the volume is not sitting at zero and that mute is not on.          */
void audioSelfTest() {
  if (!gAudioOk) {
    Serial.println(F("audio SELF-TEST: skipped -- I2S never started"));
    return;
  }
  const int N = 256;
  int16_t   buf[N];
  int      vol = gVol; if (vol > VOL_MAX) vol = VOL_MAX;
  uint32_t ph  = 0;
  uint32_t inc = (uint32_t)(660.0f * (4294967296.0f / AUDIO_SR));
  size_t   tot = 0;

  audioWake();
  for (int blk = 0; blk < (AUDIO_SR / N / 3); blk++) {      /* ~1/3 second */
    for (int i = 0; i < N; i++) {
      ph += inc;
      buf[i] = softClip((int32_t)gSine[(uint8_t)(ph >> 24)] * vol / 100);
    }
    size_t w = 0;
#if AUDIO_NEW_API
    i2s_channel_write(gTx, buf, sizeof(buf), &w, 200 / portTICK_PERIOD_MS);
#else
    i2s_write(I2S_NUM_0, buf, sizeof(buf), &w, 200 / portTICK_PERIOD_MS);
#endif
    tot += w;
  }
  audioPark();

  Serial.printf("audio SELF-TEST: 660 Hz tone, %u bytes accepted by I2S, "
                "volume %d, %s\n", (unsigned)tot, (int)gVol,
                gMute ? "MUTED" : "not muted");
  if (!tot)
    Serial.println(F("audio SELF-TEST: nothing was written. The I2S driver is\n"
                     "  refusing samples -- something else has I2S_NUM_0."));
  else
    Serial.println(F("audio SELF-TEST: samples ARE leaving the ESP32. If you\n"
                     "  heard nothing, the fault is after the pins:\n"
                     "    DIN -> GPIO14, BCLK -> GPIO26, LRC -> GPIO27\n"
                     "    VIN -> 5V (not 3V3), GND shared with the ESP32,\n"
                     "    SD  -> leave floating or tie HIGH, never to GND,\n"
                     "    speaker across the two screw terminals, 4-8 ohm."));
}

void audioStart() {
  gAudioOk = audioInit();
  if (!gAudioOk) {
    Serial.println(F("audio: I2S would not start -- carrying on silently"));
    return;
  }
#if I2S_SD_PIN >= 0
  pinMode(I2S_SD_PIN, OUTPUT);
  digitalWrite(I2S_SD_PIN, HIGH);   /* enable the amplifier before testing */
#endif
  gI2sUp = true;
  audioPark();                  /* boot up silent, not humming */
  audioSelfTest();              /* say something, and say why if it cannot */
  xTaskCreatePinnedToCore(audioTask, "audio", 3072, NULL, 2, NULL, AUDIO_CORE);
  Serial.printf("audio: MAX98357A on BCLK %d / LRC %d / DIN %d, volume %d\n",
                I2S_BCLK, I2S_LRC, I2S_DIN, (int)gVol);
}

#else   /* no audio compiled in */
void soundPlay(Snd)  {}
void soundLoop(Snd)  {}
void soundStop()     {}
void audioStart()    {}
uint8_t*      gVBuf = NULL;
uint32_t      gVCap = 0, gVLen = 0;
volatile bool gVOn = false;
volatile uint32_t gVPos = 0;
bool vAlloc(uint32_t) { return false; }
void vSave()  {}
void vLoad()  {}
void vClear() {}
volatile uint8_t  gVol      = 0;
volatile bool     gMute     = true;
volatile uint32_t gSndCount = 0;
volatile uint8_t  gSndNow   = 0;
bool              gAudioOk  = false;
#endif  /* USE_AUDIO */


/*====================================================================
 *  3. ABORT / IO PLUMBING
 *  Declared before the driver because the rate limiter pumps IO.
 *===================================================================*/
volatile bool gAbort   = false;
bool          gBusy    = false;
/*  Auto stand-up. Deliberately separate from the balance trim: the trim
 *  nudges biases a degree at a time, this one runs a whole routine, so
 *  they must never be active together.                                */
bool          gAutoUp    = false;   // feature armed from the dashboard
bool          gAutoUpRun = false;   // a recovery is in progress
uint8_t       gAupTries  = 0;
uint32_t      gAupAt     = 0;
bool          gRateLim = true;
uint16_t      gTimePct = SPEED_DEFAULT;
int16_t       gSpeedOverride = 0;
String        gCmd     = "";
String        gLineBuf = "";
float         gPitch = 0, gRoll = 0;
uint32_t      gLastMotionMs = 0;   // when a servo last actually moved
int16_t       gWalkHalf = WALK_HALFSTEPS;  // half-steps per  walk  command
int8_t        gStance   = 0;   // permanent extra foot separation, degrees.
                               // Applied as a bias, so it never shows up in
                               // the pose bookkeeping and never makes the
                               // robot think it has left the standing pose.

void pumpIO();
bool naptime(uint32_t ms);
static inline uint32_t scaled(uint32_t ms) {
  uint16_t p = gSpeedOverride > 0 ? (uint16_t)gSpeedOverride : gTimePct;
  return (uint32_t)((uint64_t)ms * p / 100);
}
#define DLY(ms)    do { if (!naptime(scaled(ms))) return; } while (0)
#define ABORTCHK() do { if (gAbort) return; } while (0)

/*====================================================================
 *  4. JOINT  --  the only place ESP32/PCA9685 detail lives
 *
 *  write()     logical angle, RATE LIMITED. Routines call this.
 *  writeNow()  logical angle, no rate limiting. The glide uses this
 *              because it already produces small steps.
 *
 *  Both go through send(), and send() is where the anti-jitter fix
 *  lives: if the pulse a servo is already being given is the pulse we
 *  are about to give it, we do not touch the bus at all.
 *
 *  Why that matters: writing the SAME value to a PCA9685 channel
 *  rewrites its ON/OFF registers part-way through the current PWM
 *  frame, which briefly deforms the pulse the servo is holding. Do
 *  that to 17 channels 50 times a second -- which the old glide did,
 *  for every joint, whether it was moving or not -- and the whole
 *  robot hums. Dropping the no-op writes is most of the cure.
 *
 *  bias  is a live offset applied AFTER inv and trim. The balance
 *  assist and the  stance  command steer it. It deliberately does not
 *  touch `last`, so pose bookkeeping stays honest.
 *===================================================================*/
struct Joint {
  const char* name;
  int8_t   ch;        // PCA9685 channel, or -1 for the head on GPIO
  bool     inv;       // true = send (180 - angle): reverses the servo
  int8_t   trim;      // fixed mechanical offset, applied after inv
  int16_t  last;      // last LOGICAL angle asked for
  int16_t  lastSent;  // last raw value actually put on the wire
  int8_t   bias;      // live offset (balance / stance)

  int physical(int a) const {
    if (inv) a = 180 - a;
    a += trim + bias;
    return constrain(a, 0, 180);
  }
  /*  The dedupe gate. Returns true if anything was sent. */
  bool send(int phys) {
    int raw = (ch >= 0) ? (int)map(phys, 0, 180, PCA_MIN, PCA_MAX)
                        : headDutyFor(phys);
#if DEDUPE_WRITES
    if (raw == lastSent) { gSkipped++; return false; }
#endif
    lastSent = raw;
    if (ch >= 0) pwm.setPWM(ch, 0, raw);
    else         headWriteDuty((uint32_t)raw);
    gWrites++;
    gLastMotionMs = millis();
    return true;
  }
  void writeNow(int a) {
    a = constrain(a, 0, 180);
    last = a;
    send(physical(a));
  }
  /*  Re-apply the current angle -- used after inv/trim/bias changes,
   *  where the logical angle has not moved but the pulse must.       */
  void refresh() { send(physical(last)); }

  /*  THE RATE LIMITER.  A routine can ask for a 120 degree jump; the
   *  servo still gets there, but fanned out into MAX_STEP_DEG pieces,
   *  so it is a fast move instead of a stall.  Costs nothing when the
   *  routine is already stepping 1-2 degrees at a time.              */
  void write(int a) {
    a = constrain(a, 0, 180);
    int from = last;
    int diff = a - from;
    if (gRateLim && abs(diff) > MAX_STEP_DEG) {
      int n = (abs(diff) + MAX_STEP_DEG - 1) / MAX_STEP_DEG;
      for (int k = 1; k < n; k++) {
        int v = from + diff * k / n;
        last = v;
        send(physical(v));
        pumpIO();
        delay(SUBSTEP_MS);
      }
    }
    last = a;
    send(physical(a));
  }
};

#define NJ 17

/*  ---- THE ONLY TABLE YOU SHOULD EVER NEED TO EDIT ----
 *  Flip `inv` for any servo that turns the wrong way. Or use the live
 *  flip <name>  command, then  save.
 *  Columns:  name  ch  inv  trim  startAngle  lastSent(-1)  bias(0)  */
Joint JOINTS[NJ] = {
  { "head", -1, false,  0,  90, -1, 0 },   // GPIO 25, not on the PCA9685
  { "lh1",   0, false,  0, 180, -1, 0 },
  { "lh2",   1, false,  0, 180, -1, 0 },
  { "lh3",   2, false,  0, 160, -1, 0 },
  { "ll1",   3, false,  0,  90, -1, 0 },
  { "ll2",   4, false,  0,  30, -1, 0 },   // NOT inverted -- confirmed against
                                           // your "Servo Initial Positions"
                                           // diagram: LL2 sits at 30 deg.
  { "ll3",   5, false,  0, 150, -1, 0 },
  { "ll4",   6, false,  0, 150, -1, 0 },
  { "ll5",   7, false,  0,  90, -1, 0 },
  { "rh1",   8, false,  0,   0, -1, 0 },
  { "rh2",   9, false,  0,   0, -1, 0 },
  { "rh3",  10, false,  0,  20, -1, 0 },
  { "rl1",  11, false,  0,  90, -1, 0 },
  { "rl2",  12, false,  0, 150, -1, 0 },
  { "rl3",  13, false,  0,  30, -1, 0 },
  { "rl4",  14, false,  0,  30, -1, 0 },
  { "rl5",  15, false,  0,  90, -1, 0 }
};

/*  Named references so every routine body below is byte-for-byte the
 *  same code as your Mega sketch.  Do not "tidy" these away.        */
Joint &head = JOINTS[0];
Joint &lh1  = JOINTS[1];   Joint &rh1  = JOINTS[9];
Joint &lh2  = JOINTS[2];   Joint &rh2  = JOINTS[10];
Joint &lh3  = JOINTS[3];   Joint &rh3  = JOINTS[11];
Joint &ll1  = JOINTS[4];   Joint &rl1  = JOINTS[12];
Joint &ll2  = JOINTS[5];   Joint &rl2  = JOINTS[13];
Joint &ll3  = JOINTS[6];   Joint &rl3  = JOINTS[14];
Joint &ll4  = JOINTS[7];   Joint &rl4  = JOINTS[15];
Joint &ll5  = JOINTS[8];   Joint &rl5  = JOINTS[16];

/*  Index constants for the joints the balance assist and the gait
 *  steer directly.  Order matches JOINTS[].                          */
#define J_LL1 4
#define J_LL2 5
#define J_LL3 6
#define J_LL4 7
#define J_LL5 8
#define J_RL1 12
#define J_RL2 13
#define J_RL3 14
#define J_RL4 15
#define J_RL5 16

/*====================================================================
 *  5. REFERENCE POSES
 *
 *  Logical angles in JOINTS[] order:
 *    head lh1 lh2 lh3 ll1 ll2 ll3 ll4 ll5 rh1 rh2 rh3 rl1 rl2 rl3 rl4 rl5
 *
 *  Not new numbers -- every one is lifted out of your own routines:
 *    STAND    = exactly what stand_straight() writes
 *    SIT      = exactly where sit_down() ends up
 *    HANDSUP  = the pose hands_down() assumes it starts from
 *    PLANK    = exactly where push_ups() leaves the robot
 *===================================================================*/
const int16_t POSE_STAND[NJ] = {
   90, 180, 180, 160,  90,  30, 150, 150,  90,   0,   0,  20,  90, 150,  30,  30,  90 };

const int16_t POSE_SIT[NJ] = {
   90, 150, 150, 130,  90,  90,  30,  90,  90,  30,  30,  50,  90,  90, 150,  90,  90 };

const int16_t POSE_HANDSUP[NJ] = {
   90,   0, 180, 160,  90,  30, 150, 150,  90, 180,   0,  20,  90, 150,  30,  30,  90 };

const int16_t POSE_PLANK[NJ] = {
   90,  90, 120,  70,  90,  30, 150, 150,  90,  90,  60, 110,  90, 150,  30,  30,  90 };

#define POSE_TOL 6      // degrees of slop when deciding "am I standing?"

int poseDistance(const int16_t* p) {
  int worst = 0;
  for (int i = 0; i < NJ; i++) {
    int d = abs((int)JOINTS[i].last - (int)p[i]);
    if (d > worst) worst = d;
  }
  return worst;
}

const char* whereAmI() {
  if (poseDistance(POSE_STAND)   <= POSE_TOL) return "standing";
  if (poseDistance(POSE_SIT)     <= POSE_TOL) return "sitting";
  if (poseDistance(POSE_PLANK)   <= POSE_TOL) return "push-up position";
  if (poseDistance(POSE_HANDSUP) <= POSE_TOL) return "standing, hands up";
  return "in between poses";
}

/*====================================================================
 *  6. GLIDE  --  move every joint to a pose together, smoothly
 *
 *  Cosine easing, so it starts slow, speeds up, ends slow. Duration
 *  scales with the furthest-travelling joint. Abortable.
 *
 *  This used to be the worst offender for bus noise: it wrote all 17
 *  joints every 20 ms even when 15 of them were not moving. It still
 *  loops over all 17 -- but send() now drops the ones whose pulse has
 *  not changed, so a one-joint correction really does move one servo.
 *===================================================================*/
/*  Set while glideTo() is moving every joint at once. The head
 *  gesture engine checks it and keeps its hands off.                 */
extern bool gGliding;

bool glideTo(const int16_t* target, int16_t ms) {
  int16_t from[NJ];
  int worst = 0;
  for (int i = 0; i < NJ; i++) {
    from[i] = JOINTS[i].last;
    int d = abs((int)target[i] - (int)from[i]);
    if (d > worst) worst = d;
  }
  if (worst == 0) return true;

  uint32_t dur = (ms > 0) ? (uint32_t)ms : (uint32_t)worst * GLIDE_MS_PER_DEG;
  if (dur < GLIDE_MIN_MS) dur = GLIDE_MIN_MS;
  if (dur > GLIDE_MAX_MS) dur = GLIDE_MAX_MS;

  int steps = (int)(dur / GLIDE_STEP_MS);
  if (steps < 1) steps = 1;

  bool wasLim = gRateLim;
  gRateLim = false;            // the glide is already fine-grained
  gGliding = true;             // hold off the head gesture engine
  for (int s = 1; s <= steps; s++) {
    float u = (float)s / (float)steps;
    float e = 0.5f * (1.0f - cosf(PI * u));
    for (int i = 0; i < NJ; i++) {
      float v = from[i] + ((float)target[i] - (float)from[i]) * e;
      JOINTS[i].writeNow((int)lroundf(v));
    }
    if (!naptime(GLIDE_STEP_MS)) { gRateLim = wasLim; gGliding = false; return false; }
  }
  for (int i = 0; i < NJ; i++) JOINTS[i].writeNow(target[i]);
  gRateLim = wasLim;
  gGliding = false;
  return true;
}

/*====================================================================
 *  7b. HEAD GESTURES
 *
 *  Your head servo is the 17th joint and the one wired directly to the
 *  ESP32 (GPIO 25) rather than to the PCA9685, because the PCA9685 has
 *  exactly 16 outputs. See section 2 for the wiring.
 *
 *  A gesture is a list of head angles and how long to take reaching
 *  each one. The engine walks the list a few degrees at a time from
 *  pumpIO(), so the head keeps moving while the arms and legs run
 *  their own routine. Nothing here ever blocks.
 *
 *  One servo means the head turns on one axis. These gestures are
 *  written as turns; if your neck bracket makes that servo tilt
 *  instead of pan, they still read correctly, just in the other axis.
 *
 *  If a routine writes the head itself, the gesture notices and gets
 *  out of the way rather than fighting it -- see headGestTick().
 *===================================================================*/
struct HKey { int8_t d; uint16_t ms; };     // d = degrees from centre

const HKey HK_NOD[]    = { {  -14, 150 }, {  0, 150 }, { -10, 130 }, { 0, 170 } };
const HKey HK_SHAKE[]  = { {  -26, 180 }, { 26, 260 }, { -20, 240 }, { 0, 200 } };
const HKey HK_TILT[]   = { {   18, 320 }, { 18, 500 }, {   0, 320 } };
const HKey HK_LOOK_L[] = { {  -32, 300 }, {-32, 600 }, {   0, 320 } };
const HKey HK_LOOK_R[] = { {   32, 300 }, { 32, 600 }, {   0, 320 } };
const HKey HK_SCAN[]   = { {  -36, 700 }, { 36, 1100 }, {  0, 700 } };
const HKey HK_BOW_H[]  = { {  -20, 420 }, {-20, 380 }, {   0, 420 } };
const HKey HK_PERK[]   = { {    0, 110 }, { -8, 120 }, {   0, 140 } };
const HKey HK_DROOP[]  = { {   24, 700 }, { 24, 900 } };
const HKey HK_BEAT[]   = { {  -16, 210 }, { 16, 210 } };          // loops
const HKey HK_SHIVER[] = { {   -7,  70 }, {  7,  70 } };          // loops

struct HGest { const HKey* k; uint8_t n; bool loop; };
#define HG(x, l) { x, (uint8_t)(sizeof(x) / sizeof(HKey)), l }
const HGest HGESTS[HG_COUNT] = {
  { NULL, 0, false },          // HG_NONE
  HG(HK_NOD,    false), HG(HK_SHAKE,  false), HG(HK_TILT,   false),
  HG(HK_LOOK_L, false), HG(HK_LOOK_R, false), HG(HK_SCAN,   false),
  HG(HK_BOW_H,  false), HG(HK_PERK,   false), HG(HK_DROOP,  false),
  HG(HK_BEAT,   true ), HG(HK_SHIVER, true ),
};
#undef HG

HeadGest gHG      = HG_NONE;
uint8_t  gHGi     = 0;         // which key we are travelling towards
uint32_t gHGt0    = 0;         // when this key started
uint32_t gHGLast  = 0;         // last tick
int16_t  gHGFrom  = HEAD_CENTRE;
int16_t  gHGWrote = HEAD_CENTRE;
uint32_t gHGCount = 0;         // gestures started, for the tests
bool     gGliding = false;     // glideTo() owns every joint while set

void headGesture(HeadGest g) {
  if (g == HG_NONE) { gHG = HG_NONE; return; }
  if (g >= HG_COUNT || HGESTS[g].n == 0) return;
  gHG      = g;
  gHGi     = 0;
  gHGt0    = millis();
  gHGFrom  = JOINTS[0].last;
  gHGWrote = gHGFrom;
  gHGCount++;
}
void headGestStop() { gHG = HG_NONE; }

/*  Called from pumpIO(). Costs one servo write every HEAD_TICK_MS and
 *  nothing at all when no gesture is running.                        */
void headGestTick() {
  if (gHG == HG_NONE || gGliding) return;

  uint32_t now = millis();
  if (now - gHGLast < HEAD_TICK_MS) return;
  gHGLast = now;

  /*  Did something else move the head? Then a routine wants it, and a
   *  routine outranks a decoration. Stand down.                      */
  if (JOINTS[0].last != gHGWrote) { gHG = HG_NONE; return; }

  const HGest& g = HGESTS[gHG];
  const HKey&  k = g.k[gHGi];
  uint32_t     el = now - gHGt0;
  int          tgt = HEAD_CENTRE + k.d;

  int a;
  if (k.ms == 0 || el >= k.ms) {
    a = tgt;
    gHGi++;
    if (gHGi >= g.n) {
      if (g.loop) { gHGi = 0; }
      else        { gHG = HG_NONE; }
    }
    gHGt0   = now;
    gHGFrom = tgt;
  } else {
    a = gHGFrom + (int)((long)(tgt - gHGFrom) * (long)el / (long)k.ms);
  }

  a = constrain(a, HEAD_MIN, HEAD_MAX);
  int d = a - gHGWrote;
  if (d >  HEAD_MAX_STEP) a = gHGWrote + HEAD_MAX_STEP;
  if (d < -HEAD_MAX_STEP) a = gHGWrote - HEAD_MAX_STEP;

  JOINTS[0].writeNow(a);
  gHGWrote = JOINTS[0].last;
}

/*====================================================================
 *  7c. EXPRESSIONS
 *
 *  One row per mood. Everything the robot shows and says comes from
 *  this table, so a face, its sound and its head movement can never
 *  drift apart -- there is only one place to change.
 *
 *  These are what the dashboard's emotion buttons send, as  feel <name>
 *===================================================================*/
struct Emote {
  const char* name;
  Face        face;
  Snd         sound;
  HeadGest    head;
};

const Emote EMOTES[] = {
  { "happy",      F_HAPPY,      S_HAPPY,    HG_NOD    },
  { "smile",      F_HAPPY,      S_GIGGLE,   HG_TILT   },
  { "angry",      F_ANGRY,      S_ANGRY,    HG_SHIVER },
  { "sad",        F_SAD,        S_SAD,      HG_DROOP  },
  { "surprised",  F_SURPRISE,   S_SURPRISE, HG_PERK   },
  { "love",       F_LOVE,       S_LOVE,     HG_TILT   },
  { "sleepy",     F_SLEEP,      S_SLEEPY,   HG_DROOP  },
  { "wink",       F_WINK,       S_WINK,     HG_TILT   },
  { "dizzy",      F_DIZZY,      S_DIZZY,    HG_SHAKE  },
  { "cool",       F_COOL,       S_COOL,     HG_LOOK_R },
  { "proud",      F_PROUD,      S_FLEX,     HG_PERK   },
  { "curious",    F_CURIOUS,    S_CURIOUS,  HG_SCAN   },
  { "determined", F_DETERMINED, S_GRUNT,    HG_NOD    },
  { "bye",        F_BYE,        S_BYE,      HG_SHAKE  },
  { "music",      F_MUSIC,      S_MUSIC,    HG_BEAT   },
  { "confused",   F_CURIOUS,    S_ERROR,    HG_SHAKE  },
  { "alert",      F_ALERT,      S_ERROR,    HG_SHIVER },
  { "neutral",    F_IDLE,       S_BEEP,     HG_PERK   },
};
#define NEMOTE ((int)(sizeof(EMOTES) / sizeof(EMOTES[0])))

/*  Fire a face, a sound and a head movement as one thing. This is the
 *  only function that should ever set them, so they stay in step.    */
void express(Face f, Snd s, HeadGest h, const char* msg) {
  faceSet(f, msg);
  if (s != S_NONE) soundPlay(s);
  if (h != HG_NONE) headGesture(h);
}

bool expressNamed(const String& name) {
  for (int i = 0; i < NEMOTE; i++) {
    if (name == EMOTES[i].name) {
      const Emote& m = EMOTES[i];
      express(m.face, m.sound, m.head, m.name);
      return true;
    }
  }
  return false;
}

/*  Names, so you can type  sound karate  or  head nod  and hear or see
 *  one piece on its own while you are tuning things.                  */
const char* const SND_NAMES[S_COUNT] = {
  "none", "boot", "ready", "beep", "happy", "giggle", "hi", "bye",
  "angry", "sad", "surprise", "love", "sleepy", "wink", "dizzy", "cool",
  "grunt", "walk", "sit", "stand", "karate", "bow", "flex", "music",
  "win", "curious", "error"
};
const char* const HG_NAMES[HG_COUNT] = {
  "none", "nod", "shake", "tilt", "look left", "look right", "scan",
  "bow", "perk", "droop", "beat", "shiver"
};

bool soundNamed(const String& n) {
  for (int i = 1; i < S_COUNT; i++)
    if (n == SND_NAMES[i]) { soundPlay((Snd)i); return true; }
  return false;
}
bool headNamed(const String& n) {
  for (int i = 1; i < HG_COUNT; i++)
    if (n == HG_NAMES[i]) { headGesture((HeadGest)i); return true; }
  return false;
}


/*  Spread or close the hips by n degrees, symmetrically. Used only if
 *  SIT_SPREAD_DEG is non-zero.                                      */
bool hipSpread(int deg) {
  if (deg == 0) return true;
  int16_t t[NJ];
  for (int i = 0; i < NJ; i++) t[i] = JOINTS[i].last;
  t[J_LL1] = constrain(JOINTS[J_LL1].last + deg, 0, 180);
  t[J_RL1] = constrain(JOINTS[J_RL1].last - deg, 0, 180);
  return glideTo(t, 900);
}

/*====================================================================
 *  7. STAGGERED POWER-UP
 *
 *  Seventeen MG996Rs told to hold a position in the same millisecond
 *  is a 15-30 A inrush. Even a strong supply dips; a 2 A supply falls
 *  over completely, which is why the robot used to jerk and buzz the
 *  instant it was switched on. One servo at a time, 120 ms apart,
 *  spreads that out to something a supply can actually deliver.
 *===================================================================*/
void energiseStaggered(const int16_t* pose) {
  Serial.println(F("energising servos one at a time..."));
  for (int i = 0; i < NJ; i++) {
    JOINTS[i].last = pose[i];
    JOINTS[i].refresh();
    Serial.printf("  %-4s ch%-3d angle %3d -> pulse %d%s\n",
                  JOINTS[i].name, JOINTS[i].ch, pose[i],
                  JOINTS[i].lastSent, JOINTS[i].inv ? "   (inverted)" : "");
    delay(BOOT_STAGGER_MS);
    pumpIO();
  }
  Serial.println(F("all servos live."));
}

/*====================================================================
 *  8. OLED  --  128x64 SSD1306 on its own I2C bus, no library needed
 *
 *  Deliberately hand-rolled rather than pulling in Adafruit_SSD1306:
 *  nothing to install, and more importantly this version pushes at
 *  most ONE 128-byte strip per pass. A full-screen update is 1024
 *  bytes; done in one blocking lump it would stall the sketch for
 *  25 ms and you would feel it as a hitch in the servos. Only the
 *  strips that actually changed get sent, one per pass.
 *===================================================================*/
#if USE_OLED
/*  5x7 font, inlined so this sketch is ONE file -- nothing extra to
 *  create in the Arduino IDE.                                       */
/*  5x7 font, codes 32..90 (' '..'Z'), 5 column bytes each,
 *  bit 0 = top row. Lower case is folded to upper case when drawn.
 *  Generated table -- glyphs authored as bitmaps, not traced from a
 *  TTF, because a real typeface rendered down to 7 pixels is mush. */
#define FONT_FIRST 32
#define FONT_LAST  90
const uint8_t FONT5x7[] PROGMEM = {
  0x00,0x00,0x00,0x00,0x00,   //  
  0x00,0x00,0x5F,0x00,0x00,   // !
  0x00,0x07,0x00,0x07,0x00,   // "
  0x14,0x7F,0x14,0x7F,0x14,   // #
  0x24,0x2A,0x7F,0x2A,0x12,   // $
  0x63,0x13,0x08,0x64,0x63,   // %
  0x36,0x49,0x55,0x22,0x50,   // &
  0x00,0x05,0x03,0x00,0x00,   // '
  0x00,0x1C,0x22,0x41,0x00,   // (
  0x00,0x41,0x22,0x1C,0x00,   // )
  0x14,0x08,0x3E,0x08,0x14,   // *
  0x08,0x08,0x3E,0x08,0x08,   // +
  0x00,0x50,0x30,0x00,0x00,   // ,
  0x08,0x08,0x08,0x08,0x08,   // -
  0x00,0x60,0x60,0x00,0x00,   // .
  0x40,0x30,0x08,0x06,0x01,   // /
  0x3E,0x51,0x49,0x45,0x3E,   // 0
  0x00,0x42,0x7F,0x40,0x00,   // 1
  0x42,0x61,0x51,0x49,0x46,   // 2
  0x21,0x41,0x45,0x47,0x39,   // 3
  0x18,0x14,0x12,0x7F,0x10,   // 4
  0x27,0x45,0x45,0x45,0x39,   // 5
  0x3C,0x4A,0x49,0x49,0x30,   // 6
  0x01,0x01,0x79,0x05,0x03,   // 7
  0x36,0x49,0x49,0x49,0x36,   // 8
  0x06,0x49,0x49,0x29,0x1E,   // 9
  0x00,0x36,0x36,0x00,0x00,   // :
  0x00,0x56,0x36,0x00,0x00,   // ;
  0x08,0x14,0x22,0x41,0x00,   // <
  0x14,0x14,0x14,0x14,0x14,   // =
  0x00,0x41,0x22,0x14,0x08,   // >
  0x02,0x01,0x51,0x09,0x06,   // ?
  0x32,0x49,0x79,0x41,0x3E,   // @
  0x7C,0x12,0x11,0x12,0x7C,   // A
  0x7F,0x49,0x49,0x49,0x36,   // B
  0x3E,0x41,0x41,0x41,0x22,   // C
  0x7F,0x41,0x41,0x22,0x1C,   // D
  0x7F,0x49,0x49,0x49,0x41,   // E
  0x7F,0x09,0x09,0x09,0x01,   // F
  0x3E,0x41,0x41,0x49,0x3A,   // G
  0x7F,0x08,0x08,0x08,0x7F,   // H
  0x00,0x41,0x7F,0x41,0x00,   // I
  0x30,0x40,0x40,0x40,0x3F,   // J
  0x7F,0x08,0x14,0x22,0x41,   // K
  0x7F,0x40,0x40,0x40,0x40,   // L
  0x7F,0x02,0x0C,0x02,0x7F,   // M
  0x7F,0x02,0x04,0x08,0x7F,   // N
  0x3E,0x41,0x41,0x41,0x3E,   // O
  0x7F,0x09,0x09,0x09,0x06,   // P
  0x3E,0x41,0x51,0x21,0x5E,   // Q
  0x7F,0x09,0x19,0x29,0x46,   // R
  0x46,0x49,0x49,0x49,0x31,   // S
  0x01,0x01,0x7F,0x01,0x01,   // T
  0x3F,0x40,0x40,0x40,0x3F,   // U
  0x1F,0x20,0x40,0x20,0x1F,   // V
  0x7F,0x20,0x18,0x20,0x7F,   // W
  0x63,0x14,0x08,0x14,0x63,   // X
  0x03,0x04,0x78,0x04,0x03,   // Y
  0x61,0x51,0x49,0x45,0x43,   // Z
};

uint8_t  oledBuf[1024];        // 128 x 64 / 8
bool     oledDirty[8];
bool     gOledOk    = false;
uint8_t  gOledPage  = 0;
uint32_t gOledNext  = 0;

void oledCmd(uint8_t c) {
  OledBus.beginTransmission(OLED_ADDR);
  OledBus.write((uint8_t)0x00);
  OledBus.write(c);
  OledBus.endTransmission();
}

/*  Blank all 132 RAM columns of every page. The two columns that sit
 *  off the left of the glass are what show up as that thin bright line,
 *  and nothing else in the sketch ever writes to them, so they have to
 *  be cleared once, here.                                             */
void oledWipeRam() {
  for (uint8_t pg = 0; pg < 8; pg++) {
    oledCmd(0xB0 | pg);
    oledCmd(0x00);
    oledCmd(0x10);
    for (uint8_t blk = 0; blk < 4; blk++) {
      OledBus.beginTransmission(OLED_ADDR);
      OledBus.write((uint8_t)0x40);
      for (uint8_t k = 0; k < 33; k++) OledBus.write((uint8_t)0x00);
      OledBus.endTransmission();
    }
  }
}

bool oledInit() {
  OledBus.begin(OLED_SDA, OLED_SCL, OLED_I2C_HZ);
  OledBus.beginTransmission(OLED_ADDR);
  if (OledBus.endTransmission() != 0) return false;   // nothing there

  static const uint8_t seq[] = {
    0xAE,               // display off
    0xD5, 0x80,         // clock divide
    0xA8, 0x3F,         // multiplex = 64 rows
    0xD3, 0x00,         // display offset
    0x40,               // start line 0
    0x8D, 0x14,         // charge pump on
    0x20, 0x02,         // PAGE addressing -- part of the stripe fix
    0xA1,               // segment remap
    0xC8,               // COM scan direction: flipped
    0xDA, 0x12,         // COM pins
    0x81, 0xCF,         // contrast
    0xD9, 0xF1,         // pre-charge
    0xDB, 0x40,         // VCOM detect
    0xA4,               // resume from RAM
    0xA6,               // not inverted
    0x2E,               // scrolling off
    0xAF                // display on
  };
  for (uint8_t i = 0; i < sizeof(seq); i++) oledCmd(seq[i]);
  oledWipeRam();                 // kills the stripe on the left edge
  memset(oledBuf, 0, sizeof(oledBuf));
  for (int i = 0; i < 8; i++) oledDirty[i] = true;
  return true;
}

/*  Push exactly one page (a 128 x 8 pixel horizontal strip). 128 data
 *  bytes at 400 kHz is about 3 ms, split into 32-byte chunks because
 *  the ESP32 Wire buffer is 128 bytes including the control byte.    */
void oledPushPage(uint8_t p) {
  oledCmd(0xB0 | p);          // page address
  oledCmd(0x00 | (OLED_COL_OFFSET & 0x0F));        // column low nibble
  oledCmd(0x10 | ((OLED_COL_OFFSET >> 4) & 0x0F)); // column high nibble
  const uint8_t* src = oledBuf + (uint16_t)p * 128;
  for (uint8_t off = 0; off < 128; off += 32) {
    OledBus.beginTransmission(OLED_ADDR);
    OledBus.write((uint8_t)0x40);
    OledBus.write(src + off, 32);
    OledBus.endTransmission();
  }
  oledDirty[p] = false;
}

/*  Called from pumpIO(). One page per visit, so the worst case cost of
 *  the display anywhere in this sketch is ~3 ms.                     */
void oledService() {
  if (!gOledOk) return;
  for (int n = 0; n < 8; n++) {
    uint8_t p = (gOledPage + n) & 7;
    if (oledDirty[p]) { oledPushPage(p); gOledPage = (p + 1) & 7; return; }
  }
}

/* ---- drawing primitives (buffer only, no I2C) ------------------- */
static inline void oledPx(int x, int y, bool on) {
  if ((unsigned)x >= 128 || (unsigned)y >= 64) return;
  uint16_t i = (uint16_t)(y >> 3) * 128 + x;
  uint8_t  m = 1 << (y & 7);
  uint8_t  v = on ? (oledBuf[i] | m) : (oledBuf[i] & ~m);
  if (v != oledBuf[i]) { oledBuf[i] = v; oledDirty[y >> 3] = true; }
}
void oledClear() {
  for (uint16_t i = 0; i < sizeof(oledBuf); i++) {
    if (oledBuf[i]) { oledBuf[i] = 0; oledDirty[i >> 7] = true; }
  }
}
void oledFillRect(int x, int y, int w, int h, bool on) {
  for (int j = y; j < y + h; j++)
    for (int i = x; i < x + w; i++) oledPx(i, j, on);
}
void oledFillCircle(int cx, int cy, int r, bool on) {
  for (int j = -r; j <= r; j++)
    for (int i = -r; i <= r; i++)
      if (i * i + j * j <= r * r) oledPx(cx + i, cy + j, on);
}
/*  A filled arc used for the mouth: the lower half of a ring.
 *  dir  +1 = smile (curve opens upward), -1 = frown.                 */
void oledArc(int cx, int cy, int r, int thick, int dir) {
  for (int j = -r; j <= r; j++) {
    for (int i = -r; i <= r; i++) {
      int d2 = i * i + j * j;
      if (d2 > r * r || d2 < (r - thick) * (r - thick)) continue;
      if (dir > 0 ? (j > 0) : (j < 0)) oledPx(cx + i, cy + j, true);
    }
  }
}
void oledChar(int x, int y, char c, int scale) {
  if (c >= 'a' && c <= 'z') c -= 32;            // fold to upper case
  if (c < FONT_FIRST || c > FONT_LAST) c = ' ';
  const uint8_t* g = FONT5x7 + (c - FONT_FIRST) * 5;
  for (int col = 0; col < 5; col++) {
    uint8_t bits = pgm_read_byte(g + col);
    for (int row = 0; row < 7; row++) {
      if (!((bits >> row) & 1)) continue;
      if (scale == 1) oledPx(x + col, y + row, true);
      else oledFillRect(x + col * scale, y + row * scale, scale, scale, true);
    }
  }
}
void oledText(int x, int y, const char* s, int scale) {
  while (*s) { oledChar(x, y, *s++, scale); x += 6 * scale; }
}
void oledTextCentre(int y, const char* s, int scale) {
  int w = 0; for (const char* p = s; *p; p++) w += 6 * scale;
  oledText((128 - w) / 2, y, s, scale);
}

/* ---- extra primitives, for the faces ---------------------------- */
/*  A thick line. oledLineTC can also erase, which is how you draw a
 *  highlight ON something already lit -- see the sunglasses.          */
void oledLineTC(int x0, int y0, int x1, int y1, int thick, bool on) {
  int dx = abs(x1 - x0), dy = abs(y1 - y0);
  int n = (dx > dy ? dx : dy); if (n < 1) n = 1;
  for (int i = 0; i <= n; i++) {
    int x = x0 + (x1 - x0) * i / n, y = y0 + (y1 - y0) * i / n;
    oledFillRect(x - thick / 2, y - thick / 2, thick, thick, on);
  }
}
void oledLineT(int x0, int y0, int x1, int y1, int thick) {
  oledLineTC(x0, y0, x1, y1, thick, true);
}
void oledRing(int cx, int cy, int r, int thick) {
  int ri = r - thick; if (ri < 0) ri = 0;
  for (int j = -r; j <= r; j++)
    for (int i = -r; i <= r; i++) {
      int d2 = i * i + j * j;
      if (d2 <= r * r && d2 >= ri * ri) oledPx(cx + i, cy + j, true);
    }
}
void oledRoundRect(int x, int y, int w, int h, int r) {
  oledFillRect(x + r, y, w - 2 * r, h, true);
  oledFillRect(x, y + r, w, h - 2 * r, true);
  oledFillCircle(x + r, y + r, r, true);
  oledFillCircle(x + w - r - 1, y + r, r, true);
  oledFillCircle(x + r, y + h - r - 1, r, true);
  oledFillCircle(x + w - r - 1, y + h - r - 1, r, true);
}
/*  A heart, for the in-love face. Two lobes and a triangle.          */
void oledHeart(int cx, int cy, int s) {
  int r = s / 2; if (r < 1) r = 1;
  oledFillCircle(cx - r / 2 - 1, cy - r / 2, r, true);
  oledFillCircle(cx + r / 2 + 1, cy - r / 2, r, true);
  for (int j = 0; j <= s; j++) {
    int half = (s - j) * (s + 2) / (2 * s);
    for (int i = -half; i <= half; i++) oledPx(cx + i, cy + j - r / 2, true);
  }
}
/*  A rough spiral, for dizzy eyes. ph rotates it.                    */
void oledSpiral(int cx, int cy, int r, float ph) {
  for (float a = 0; a < 6.28f * 2.2f; a += 0.22f) {
    float rr = r * a / (6.28f * 2.2f);
    int x = cx + (int)(rr * cosf(a + ph)), y = cy + (int)(rr * sinf(a + ph));
    oledFillRect(x - 1, y - 1, 2, 2, true);
  }
}
/*  A quaver, for the dancing face.                                   */
void oledNoteGlyph(int x, int y) {
  oledFillCircle(x, y + 8, 3, true);
  oledFillRect(x + 2, y - 1, 2, 10, true);
  oledFillRect(x + 4, y - 1, 3, 2, true);
}
/*  A teardrop / sweat bead.                                          */
void oledDrop(int x, int y, int s) {
  oledFillCircle(x, y + s, s, true);
  for (int j = 0; j < s * 2; j++) {
    int half = j * s / (s * 2); if (half < 0) half = 0;
    for (int i = -half; i <= half; i++) oledPx(x + i, y + j - s, true);
  }
}

/*====================================================================
 *  9. FACES
 *
 *  Twenty expressions, each with its own animation, drawn straight
 *  into the frame buffer.
 *
 *  This all runs in a task pinned to core 0 (see faceTask below), for
 *  the same reason the sound does: pushing 1 KB over I2C takes about
 *  25 ms, and doing that on core 1 in the middle of a walk would show
 *  up as a stutter in the legs. On its own core it is invisible.
 *===================================================================*/
/*  enum Face is defined near the top -- see the note there. */
Face     gFace     = F_BOOT;
Face     gFaceWant = F_BOOT;
char     gFaceMsg[22] = "";
uint32_t gBlinkAt  = 0;
bool     gBlinking = false;
int8_t   gEyeShift = 0;
uint32_t gAnim     = 0;      // frame counter, drives every animation
uint32_t gFaceCount= 0;      // expressions set, for the tests
bool     gFaceTaskOn = false;

void faceSet(Face f, const char* msg) {
  if (f != gFaceWant) gFaceCount++;
  gFaceWant = f;
  strncpy(gFaceMsg, msg ? msg : "", sizeof(gFaceMsg) - 1);
  gFaceMsg[sizeof(gFaceMsg) - 1] = 0;
}

/* ---- eyes -------------------------------------------------------- */
#define EYE_ROUND   0
#define EYE_BIG     1
#define EYE_CLOSED  2
#define EYE_HAPPY   3    // upward arc, a smiling eye
#define EYE_NARROW  4
#define EYE_X       5
#define EYE_HEART   6
#define EYE_SPIRAL  7
#define EYE_RING    8
#define EYE_DROOP   9

void drawEye(int cx, int cy, int style, int shift, float ph) {
  cx += shift;
  switch (style) {
    case EYE_CLOSED:
      oledFillRect(cx - 11, cy - 1, 22, 4, true);
      break;
    case EYE_HAPPY:                       // ^ ^  a happy squint
      oledArc(cx, cy + 5, 11, 4, -1);
      break;
    case EYE_NARROW:
      oledFillRect(cx - 11, cy - 4, 22, 8, true);
      oledFillCircle(cx + 4, cy - 3, 2, false);
      break;
    case EYE_DROOP:
      oledFillCircle(cx, cy, 10, true);
      oledArc(cx, cy - 6, 13, 8, -1);     // heavy upper lid
      oledFillCircle(cx + 4, cy + 2, 3, false);
      break;
    case EYE_X:
      for (int k = -9; k <= 9; k++) {
        oledFillRect(cx + k - 1, cy + k - 1, 3, 3, true);
        oledFillRect(cx + k - 1, cy - k - 1, 3, 3, true);
      }
      break;
    case EYE_HEART:
      oledHeart(cx, cy - 2, 11);
      break;
    case EYE_SPIRAL:
      oledSpiral(cx, cy, 12, ph);
      break;
    case EYE_RING:                        // wide open, startled
      oledRing(cx, cy, 13, 4);
      oledFillCircle(cx, cy, 4, true);
      break;
    case EYE_BIG:
      oledFillCircle(cx, cy, 13, true);
      oledFillCircle(cx + 5, cy - 5, 4, false);
      break;
    default:
      oledFillCircle(cx, cy, 10, true);
      oledFillCircle(cx + 4, cy - 4, 3, false);
      break;
  }
}

/* ---- mouths ------------------------------------------------------ */
#define M_SMILE_BIG 0
#define M_SMILE     1
#define M_FLAT      2
#define M_FROWN     3
#define M_OPEN_O    4
#define M_GRIT      5
#define M_SMIRK     6
#define M_WAVY      7
#define M_TINY      8
#define M_GRIN      9

void drawMouth(int style, int y, float ph) {
  switch (style) {
    case M_SMILE_BIG: oledArc(64, y - 8, 18, 5, 1);                     break;
    case M_SMILE:     oledArc(64, y - 6, 13, 4, 1);                     break;
    case M_FLAT:      oledFillRect(52, y, 24, 4, true);                  break;
    case M_FROWN:     oledArc(64, y + 9, 15, 4, -1);                     break;
    case M_OPEN_O:    oledRing(64, y, 8 + (int)(2 * sinf(ph)), 3);       break;
    case M_GRIT:
      oledFillRect(50, y - 4, 28, 9, true);
      for (int i = 0; i < 5; i++) oledFillRect(53 + i * 6, y - 4, 2, 9, false);
      break;
    case M_SMIRK:     oledLineT(54, y + 3, 74, y - 3, 4);                break;
    case M_WAVY:
      for (int i = -14; i <= 14; i++)
        oledFillRect(64 + i - 1, y + (int)(3 * sinf(i / 3.0f + ph)) - 1, 3, 3, true);
      break;
    case M_TINY:      oledFillRect(59, y, 10, 4, true);                  break;
    case M_GRIN:
      oledArc(64, y - 8, 18, 10, 1);
      oledFillRect(48, y - 2, 32, 2, false);
      break;
  }
}

/* ---- the whole face --------------------------------------------- */
void faceDraw() {
  oledClear();
  const int eyeY = 24, lx = 40, rx = 88;
  float ph = gAnim * 0.4f;                    // general animation phase
  int   sh = gEyeShift;

  switch (gFace) {

    case F_BOOT: {
      oledTextCentre(10, "HUMANOID", 2);
      /*  a little progress bar that fills and wraps */
      int w = 8 + (int)(gAnim * 6) % 96;
      oledRoundRect(14, 36, 100, 12, 5);
      oledFillRect(16, 38, 96, 8, false);
      oledFillRect(16, 38, w > 96 ? 96 : w, 8, true);
      oledTextCentre(54, "WAKING UP", 1);
      return;
    }

    case F_SLEEP: {
      drawEye(lx, eyeY, EYE_CLOSED, 0, ph);
      drawEye(rx, eyeY, EYE_CLOSED, 0, ph);
      drawMouth(M_TINY, 44, ph);
      /*  three Z's drifting up and fading out by leaving the screen.
       *  gAnim/2, not gAnim/3: a third of a second between moves reads
       *  as a stall rather than as breathing.                         */
      for (int k = 0; k < 3; k++) {
        int step = (gAnim / 2 + k) % 9;
        int x = 96 + k * 6 - step, y = 30 - step * 3 - k * 5;
        if (y > 0 && y < 56) oledChar(x, y, 'Z', k == 0 ? 2 : 1);
      }
      break;
    }

    case F_ALERT: {
      int jit = (gAnim & 1) ? 2 : -2;         // the whole face shakes
      drawEye(lx + jit, eyeY, EYE_X, 0, ph);
      drawEye(rx + jit, eyeY, EYE_X, 0, ph);
      oledFillRect(48 + jit, 46, 32, 4, true);
      break;
    }

    case F_SIT: {
      drawEye(lx, eyeY, EYE_NARROW, 0, ph);
      drawEye(rx, eyeY, EYE_NARROW, 0, ph);
      drawMouth(M_SMILE, 44, ph);
      break;
    }

    case F_ANGRY: {
      int jit = (gAnim & 1) ? 2 : -2;         // shivering with rage
      drawEye(lx + jit, eyeY + 2, EYE_NARROW, 0, ph);
      drawEye(rx + jit, eyeY + 2, EYE_NARROW, 0, ph);
      /*  brows angled down towards the nose */
      oledLineT(lx - 13 + jit, eyeY - 13, lx + 12 + jit, eyeY - 5, 5);
      oledLineT(rx + 13 + jit, eyeY - 13, rx - 12 + jit, eyeY - 5, 5);
      drawMouth(M_FROWN, 46, ph);
      /*  steam puffs from the top corners */
      for (int k = 0; k < 2; k++) {
        int s = (gAnim / 2 + k * 2) % 6;
        oledRing(10 + k * 108, 12 - s, 2 + s / 2, 1);
      }
      break;
    }

    case F_SAD: {
      drawEye(lx, eyeY, EYE_DROOP, 0, ph);
      drawEye(rx, eyeY, EYE_DROOP, 0, ph);
      oledLineT(lx - 13, eyeY - 14, lx + 11, eyeY - 18, 3);
      oledLineT(rx + 13, eyeY - 14, rx - 11, eyeY - 18, 3);
      drawMouth(M_FROWN, 46, ph);
      /*  a tear that falls, over and over */
      int t = (gAnim / 2) % 10;
      oledDrop(lx - 12, eyeY + 8 + t * 3, 3);
      break;
    }

    case F_SURPRISE: {
      int pulse = (gAnim & 2) ? 1 : 0;
      drawEye(lx, eyeY - pulse, EYE_RING, 0, ph);
      drawEye(rx, eyeY - pulse, EYE_RING, 0, ph);
      drawMouth(M_OPEN_O, 48, ph);
      /*  exclamation streaks */
      oledLineT(8, 8, 16, 18, 3);
      oledLineT(120, 8, 112, 18, 3);
      break;
    }

    case F_LOVE: {
      int pulse = (gAnim % 6 < 3) ? 1 : -1;   // hearts beat
      oledHeart(lx, eyeY - 2, 11 + pulse);
      oledHeart(rx, eyeY - 2, 11 + pulse);
      drawMouth(M_SMILE_BIG, 48, ph);
      /*  little hearts floating up the sides */
      for (int k = 0; k < 2; k++) {
        int s = (gAnim / 2 + k * 4) % 12;
        oledHeart(12 + k * 104, 52 - s * 4, 5);
      }
      break;
    }

    case F_WINK: {
      drawEye(lx, eyeY, EYE_CLOSED, 0, ph);
      drawEye(rx, eyeY, EYE_BIG, 0, ph);
      drawMouth(M_SMIRK, 46, ph);
      /*  a sparkle by the open eye, twinkling */
      if (gAnim & 1) {
        oledLineT(rx + 16, eyeY - 14, rx + 22, eyeY - 20, 2);
        oledLineT(rx + 16, eyeY - 20, rx + 22, eyeY - 14, 2);
      }
      break;
    }

    case F_DIZZY: {
      drawEye(lx, eyeY, EYE_SPIRAL, 0,  ph);
      drawEye(rx, eyeY, EYE_SPIRAL, 0, -ph);
      drawMouth(M_WAVY, 46, ph);
      break;
    }

    case F_COOL: {
      /*  sunglasses: one band across both eyes */
      oledRoundRect(24, eyeY - 11, 34, 20, 6);
      oledRoundRect(70, eyeY - 11, 34, 20, 6);
      oledFillRect(58, eyeY - 3, 12, 4, true);
      /*  A glint sliding across the lenses. It has to be drawn dark:
       *  the lenses are solid white, so a white streak on top of them
       *  would be invisible.                                          */
      int g = 26 + (int)(gAnim * 5) % 76;
      oledLineTC(g, eyeY - 9, g + 6, eyeY + 7, 3, false);
      oledLineTC(g + 5, eyeY - 9, g + 11, eyeY + 7, 2, false);
      drawMouth(M_SMIRK, 48, ph);
      break;
    }

    case F_DETERMINED: {
      drawEye(lx, eyeY + 1, EYE_NARROW, 0, ph);
      drawEye(rx, eyeY + 1, EYE_NARROW, 0, ph);
      oledLineT(lx - 13, eyeY - 12, lx + 12, eyeY - 9, 4);
      oledLineT(rx + 13, eyeY - 12, rx - 12, eyeY - 9, 4);
      drawMouth(M_GRIT, 47, ph);
      /*  effort bead on the temple, dripping */
      int t = (gAnim / 2) % 8;
      oledDrop(112, 14 + t * 3, 3);
      break;
    }

    case F_PROUD: {
      drawEye(lx, eyeY, EYE_HAPPY, 0, ph);
      drawEye(rx, eyeY, EYE_HAPPY, 0, ph);
      drawMouth(M_GRIN, 46, ph);
      /*  sparkles popping in and out around the head */
      const int sx[4] = { 12, 116, 22, 106 }, sy[4] = { 10, 12, 44, 42 };
      for (int k = 0; k < 4; k++) {
        if (((gAnim / 2) + k) % 4 == 0) continue;
        oledLineT(sx[k] - 4, sy[k], sx[k] + 4, sy[k], 2);
        oledLineT(sx[k], sy[k] - 4, sx[k], sy[k] + 4, 2);
      }
      break;
    }

    case F_CURIOUS: {
      drawEye(lx, eyeY + 2, EYE_ROUND, 0, ph);
      drawEye(rx, eyeY - 2, EYE_BIG,   0, ph);
      oledLineT(rx - 12, eyeY - 18, rx + 12, eyeY - 22, 3);
      drawMouth(M_TINY, 46, ph);
      /*  a question mark bobbing */
      int b = (gAnim % 4 < 2) ? 0 : 2;
      oledChar(108, 6 + b, '?', 2);
      break;
    }

    case F_BYE: {
      drawEye(lx, eyeY, EYE_HAPPY, 0, ph);
      drawEye(rx, eyeY, EYE_HAPPY, 0, ph);
      drawMouth(M_SMILE_BIG, 46, ph);
      /*  a hand waving in the corner */
      int t = (gAnim % 4 < 2) ? -4 : 4;
      oledRoundRect(102, 8, 14, 16, 5);
      oledLineT(109, 24, 109 + t, 34, 4);
      break;
    }

    case F_MUSIC: {
      int bob = (gAnim % 4 < 2) ? -2 : 2;     // head-bopping eyes
      drawEye(lx, eyeY + bob, EYE_HAPPY, 0, ph);
      drawEye(rx, eyeY - bob, EYE_HAPPY, 0, ph);
      drawMouth(M_SMILE_BIG, 46 + bob, ph);
      for (int k = 0; k < 2; k++) {
        int s = (gAnim / 2 + k * 5) % 10;
        oledNoteGlyph(8 + k * 108, 44 - s * 4);
      }
      break;
    }

    case F_WALK: {
      drawEye(lx, eyeY, EYE_ROUND, sh, ph);
      drawEye(rx, eyeY, EYE_ROUND, sh, ph);
      drawMouth(M_SMILE, 44, ph);
      /*  speed dashes, moving backwards past the face */
      for (int k = 0; k < 3; k++) {
        int x = (int)((gAnim * 4 + k * 12) % 40);
        oledFillRect(2 + x / 2, 16 + k * 12, 8, 2, true);
        oledFillRect(118 - x / 2, 16 + k * 12, 8, 2, true);
      }
      break;
    }

    case F_BUSY: {
      drawEye(lx, eyeY, EYE_NARROW, sh, ph);
      drawEye(rx, eyeY, EYE_NARROW, sh, ph);
      drawMouth(M_FLAT, 44, ph);
      /*  ... thinking dots */
      for (int k = 0; k < 3; k++)
        if ((gAnim / 2) % 4 > k) oledFillCircle(52 + k * 12, 56, 3, true);
      break;
    }

    case F_HAPPY: {
      if (gBlinking) {
        drawEye(lx, eyeY, EYE_CLOSED, sh, ph);
        drawEye(rx, eyeY, EYE_CLOSED, sh, ph);
      } else {
        drawEye(lx, eyeY, EYE_BIG, sh, ph);
        drawEye(rx, eyeY, EYE_BIG, sh, ph);
      }
      drawMouth(M_SMILE_BIG, 48, ph);
      /*  blushing cheeks, breathing in and out */
      int bl = (gAnim & 1) ? 1 : 0;
      oledFillRect(14 - bl, 40, 8 + bl * 2, 2, true);
      oledFillRect(12 - bl, 44, 12 + bl * 2, 2, true);
      oledFillRect(106 - bl, 40, 8 + bl * 2, 2, true);
      oledFillRect(104 - bl, 44, 12 + bl * 2, 2, true);
      /*  two little twinkles taking turns, so the face is never still */
      {
        int k = (gAnim / 2) & 1;
        int tx = k ? 116 : 12, ty = k ? 12 : 9;
        oledLineT(tx - 4, ty, tx + 4, ty, 2);
        oledLineT(tx, ty - 4, tx, ty + 4, 2);
      }
      break;
    }

    default: {                                  // F_IDLE
      if (gBlinking) {
        drawEye(lx, eyeY, EYE_CLOSED, sh, ph);
        drawEye(rx, eyeY, EYE_CLOSED, sh, ph);
      } else {
        drawEye(lx, eyeY, EYE_ROUND, sh, ph);
        drawEye(rx, eyeY, EYE_ROUND, sh, ph);
      }
      drawMouth(M_SMILE, 44, ph);
      break;
    }
  }
  if (gFaceMsg[0]) oledTextCentre(57, gFaceMsg, 1);
}

/*  Which faces animate on their own, and therefore need a redraw
 *  every frame even when nothing else changed.                       */
static bool faceAnimates(Face f) {
  switch (f) {
    case F_IDLE: case F_SIT: return false;      // still unless blinking
    default:                 return true;
  }
}

/*  One animation frame: work out the state, redraw if needed, push. */
/*====================================================================
 *  OLED USER MODES  --  phone drawing, text, scrolling banner
 *    gOledMode  0 = the normal animated faces (unchanged)
 *               1 = your text, 2 = your drawing, 3 = scrolling banner
 *  Drawings are 1024 bytes, exactly the frame buffer layout, so the
 *  browser packs them and the ESP32 only has to memcpy.
 *===================================================================*/
uint8_t  gOledMode = 0;
char     gOledText[64] = "";
uint8_t  gOledBmp[1024];
bool     gBmpHave  = false;
bool     gSlideOn  = false;
uint32_t gSlideAt  = 0;
uint8_t  gSlideIdx = 0;
/*  P8 / P9.  Where your words sit on the glass and how they move.
 *  gTxX / gTxY are the top-left corner in OLED pixels, so the phone can
 *  simply drag them. gTxSc is the character scale 1..4 (a character is
 *  6 x 8 pixels at scale 1). gTxOver draws the words ON TOP of the
 *  picture that is already in memory instead of clearing the screen,
 *  which is how text over a drawing works. The banner uses gScrDir
 *  (-1 = right to left, +1 = left to right) and gScrSpd in pixels per
 *  second, so the speed slider is honest rather than a guess.        */
int16_t  gTxX   = 6, gTxY = 24;
uint8_t  gTxSc  = 2;
bool     gTxOver = false;
int8_t   gScrDir = -1;
uint8_t  gScrSpd = 40;

static void drawKey(int slot, char* k) { snprintf(k, 8, "drw%d", slot & 7); }

bool drawHas(int slot) {
  char k[8]; drawKey(slot, k);
  return prefs.getBytesLength(k) == 1024;
}
bool drawSave(int slot) {
  char k[8]; drawKey(slot, k);
  return prefs.putBytes(k, gOledBmp, 1024) == 1024;
}
bool drawLoad(int slot) {
  char k[8]; drawKey(slot, k);
  if (prefs.getBytesLength(k) != 1024) return false;
  prefs.getBytes(k, gOledBmp, 1024);
  gBmpHave = true;
  return true;
}
void drawDel(int slot) { char k[8]; drawKey(slot, k); prefs.remove(k); }

/*  Show whatever the phone sent. Called instead of the face renderer. */
void oledShowUser() {
  if (gOledMode == 2 && gBmpHave) {
    memcpy(oledBuf, gOledBmp, 1024);
    for (int p = 0; p < 8; p++) oledDirty[p] = true;
    return;
  }
  /*  Text over a drawing: begin from the picture rather than a blank
   *  screen, then the words are written on top of it. The two layers
   *  stay separate, so either can be moved or resized on its own.    */
  if (gTxOver && gBmpHave) {
    memcpy(oledBuf, gOledBmp, 1024);
    for (int p = 0; p < 8; p++) oledDirty[p] = true;
  } else {
    oledClear();
  }
  int len = (int)strlen(gOledText);
  if (len < 1) { gOledMode = 0; return; }
  int sc = gTxSc < 1 ? 1 : (gTxSc > 4 ? 4 : gTxSc);
  int cw = 6 * sc, chh = 8 * sc;

  if (gOledMode == 3) {                       /* scrolling banner */
    int tw = len * cw;
    int w  = tw + 128;
    uint32_t spd = gScrSpd < 5 ? 5 : gScrSpd;
    /*  64-bit on purpose: millis() * 200 overflows a uint32_t after
     *  about 21 seconds of uptime, which made the banner jump.       */
    uint32_t ph  = (uint32_t)((((uint64_t)millis() * spd) / 1000ULL) % (uint64_t)w);
    int x = (gScrDir < 0) ? (128 - (int)ph)        /* right to left */
                          : ((int)ph - tw);        /* left to right */
    int y = gTxY;
    if (y > 64 - chh) y = 64 - chh;
    if (y < 0) y = 0;
    oledText(x, y, gOledText, sc);
    return;
  }

  /*  Static text, at exactly the spot you dragged it to. Anything that
   *  would run off the right-hand edge wraps onto the next line, so a
   *  long sentence is still readable at any size.                    */
  int per = (128 - (gTxX < 0 ? 0 : (int)gTxX)) / (cw < 1 ? 1 : cw);
  if (per < 1) per = 1;
  if (per > 30) per = 30;
  char line[32];
  int y = gTxY, i = 0;
  while (i < len && y < 64) {
    int n = len - i;
    if (n > per) n = per;
    memcpy(line, gOledText + i, n);
    line[n] = 0;
    oledText(gTxX, y, line, sc);
    i += n;
    y += chh + 1;
  }
}

/*  Cycle the saved drawings, 4 s each. Purely cosmetic, never blocks. */
void drawSlideTick() {
  if (!gSlideOn) return;
  uint32_t now = millis();
  if (now - gSlideAt < 4000) return;
  gSlideAt = now;
  for (int k = 0; k < NDRAW; k++) {
    gSlideIdx = (uint8_t)((gSlideIdx + 1) % NDRAW);
    if (drawLoad(gSlideIdx)) { gOledMode = 2; return; }
  }
  gSlideOn = false;
}

/*  Keeps the face alive when nothing is happening: every few seconds it
 *  drifts to another calm expression instead of sitting on one smile. */
void idleFaceTick() {
  static uint32_t next = 0;
  static uint8_t  k = 0;
  const Face calm[6] = { F_IDLE, F_HAPPY, F_CURIOUS, F_WINK, F_COOL, F_IDLE };
  uint32_t now = millis();
  if (gBusy || gOledMode) { next = now + 6000; return; }
  if (gFaceWant != F_IDLE && gFaceWant != F_HAPPY && gFaceWant != F_CURIOUS &&
      gFaceWant != F_WINK && gFaceWant != F_COOL) return;
  if ((int32_t)(now - next) < 0) return;
  next = now + 7000 + (millis() % 4000);
  k = (uint8_t)((k + 1) % 6);
  faceSet(calm[k], "");
}

void faceRender() {
  if (!gOledOk) return;
  if (gOledMode) {                    /* your drawing / text owns the screen */
    oledShowUser();
    for (int p = 0; p < 8; p++) if (oledDirty[p]) oledPushPage(p);
    return;
  }
  idleFaceTick();
  uint32_t now = millis();

  static char   lastMsg[22] = "\x01";
  static int    lastFace    = -1;      // int, not Face: no valid Face is -1
  static bool   lastBlink   = false;
  static int8_t lastShift   = 0;

  /* blink on the calm faces */
  if (gFaceWant == F_IDLE || gFaceWant == F_HAPPY) {
    if (gBlinking && now - gBlinkAt > 130) { gBlinking = false; gBlinkAt = now; }
    else if (!gBlinking && now - gBlinkAt > OLED_BLINK_MS) { gBlinking = true; gBlinkAt = now; }
  } else if (gBlinking) gBlinking = false;

  /* eyes track the stride while walking */
  if (gFaceWant == F_WALK) gEyeShift = (int8_t)(6.0f * sinf(now / 320.0f));
  else                     gEyeShift = 0;

  gFace = gFaceWant;
  gAnim++;

  if (faceAnimates(gFace) || (int)gFace != lastFace || gBlinking != lastBlink ||
      gEyeShift != lastShift || strcmp(gFaceMsg, lastMsg) != 0) {
    faceDraw();
    lastFace = (int)gFace; lastBlink = gBlinking; lastShift = gEyeShift;
    strncpy(lastMsg, gFaceMsg, sizeof(lastMsg) - 1);
    lastMsg[sizeof(lastMsg) - 1] = 0;
  }
  /*  inside the task we can afford to flush everything at once */
  for (int p = 0; p < 8; p++) if (oledDirty[p]) oledPushPage(p);
}

void faceTask(void*) {
  for (;;) {
    faceRender();
    vTaskDelay(pdMS_TO_TICKS(OLED_FPS_MS));
  }
}

void faceStart() {
  if (!gOledOk) return;
  gFaceTaskOn = true;
  xTaskCreatePinnedToCore(faceTask, "face", 3072, NULL, 1, NULL, AUDIO_CORE);
}

/*  Still called from pumpIO(). If the task is running there is nothing
 *  to do here -- which is the point, the display costs core 1 nothing.
 *  If the task could not start, fall back to servicing it inline.    */
void faceTick() {
  if (gFaceTaskOn || !gOledOk) return;
  static uint32_t next = 0;
  uint32_t now = millis();
  if ((int32_t)(now - next) < 0) { oledService(); return; }
  next = now + OLED_FPS_MS;
  faceRender();
}

#else   /* no OLED compiled in */
/*  enum Face is defined near the top -- see the note there. */
uint8_t  gOledMode = 0;
char     gOledText[64] = "";
uint8_t  gOledBmp[1024];
bool     gBmpHave = false, gSlideOn = false;
uint32_t gSlideAt = 0;
uint8_t  gSlideIdx = 0;
/*  P8 / P9.  Where your words sit on the glass and how they move.
 *  gTxX / gTxY are the top-left corner in OLED pixels, so the phone can
 *  simply drag them. gTxSc is the character scale 1..4 (a character is
 *  6 x 8 pixels at scale 1). gTxOver draws the words ON TOP of the
 *  picture that is already in memory instead of clearing the screen,
 *  which is how text over a drawing works. The banner uses gScrDir
 *  (-1 = right to left, +1 = left to right) and gScrSpd in pixels per
 *  second, so the speed slider is honest rather than a guess.        */
int16_t  gTxX   = 6, gTxY = 24;
uint8_t  gTxSc  = 2;
bool     gTxOver = false;
int8_t   gScrDir = -1;
uint8_t  gScrSpd = 40;
bool drawHas(int) { return false; }
bool drawSave(int) { return false; }
bool drawLoad(int) { return false; }
void drawDel(int)  {}
void oledShowUser()  {}
void drawSlideTick() {}
void idleFaceTick()  {}
void faceSet(Face, const char*) {}
void faceTick()   {}
void faceRender() {}
void faceStart()  {}
bool gOledOk     = false;
bool gFaceTaskOn = false;
uint32_t gAnim      = 0;
uint32_t gFaceCount = 0;
Face gFaceWant = F_BOOT;
#endif  /* USE_OLED */

/*====================================================================
 *  10. MPU6050  --  tilt sensing, balance assist, fall detection
 *
 *  Raw I2C, no extra library. Shares the servo bus: the MPU is 0x68
 *  and the PCA9685 is 0x40, so they coexist.
 *
 *  HOW THE BALANCE ASSIST BEHAVES -- this is the part you asked about.
 *
 *  It is an integrator with a deadband, not a proportional controller,
 *  and that distinction is the whole answer to "will the servos move
 *  every second?".
 *
 *    * While the tilt is inside +/- BAL_DEADBAND degrees the loop does
 *      nothing whatsoever. It does not send a small correction. It
 *      sends no bytes. The servos are not addressed at all.
 *    * Outside the deadband it nudges the trim by BAL_STEP_DEG (1 deg)
 *      and then waits BAL_STEP_MS (150 ms) before even looking again.
 *      So the fastest it can ever move is about 7 degrees per second,
 *      spread over 6 joints. You will see a lean, not a shiver.
 *    * As soon as the lean brings the tilt back inside the deadband it
 *      stops and holds. Steady state is silence.
 *    * Authority is capped at BAL_MAX_DEG. If it hits that cap and is
 *      still not level after BAL_GIVEUP_MS it switches itself off and
 *      tells you, instead of straining against something it cannot fix.
 *
 *  It steers six joints only: the four roll joints together (which is
 *  the sideways lean) and the two ankle pitches mirrored (the forward
 *  lean). It never touches an arm, a knee or a hip pitch, so it can
 *  never interfere with a pose or a routine.
 *
 *  The trim is applied as Joint::bias, i.e. after inv and trim and
 *  outside the pose bookkeeping, so balancing cannot confuse
 *  poseDistance() or make the robot think it is sitting.
 *===================================================================*/

/*  Which way the corrections push. You cannot know these from a
 *  datasheet -- they depend on how your brackets are bolted on. Test
 *  with  baltest roll  and  baltest pitch  (see below): the robot
 *  should lean AGAINST the way you are tipping it. If it leans with
 *  you, flip the matching sign here to -1 and re-upload.             */
#define BAL_ROLL_SIGN    1
#define BAL_PITCH_SIGN   1

#if USE_MPU
#define MPU_ADDR 0x68
bool  gMpuOk    = false;
bool  gBalOn    = BAL_ENABLE;
float gPitchZero = 0, gRollZero = 0;
int8_t gBalRoll = 0, gBalPitch = 0;      // current trim, degrees
bool  gFallen   = false;

bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);              // wake up
  if (Wire.endTransmission() != 0) return false;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00);              // accel +/- 2 g
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); Wire.write(0x03);              // 44 Hz internal filter
  Wire.endTransmission();
  return true;
}

void mpuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom(MPU_ADDR, 6) != 6) return;
  int16_t ax = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t ay = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t az = (int16_t)((Wire.read() << 8) | Wire.read());
  float fx = ax / 16384.0f, fy = ay / 16384.0f, fz = az / 16384.0f;
  float p = atan2f(-fx, sqrtf(fy * fy + fz * fz)) * 180.0f / PI;
  float r = atan2f(fy, fz) * 180.0f / PI;
  /*  ANTI-JITTER: a much heavier low-pass than before. BAL_LP is how
   *  many samples the average leans on, so the one-to-two degree wander
   *  that used to buy a trim step several times a second is averaged out
   *  of existence. A real lean still arrives within a fraction of a
   *  second, because it does not average away.                        */
  static bool seeded = false;
  if (!seeded) { seeded = true; gPitch = p; gRoll = r; return; }
  float k = 1.0f / (float)(BAL_LP < 1 ? 1 : BAL_LP);
  gPitch += (p - gPitch) * k;
  gRoll  += (r - gRoll)  * k;
}

/*  Push the current trim onto the six joints it is allowed to steer.
 *  refresh() dedupes, so joints whose pulse does not change are not
 *  written and the bus stays quiet.                                  */
void balApply() {
  int r = BAL_ROLL_SIGN  * gBalRoll;
  int p = BAL_PITCH_SIGN * gBalPitch;
  int s = gStance;      /* wider feet: hip rolls out, ankle rolls back the
                         * same amount, so the sole stays flat on the floor
                         * instead of standing on its inner edge.          */
  JOINTS[J_LL1].bias = (int8_t)(r + s);  JOINTS[J_RL1].bias = (int8_t)(r - s);
  JOINTS[J_LL5].bias = (int8_t)(r - s);  JOINTS[J_RL5].bias = (int8_t)(r + s);
  JOINTS[J_LL4].bias = (int8_t)(-p);                 // pitch: mirrored
  JOINTS[J_RL4].bias = (int8_t)( p);
  JOINTS[J_LL1].refresh(); JOINTS[J_RL1].refresh();
  JOINTS[J_LL5].refresh(); JOINTS[J_RL5].refresh();
  JOINTS[J_LL4].refresh(); JOINTS[J_RL4].refresh();
}

void balReset() {
  gBalRoll = gBalPitch = 0;
  balApply();
}

/*  One integrator step. Returns true if it moved anything.           */
static bool balStep() {
  float er = gRoll  - gRollZero;
  float ep = gPitch - gPitchZero;
  bool moved = false;

  /*  ANTI-JITTER confirmation counters. A lean has to be past the
   *  deadband AND leaning the same way for BAL_CONFIRM samples in a row
   *  before a single trim step is let out. Sensor noise changes sign
   *  constantly, so it never earns a step; a hand pushing the robot
   *  holds its sign and earns one straight away.                      */
  static int8_t cr = 0, cp = 0;

  /*  Two bands, not one. Past BAL_DEADBAND it corrects; back inside
   *  BAL_RELEASE it gives the trim back one degree at a time and then
   *  does nothing at all. In between it holds whatever it has. That
   *  gap is what stops the servos from buzzing around the threshold. */
  if (fabsf(er) < BAL_RELEASE) {
    cr = 0;
    if (gBalRoll != 0) { gBalRoll += (int8_t)(gBalRoll > 0 ? -1 : 1); moved = true; }
  } else if (fabsf(er) > BAL_DEADBAND) {
    int8_t sg = (er > 0) ? 1 : -1;
    cr = (int8_t)((cr * sg <= 0) ? sg : cr + sg);
    if (abs((int)cr) >= BAL_CONFIRM) {
      cr = 0;
      int s = constrain((int)lroundf(BAL_GAIN * fabsf(er)), 1, BAL_STEP_DEG);
      int v = gBalRoll + ((er > 0) ? -s : s);
      v = constrain(v, -BAL_MAX_DEG, BAL_MAX_DEG);
      if (v != gBalRoll) { gBalRoll = (int8_t)v; moved = true; }
    }
  } else cr = 0;

  if (fabsf(ep) < BAL_RELEASE) {
    cp = 0;
    if (gBalPitch != 0) { gBalPitch += (int8_t)(gBalPitch > 0 ? -1 : 1); moved = true; }
  } else if (fabsf(ep) > BAL_DEADBAND) {
    int8_t sg = (ep > 0) ? 1 : -1;
    cp = (int8_t)((cp * sg <= 0) ? sg : cp + sg);
    if (abs((int)cp) >= BAL_CONFIRM) {
      cp = 0;
      int s = constrain((int)lroundf(BAL_GAIN * fabsf(ep)), 1, BAL_STEP_DEG);
      int v = gBalPitch + ((ep > 0) ? -s : s);
      v = constrain(v, -BAL_MAX_DEG, BAL_MAX_DEG);
      if (v != gBalPitch) { gBalPitch = (int8_t)v; moved = true; }
    }
  } else cp = 0;

  if (moved) balApply();
  return moved;
}

/*  Called from pumpIO(). Reads at 40 Hz, corrects at most every
 *  BAL_STEP_MS, and detects a fall independently of the balancing.   */
void mpuTick() {
  if (!gMpuOk) return;
  static uint32_t lastRead = 0, lastStep = 0, tiltSince = 0, satSince = 0;

  uint32_t now = millis();
  if (now - lastRead < 25) return;
  lastRead = now;
  mpuRead();

  /* ---- fall detection ------------------------------------------
   *  Two completely different situations, and the old code treated
   *  them as one:
   *    idle  + big sustained tilt  = the robot really has fallen
   *    busy  + big tilt            = a push-up, a bow, a dab. Expected.
   *  So while a routine runs the threshold is much larger and, by
   *  default, tilt never interrupts it.                             */
  float dev   = fmaxf(fabsf(gPitch - gPitchZero), fabsf(gRoll - gRollZero));
  float lim   = gBusy ? (float)FALL_DEG_MOVING : (float)FALL_DEG;
  uint32_t hold = gBusy ? (uint32_t)FALL_HOLD_MOVING_MS : (uint32_t)FALL_HOLD_MS;
  if (dev <= lim) { tiltSince = 0; if (!gBusy) gFallen = false; }
  else {
    if (tiltSince == 0) tiltSince = now;
    else if (now - tiltSince > hold) {
      if (!gBusy) {
        if (!gFallen) {
          gFallen = true;
          Serial.printf("!! fallen: tilt %.0f / %.0f deg\n",
                        gPitch - gPitchZero, gRoll - gRollZero);
          faceSet(F_ALERT, "FALLEN");
        }
      } else if (TILT_GUARD_MOVING) {
        if (!gAbort) {
          Serial.printf("!! %.0f deg during a move -- stopping\n", dev);
          gAbort = true;
        }
      }
    }
  }

  /* ---- balance assist ---- */
  if (!gBalOn) return;
  if (BAL_ONLY_IDLE && gBusy) return;
  if (gAutoUpRun) return;             // recovery owns the servos right now
  if (gFlexOn && flexLive()) return;  // the glove is driving: hands off
  if (now - gLastMotionMs < BAL_SETTLE_MS) return;    // let the pose settle
  if (poseDistance(POSE_STAND) > 25) return;          // only while standing
  if (now - lastStep < BAL_STEP_MS) return;
  lastStep = now;

  /*  Require the tilt to persist. A knock, or the robot's own inertia at
   *  the end of a move, is not something worth correcting.            */
  static uint32_t offSince = 0;
  bool nowOff = (fabsf(gRoll - gRollZero) > BAL_DEADBAND ||
                 fabsf(gPitch - gPitchZero) > BAL_DEADBAND);
  if (nowOff) { if (offSince == 0) offSince = now; }
  else offSince = 0;
  bool settling = (gBalRoll != 0 || gBalPitch != 0);
  if (nowOff && (now - offSince) < BAL_HOLD_MS && !settling) return;

  bool sat = (abs(gBalRoll) >= BAL_MAX_DEG || abs(gBalPitch) >= BAL_MAX_DEG);
  bool off = (fabsf(gRoll - gRollZero) > BAL_DEADBAND ||
              fabsf(gPitch - gPitchZero) > BAL_DEADBAND);
  if (sat && off) {
    if (satSince == 0) satSince = now;
    else if (now - satSince > BAL_GIVEUP_MS) {
      gBalOn = false;
      satSince = 0;
      Serial.println(F("balance: out of travel and still not level -- giving up."));
      Serial.println(F("  the robot is leaning further than 8 deg of trim can fix."));
      Serial.println(F("  check the feet are flat and the supply is not sagging,"));
      Serial.println(F("  then  bal zero  while it is standing straight."));
      faceSet(F_ALERT, "BAL GIVE UP");
    }
  } else satSince = 0;

  balStep();
}

/*====================================================================
 *  BALANCE SUSPEND / RESUME
 *  Every command runs with the trim stood down, then the trim comes
 *  back by itself if (a) it was on before, and (b) the robot really is
 *  standing again. "balance on" is therefore never a requirement for
 *  any action -- every move works identically with it on or off.
 *===================================================================*/
bool     gBalSaved  = false;   // what balance was before the action
bool     gBalSusp   = false;   // suspended right now
bool     gBalPend   = false;   // waiting to switch back on
uint32_t gBalPendMs = 0;

void balSuspend() {
  if (gBalSusp) return;
  gBalSusp  = true;
  /*  If a resume was still owed from an earlier action -- you sat down,
   *  so it never got the chance -- keep that promise instead of
   *  overwriting it with the current (off) state. This is why balance
   *  still comes back after  sit down  then  stand up.               */
  if (!gBalPend) gBalSaved = gBalOn;
  gBalPend  = false;
  if (gBalOn) {
    gBalOn = false;
    balReset();                /* hand the joints back, no leftover trim */
  }
}
void balResume() {
  if (!gBalSusp) return;
  gBalSusp = false;
  if (!gBalSaved) return;      /* it was off before: leave it off */
  gBalPend   = true;
  gBalPendMs = millis() + BAL_RESUME_MS;
}
/*  Called from pumpIO(). Waits for the robot to be idle, settled and
 *  actually upright before it lets the trim act again.               */
void balResumeTick() {
  if (!gBalPend || gBusy) return;
  if ((int32_t)(millis() - gBalPendMs) < 0) return;
  /*  Not standing yet -- most likely sitting. Keep waiting rather than
   *  forgetting: the moment the robot is upright again, balance returns
   *  by itself.                                                      */
  if (poseDistance(POSE_STAND) > 25) {
    static uint32_t said = 0;
    if (millis() - said > 8000) {
      said = millis();
      Serial.println(F("balance is waiting -- it comes back when you are standing"));
    }
    gBalPendMs = millis() + 1500;
    return;
  }
  gBalPend = false;
  gBalOn   = true;
  balReset();
  Serial.println(F("balance back ON"));
}

void balZero() {
  if (!gMpuOk) { Serial.println(F("no MPU6050 found")); return; }
  balReset();
  float p = 0, r = 0;
  for (int i = 0; i < 40; i++) { mpuRead(); delay(20); }   // let it settle
  for (int i = 0; i < 40; i++) { mpuRead(); delay(20); p += gPitch; r += gRoll; }
  gPitchZero = p / 40.0f;
  gRollZero  = r / 40.0f;
  prefs.putFloat("pz", gPitchZero);
  prefs.putFloat("rz", gRollZero);
  Serial.printf("level recorded: pitch %.1f  roll %.1f (saved to flash)\n",
                gPitchZero, gRollZero);
}

/*  Sign finder. Trims one axis to its limit and back so you can watch
 *  which way the robot leans, without having to reason about it.     */
void balTest(bool rollAxis) {
  Serial.printf("driving the %s trim +%d, then -%d, then back to 0.\n",
                rollAxis ? "roll (sideways)" : "pitch (forward/back)",
                BAL_MAX_DEG, BAL_MAX_DEG);
  Serial.println(F("watch which way it leans. It must lean AGAINST a push."));
  bool was = gBalOn; gBalOn = false;
  for (int v = 0; v <= BAL_MAX_DEG; v++) {
    if (rollAxis) gBalRoll = v; else gBalPitch = v;
    balApply(); if (!naptime(90)) break;
  }
  if (!naptime(700)) { }
  for (int v = BAL_MAX_DEG; v >= -BAL_MAX_DEG; v--) {
    if (rollAxis) gBalRoll = v; else gBalPitch = v;
    balApply(); if (!naptime(90)) break;
  }
  if (!naptime(700)) { }
  for (int v = -BAL_MAX_DEG; v <= 0; v++) {
    if (rollAxis) gBalRoll = v; else gBalPitch = v;
    balApply(); if (!naptime(90)) break;
  }
  balReset();
  gBalOn = was;
  Serial.printf("done. If it leaned the WRONG way, set BAL_%s_SIGN to -1.\n",
                rollAxis ? "ROLL" : "PITCH");
}
#else
bool  gMpuOk = false, gBalOn = false, gFallen = false;
int8_t gBalRoll = 0, gBalPitch = 0;
float gPitchZero = 0, gRollZero = 0;
void mpuTick() {}
void balReset() {}
/*  Stance still works with the sensor compiled out -- it is only a bias. */
void balApply() {
  int s = gStance;
  JOINTS[J_LL1].bias = (int8_t)( s);  JOINTS[J_RL1].bias = (int8_t)(-s);
  JOINTS[J_LL5].bias = (int8_t)(-s);  JOINTS[J_RL5].bias = (int8_t)( s);
  JOINTS[J_LL1].refresh(); JOINTS[J_RL1].refresh();
  JOINTS[J_LL5].refresh(); JOINTS[J_RL5].refresh();
}
void balZero()  { Serial.println(F("MPU6050 not enabled -- set USE_MPU to 1")); }
void balTest(bool) { Serial.println(F("MPU6050 not enabled")); }
/*  The suspend / resume engine still exists with the sensor compiled
 *  out, so the command runner does not need a single #if around it.  */
bool gBalSaved = false, gBalSusp = false, gBalPend = false;
void balSuspend()   {}
void balResume()    {}
void balResumeTick() {}
#endif  /* USE_MPU */

/*====================================================================
 *  10. FORWARD DECLARATIONS
 *
 *  Only here so the file is valid C++ in the order it is written --
 *  the dance and gangnam routines call the individual moves that are
 *  defined below them. The Arduino IDE would generate these for you;
 *  writing them out means the sketch also compiles anywhere else.
 *===================================================================*/
void stand_straight();      void say_hi();              void shake_hand();
void hands_up();            void hands_down();          void right_bicep();
void left_bicep();          void double_biceps();       void do_bow();
void maan_karate();         void ape_move();            void exercise_one();
void side_bend();           void sit_down();            void stand_up();
void do_dab();              void push_ups();            void push_ups_recover();
void gangnam_style_dance(); void gangnam_style1();      void gangnam_style3();
void gangnam_style2_right(); void gangnam_style2_left();
void dance_steps();         void dance_move1();         void dance_move2();
void dance_move3();         void dance_move4();         void dance_move5();
void dance_move6();         void dance_move7();
void old_walk();            void old_turn_left();       void old_turn_right();
void walk_forward();        void turn_left_new();       void turn_right_new();
bool goToPose(const int16_t* target);
void queueCommand(const String &raw);
void quietReport();

/*====================================================================
 *  11. THE NEW GAIT
 *
 *  Five numbers describe both legs completely:
 *
 *      hl   left  hip pitch, degrees forward from standing
 *      hr   right hip pitch, degrees forward from standing
 *      kl   left  knee bend, degrees (0 = straight)
 *      kr   right knee bend, degrees (0 = straight)
 *      r    roll: sideways weight shift, + = onto the RIGHT foot
 *
 *  Everything else is derived, and derived in a way that keeps three
 *  geometric rules true in EVERY frame, not merely at the endpoints:
 *
 *    FLAT SOLE      d(ankle) = d(hip) + d(knee), each leg on its own.
 *                   This is what stops the robot toppling: however the
 *                   thigh and shin are arranged, the foot stays
 *                   parallel to the floor.
 *    ROLL COUPLING  ll1 = rl1 = ll5 = rl5, always. The hips and ankles
 *                   roll as one parallel linkage, so the torso stays
 *                   upright while the weight moves sideways.
 *    KNEE MIRROR    the left knee only ever bends by DECREASING its
 *                   angle, the right only by INCREASING its own.
 *                   Getting this backwards is what made the legs fold
 *                   into each other in the old sitting bug.
 *
 *  Those three relations are linear in (hl, hr, kl, kr, r), and the
 *  interpolation below is linear in the pose too, so they survive
 *  mid-move as well -- there is no frame anywhere in a step where the
 *  sole is not flat.
 *
 *  Standing is hl = hr = kl = kr = r = 0, which reproduces POSE_STAND
 *  exactly. Every walk and every turn begins and ends there, so unlike
 *  the old routine there is nothing left over to accumulate.
 *===================================================================*/
/*  struct GaitPose is defined near the top -- see the note there. */

const GaitPose GAIT_HOME = { 0, 0, 0, 0, 0 };

void gaitApply(const GaitPose &g) {
  int R  = (int)lroundf(g.r) * GAIT_LEAN_SIGN;
  int Hl = (int)lroundf(g.hl), Kl = (int)lroundf(g.kl);
  int Hr = (int)lroundf(g.hr), Kr = (int)lroundf(g.kr);

  /* roll: one number, four joints, no exceptions */
  ll1.write(90 + R);
  ll5.write(90 + R);
  rl1.write(90 + R);
  rl5.write(90 + R);

  /* left leg */
  ll2.write(30  + Hl);
  ll3.write(150 - Kl);
  ll4.write(150 + (Hl - Kl));

  /* right leg -- mirrored, which is why the signs flip */
  rl2.write(150 - Hr);
  rl3.write(30  + Kr);
  rl4.write(30  - (Hr - Kr));

#if GAIT_ARM_SWING
  /* Cosmetic only. lh2 sits at its 180 end stop and rh2 at its 0 end
   * stop when standing, so the arms cannot swing in opposition without
   * clipping -- they swing as a mirrored pair instead.               */
  int s = (int)lroundf(6.0f + 6.0f * (g.hl / (float)GAIT_HIP_AMP));
  s = constrain(s, 0, 12);
  lh2.write(180 - s);
  rh2.write(0   + s);
#endif
}

/*  Cosine-eased move from one gait pose to another. Returns false if
 *  the move was interrupted.                                         */
bool gaitMove(const GaitPose &a, const GaitPose &b, uint32_t ms) {
  uint32_t dur = scaled(ms);
  int steps = (int)(dur / GAIT_FRAME_MS);
  if (steps < 1) steps = 1;
  for (int s = 1; s <= steps; s++) {
    float u = (float)s / (float)steps;
    float e = 0.5f * (1.0f - cosf(PI * u));
    GaitPose g;
    g.hl = a.hl + (b.hl - a.hl) * e;
    g.hr = a.hr + (b.hr - a.hr) * e;
    g.kl = a.kl + (b.kl - a.kl) * e;
    g.kr = a.kr + (b.kr - a.kr) * e;
    g.r  = a.r  + (b.r  - a.r ) * e;
    gaitApply(g);
    if (!naptime(GAIT_FRAME_MS)) return false;
  }
  gaitApply(b);
  return true;
}

#define GAITGO(from, to, ms) do { if (!gaitMove(from, to, ms)) return; \
                                  (from) = (to); } while (0)

/*====================================================================
 *  WALK FORWARD
 *
 *  One half-step = one foot placed in front of the other. The legs
 *  counter-rotate: while the swinging leg's hip goes from behind to in
 *  front, the standing leg's hip goes from in front to behind, which
 *  is what actually carries the body forward. hl = -hr throughout.
 *
 *  Each half-step is five phases:
 *      1  shift the weight onto the foot that is about to take it
 *      2  bend the swinging knee  -- THIS is the bit the old routine
 *         never did, and why the foot used to scrape
 *      3  swing: both hips counter-rotate
 *      4  straighten the knee, putting the foot down in front
 *      5  settle
 *
 *  The last half-step brings the hips back to centre instead of to the
 *  far side, so the robot finishes standing square.
 *===================================================================*/
void walk_forward() {
  GaitPose g = GAIT_HOME;
  bool swingLeft = true;
  int  half = (gWalkHalf < 1) ? 1 : gWalkHalf;

  Serial.printf("walking: %d half-steps, hip +/-%d, knee lift %d, roll %d\n",
                half, GAIT_HIP_AMP, GAIT_KNEE_LIFT, GAIT_ROLL);
  faceSet(F_WALK, "WALKING");

  for (int i = 0; i <= half; i++) {
    bool  finish = (i == half);            // the squaring-up half-step
    float phi    = finish ? 0.0f
                          : (swingLeft ? (float)GAIT_HIP_AMP
                                       : -(float)GAIT_HIP_AMP);
    GaitPose t;

    /* 1. weight onto the STANDING foot (the one we are not lifting) */
    t = g;  t.r = swingLeft ? (float)GAIT_ROLL : -(float)GAIT_ROLL;
    GAITGO(g, t, GAIT_SHIFT_MS);
    if (!naptime(GAIT_PAUSE_MS)) return;

    /* 2. lift: bend the swinging knee, flat sole keeps the foot level */
    t = g;
    if (swingLeft) t.kl = GAIT_KNEE_LIFT; else t.kr = GAIT_KNEE_LIFT;
    GAITGO(g, t, GAIT_LIFT_MS);

    /* 3. swing: the hips counter-rotate, body travels forward */
    t = g;  t.hl = phi;  t.hr = -phi;
    GAITGO(g, t, GAIT_SWING_MS);

    /* 4. lower: knee straight, foot lands in front */
    t = g;
    if (swingLeft) t.kl = 0; else t.kr = 0;
    GAITGO(g, t, GAIT_LOWER_MS);
    if (!naptime(GAIT_PAUSE_MS)) return;

    swingLeft = !swingLeft;
    Serial.printf("  half-step %d/%d done\n", i + 1, half + 1);
  }

  /* level the hips back up -- now identical to POSE_STAND */
  GaitPose t = g;  t.r = 0;
  GAITGO(g, t, GAIT_SHIFT_MS);
}

/*====================================================================
 *  TURN IN PLACE
 *
 *  There is no yaw joint anywhere on this robot, so a turn cannot be
 *  commanded directly -- it has to be shuffled out. The weight goes
 *  onto the foot we pivot around and STAYS there for the whole turn.
 *  The other leg lifts, swings forward, is put down, and is then
 *  dragged back to centre while it is only lightly loaded. That drag
 *  is what rotates the body.
 *
 *  Because of that, how far it turns per cycle depends on the floor.
 *  On carpet it will be a lot, on smooth tiles very little. Change
 *  TURN_CYCLES to taste rather than expecting a fixed number of
 *  degrees.
 *===================================================================*/
void turn_in_place(bool toRight) {
  bool stepLeft = toRight;         // turn right -> pivot on the right foot
  GaitPose g = GAIT_HOME, t;

  Serial.printf("turning %s: %d cycles, pivot on the %s foot\n",
                toRight ? "right" : "left", TURN_CYCLES,
                stepLeft ? "right" : "left");
  faceSet(F_WALK, toRight ? "TURN RIGHT" : "TURN LEFT");

  /* weight onto the pivot foot, once, for the whole manoeuvre */
  t = g;  t.r = stepLeft ? (float)GAIT_ROLL : -(float)GAIT_ROLL;
  GAITGO(g, t, GAIT_SHIFT_MS);
  if (!naptime(GAIT_PAUSE_MS)) return;

  for (int c = 0; c < TURN_CYCLES; c++) {
    /* lift the stepping leg */
    t = g;
    if (stepLeft) t.kl = GAIT_KNEE_LIFT; else t.kr = GAIT_KNEE_LIFT;
    GAITGO(g, t, GAIT_LIFT_MS);

    /* swing it forward -- only that leg's hip moves, the pivot leg
     * stays exactly where it is, which is what makes this a turn and
     * not a step                                                    */
    t = g;
    if (stepLeft) t.hl = TURN_HIP; else t.hr = TURN_HIP;
    GAITGO(g, t, GAIT_SWING_MS);

    /* put it down */
    t = g;
    if (stepLeft) t.kl = 0; else t.kr = 0;
    GAITGO(g, t, GAIT_LOWER_MS);
    if (!naptime(GAIT_PAUSE_MS)) return;

    /* drag it back to centre: the body rotates about the pivot foot */
    t = g;
    if (stepLeft) t.hl = 0; else t.hr = 0;
    GAITGO(g, t, GAIT_SWING_MS);
    if (!naptime(GAIT_PAUSE_MS)) return;
    Serial.printf("  cycle %d/%d\n", c + 1, TURN_CYCLES);
  }

  t = g;  t.r = 0;
  GAITGO(g, t, GAIT_SHIFT_MS);
}

void turn_left_new()  { turn_in_place(false); }
void turn_right_new() { turn_in_place(true);  }

/*====================================================================
 *  12. YOUR ROUTINES -- UNTOUCHED
 *
 *  Every one of these 31 routines is copied out of your sketch
 *  character for character. Same angles, same loop bounds, same order,
 *  same delays. I did not "improve" any of them, because you said the
 *  file you sent is the one that suits the robot, and I agree -- the
 *  arm and torso work in here is good.
 *
 *  What changed is underneath them, not inside them: the rate limiter
 *  in Joint::write(), the duplicate-write filter, and the required
 *  start pose in CMDS[]. So these run exactly as before, just without
 *  slamming into position when they begin and without buzzing when
 *  they hold.
 *
 *  The two exceptions are further down: your walking and turning
 *  routines are kept as old_walk() / old_turn_left() / old_turn_right()
 *  so you can still play them and compare, but the default `walk`
 *  command now runs the new gait in section 11.
 *===================================================================*/

void stand_straight() {
  head.write(90);
  rh1.write(0);
  rh2.write(0);
  rh3.write(20);
  rl1.write(90);
  rl2.write(150);
  rl3.write(30);
  rl4.write(30);
  rl5.write(90);
  lh1.write(180);
  lh2.write(180);
  lh3.write(160);
  ll1.write(90);
  ll2.write(30);
  ll3.write(150);
  ll4.write(150);
  ll5.write(90);
}

//********************say hi********************//
void say_hi() {
  for (int i = 0; i <= 180; i++) {
    rh1.write(0 + i);
    if (i <= 90) {
      rh2.write(0 + i);
    }
    if (i <= 40) {
      rh3.write(20 + i);
    }
    DLY(15);
  }
  for (int i = 1; i <= 3; i++) {
    for (int i = 0; i <= 60; i++) {
      rh3.write(60 + i);
      DLY(15);
    }
    for (int i = 0; i <= 60; i++) {
      rh3.write(120 - i);
      DLY(15);
    }
  }
  for (int i = 0; i <= 180; i++) {
    rh1.write(180 - i);
    if (i <= 90) {
      rh2.write(90 - i);
    }
    if (i <= 40) {
      rh3.write(60 - i);
    }
    DLY(15);
  }
}

//********************shake hand********************//
void shake_hand() {
  for (int i = 0; i <= 40; i++) {
    rh1.write(0 + i);
    DLY(10);
  }
  for (int i = 0; i <= 3; i++) {
    for (int i = 0; i <= 30; i++) {
      rh1.write(40 + i);
      DLY(15);
    }
    for (int i = 0; i <= 30; i++) {
      rh1.write(70 - i);
      DLY(15);
    }
  }
  for (int i = 0; i <= 40; i++) {
    rh1.write(40 - i);
    DLY(10);
  }
}

//********************hands up********************//
void hands_up() {
  for (int i = 0; i <= 180; i++) {
    rh1.write(0 + i);
    lh1.write(180 - i);
    DLY(15);
  }
}

//********************hands down********************//
void hands_down() {
  for (int i = 0; i <= 180; i++) {
    rh1.write(180 - i);
    lh1.write(0 + i);
    DLY(15);
  }
}

//********************right bicep********************//
void right_bicep() {
  for (int i = 0; i <= 180; i++) {
    rh1.write(0 + i);
    if (i <= 90) {
      rh2.write(0 + i);
    }
    if (i <= 60) {
      head.write(90 - i);
    }
    DLY(15);
  }
  for (int i = 0; i <= 120; i++) {
    rh3.write(20 + i);
    DLY(15);
  }
  DLY(3000);
  for (int i = 0; i <= 120; i++) {
    rh3.write(140 - i);
    DLY(15);
  }
  for (int i = 0; i <= 180; i++) {
    rh1.write(180 - i);
    if (i <= 90) {
      rh2.write(90 - i);
    }
    if (i <= 60) {
      head.write(30 + i);
    }
    DLY(15);
  }
}

//********************left bicep********************//
void left_bicep() {
  for (int i = 0; i <= 180; i++) {
    lh1.write(180 - i);
    if (i <= 90) {
      lh2.write(180 - i);
    }
    if (i <= 60) {
      head.write(90 + i);
    }
    DLY(15);
  }
  for (int i = 0; i <= 120; i++) {
    lh3.write(160 - i);
    DLY(15);
  }
  DLY(3000);
  for (int i = 0; i <= 120; i++) {
    lh3.write(40 + i);
    DLY(15);
  }
  for (int i = 0; i <= 180; i++) {
    lh1.write(0 + i);
    if (i <= 90) {
      lh2.write(90 + i);
    }
    if (i <= 60) {
      head.write(150 - i);
    }
    DLY(15);
  }
}

//********************double biceps********************//
void double_biceps() {
  for (int i = 0; i <= 180; i++) {
    lh1.write(180 - i);
    rh1.write(0 + i);
    if (i <= 90) {
      lh2.write(180 - i);
      rh2.write(0 + i);
    }
    DLY(15);
  }
  for (int i = 0; i <= 120; i++) {
    lh3.write(160 - i);
    rh3.write(20 + i);
    DLY(15);
  }
  DLY(3000);
  for (int i = 0; i <= 120; i++) {
    lh3.write(40 + i);
    rh3.write(140 - i);
    DLY(15);
  }
  for (int i = 0; i <= 180; i++) {
    lh1.write(0 + i);
    rh1.write(180 - i);
    if (i <= 90) {
      lh2.write(90 + i);
      rh2.write(90 - i);
    }
    DLY(15);
  }
}

//********************do bow********************//
void do_bow() {
  for (int i = 0; i <= 10; i++) {
    ll2.write(30 + (i * 2));
    rl2.write(150 - (i * 2));
    ll4.write(150 + i);
    rl4.write(30 - i);
    DLY(100);
  }
  DLY(3000);
  for (int i = 0; i <= 10; i++) {
    ll2.write(50 - (i * 2));
    rl2.write(130 + (i * 2));
    ll4.write(160 - i);
    rl4.write(20 + i);
    DLY(100);
  }
}

//********************maan karate********************//
void maan_karate() {
  for (int i = 0; i <= 180; i++) {
    if (i <= 90) {
      head.write(90 + i);
    }
    if (i <= 120) {
      lh2.write( 180 - i);
    }
    if (i <= 20) {
      lh3.write( 160 + i);
    }
    rh1.write( 0 + i);
    if (i <= 120) {
      rh2.write( 0 + i);
    }
    if (i <= 130) {
      rh3.write( 20 + i);
    }
    if (i <= 10) {
      rl1.write( 90 - i);
      rl5.write( 90 - i);
    }
    if (i <= 10) {
      rl2.write( 150 - i);
      rl3.write( 30 + i);
      rl4.write( 30 + i);
    }
    DLY(15);
  }
  DLY(5000);
  for (int i = 0; i <= 180; i++) {
    if (i <= 90) {
      head.write(180 - i);
    }
    if (i <= 120) {
      lh2.write( 60 + i);
    }
    if (i <= 20) {
      lh3.write( 180 - i);
    }
    rh1.write( 180 - i);
    if (i <= 120) {
      rh2.write( 120 - i);
    }
    if (i <= 130) {
      rh3.write( 150 - i);
    }
    if (i <= 10) {
      rl1.write( 80 + i);
      rl5.write( 80 + i);
    }
    if (i <= 10) {
      rl2.write( 140 + i);
      rl3.write( 40 - i);
      rl4.write( 40 - i);
    }
    DLY(15);
  }
}

//********************ape move********************//
void ape_move() {
  for (int i = 0; i <= 180; i++) {
    if (i <= 90) {
      lh1.write( 180 - i);
      rh1.write( 0 + i);
    }
    if (i <= 20) {
      lh2.write( 180 - i);
      rh2.write( 0 + i);
    }
    DLY(15);
  }
  DLY(100);
  for (int i = 0; i <= 100; i++) {
    if (i <= 50) {
      head.write(90 + i);
    }
    lh3.write( 160 - i);
    DLY(10);
  }
  for (int i = 1; i <= 3; i++) {
    for (int i = 0; i <= 100; i++) {
      head.write(140 - i);
      rh3.write( 20 + i);
      lh3.write( 60 + i);
      DLY(10);
    }
    for (int i = 0; i <= 100; i++) {
      head.write(40 + i);
      rh3.write( 120 - i);
      lh3.write( 160 - i);
      DLY(10);
    }
  }
  for (int i = 0; i <= 100; i++) {
    if (i <= 50) {
      head.write(140 - i);
    }
    lh3.write( 60 + i);
    DLY(10);
  }
  DLY(100);
  for (int i = 0; i <= 180; i++) {
    if (i <= 90) {
      lh1.write( 90 + i);
      rh1.write( 90 - i);
    }
    if (i <= 20) {
      lh2.write( 160 + i);
      rh2.write( 20 - i);
    }
    DLY(15);
  }
}

//********************exercise one********************//
void exercise_one() {
  for (int i = 0; i <= 90; i++) {
    rh1.write(0 + i);
    lh1.write(180 - i);
    DLY(15);
  }
  DLY(2000);
  for (int i = 0; i <= 90; i++) {
    rh1.write(90 + i);
    lh1.write(90 - i);
    DLY(15);
  }
  DLY(2000);
  for (int i = 0; i <= 90; i++) {
    rh1.write(180 - i);
    lh1.write(0 + i);
    rh2.write(0 + i);
    lh2.write(180 - i);
    DLY(15);
  }
  DLY(2000);
  for (int i = 0; i <= 90; i++) {
    rh1.write(90 - i);
    lh1.write(90 + i);
    rh2.write(90 - i);
    lh2.write(90 + i);
    DLY(15);
  }
  DLY(2000);
}

//********************exercise one side bend********************//
void side_bend() {
  //right
  for (int i = 0; i <= 180; i++) {
    rh1.write(0 + i);
    if (i % 4 == 0) {
      lh2.write(180 - i / 4);//135
    }
    if (i % 3 == 0) {
      lh3.write(160 - i / 3);//100
    }
    DLY(15);
  }
  for (int i = 0; i <= 60; i++) {
    rh3.write(20 + i);
    if (i % 6 == 0) {
      rl1.write( 90 - i / 6);
      rl5.write( 90 - i / 6);
      ll1.write( 90 - i / 6);
      ll5.write( 90 - i / 6);
    }
    DLY(30);
  }
  for (int i = 0; i <= 60; i++) {
    rh3.write(80 - i);
    if (i % 6 == 0) {
      rl1.write( 80 + i / 6);
      rl5.write( 80 + i / 6);
      ll1.write( 80 + i / 6);
      ll5.write( 80 + i / 6);
    }
    DLY(30);
  }
  for (int i = 0; i <= 180; i++) {
    rh1.write(180 - i);
    if (i % 4 == 0) {
      lh2.write(135 + i / 4);//135
    }
    if (i % 3 == 0) {
      lh3.write(100 + i / 3);//100
    }
    DLY(15);
  }
  DLY(1000);
  //left
  for (int i = 0; i <= 180; i++) {
    lh1.write(180 - i);
    if (i % 4 == 0) {
      rh2.write(0 + i / 4);//45
    }
    if (i % 3 == 0) {
      rh3.write(20 + i / 3);//80
    }
    DLY(15);
  }
  for (int i = 0; i <= 60; i++) {
    lh3.write(160 - i);
    if (i % 6 == 0) {
      rl1.write( 90 + i / 6);
      rl5.write( 90 + i / 6);
      ll1.write( 90 + i / 6);
      ll5.write( 90 + i / 6);
    }
    DLY(30);
  }
  for (int i = 0; i <= 60; i++) {
    lh3.write(100 + i);
    if (i % 6 == 0) {
      rl1.write( 100 - i / 6);
      rl5.write( 100 - i / 6);
      ll1.write( 100 - i / 6);
      ll5.write( 100 - i / 6);
    }
    DLY(30);
  }
  for (int i = 0; i <= 180; i++) {
    lh1.write(0 + i);
    if (i % 4 == 0) {
      rh2.write(45 - i / 4);//45
    }
    if (i % 3 == 0) {
      rh3.write(80 - i / 3);//80
    }
    DLY(15);
  }
}

//********************sit down********************//
void sit_down() {
  for (int i = 0; i <= 60; i++) {
    if (i <= 30) {
      lh2.write(180 - i);
      rh2.write(0 + i);
      lh1.write(180 - i);
      rh1.write(0 + i);
    }
    if (i % 2 == 0) {
      rh3.write(20 + i / 2);
      lh3.write(160 - i / 2);
    }
    rl2.write(150 - i);
    ll2.write(30 + i);     // ll2 is not inverted: this is 30 -> 90
    rl3.write(30 + (i * 2));
    ll3.write(150 - (i * 2));
    rl4.write(30 + i);
    ll4.write(150 - i);
    DLY(100);
  }
}

//********************stand up********************//
void stand_up() {
  for (int i = 0; i <= 60; i++) {
    if (i <= 30) {
      lh2.write(150 + i);
      rh2.write(30 - i);
      lh1.write(150 + i);
      rh1.write(30 - i);
    }
    if (i % 2 == 0) {
      rh3.write(50 - i / 2);
      lh3.write(130 + i / 2);
    }
    rl2.write(90 + i);
    ll2.write(90 - i);
    rl3.write(150 - (i * 2));
    ll3.write(30 + (i * 2));
    rl4.write(90 - i);
    ll4.write(90 + i);
    DLY(100);
  }
}

//********************dab********************//
void do_dab() {
  for (int i = 0; i <= 90; i++) {
    lh2.write(180 - i);
    rh1.write(0 + i);
    if (i <= 70) {
      rh3.write(20 + i);
    }
    if (i <= 20) {
      lh3.write(160 + i);
    }
    if (i % 10 == 0) {
      rl2.write(150 - (i / 10));
      ll2.write(30 + (i / 10));
    }
    DLY(15);
  }
  DLY(5000);
  for (int i = 0; i <= 90; i++) {
    lh2.write(90 + i);
    rh1.write(90 - i);
    if (i <= 70) {
      rh3.write(90 - i);
    }
    if (i <= 20) {
      lh3.write(180 - i);
    }
    if (i % 10 == 0) {
      rl2.write(141 + (i / 10));
      ll2.write(39 - (i / 10));
    }
    DLY(15);
  }
}

/*====================================================================
 *  PART 5a. segGlide() -- move a whole GROUP of joints together
 *
 *  Every routine in this sketch writes its joints one after another
 *  inside a loop: rl2, then ll2, then rl3, then ll3... Those writes all
 *  go down the same I2C bus, so the joints of one "step" really do
 *  arrive a millisecond or two apart, and when a step is large the rate
 *  limiter inside Joint::write() fans that ONE joint out over several
 *  milliseconds while the others stand still. On 1-degree steps you
 *  cannot see it. On the big entry moves of a push-up you can, and that
 *  is exactly the jerkiness you are describing.
 *
 *  segGlide() takes a list of joints with their destinations and walks
 *  ALL of them from where they are now to where they should be, in lock
 *  step, one interpolated fraction at a time, with a smooth ease in and
 *  ease out so nothing snatches at either end. It uses writeNow(), so
 *  the per-joint rate limiter never fires and never fights the glide.
 *
 *  It obeys STOP exactly the way DLY() does, because naptime() returns
 *  false the moment gAbort is set, and it hands that  false  back so the
 *  caller can bail out cleanly. It does NOT read the tilt sensor, so a
 *  deliberate push-up can never be cut short by a lean.
 *
 *  Nothing else in the sketch is changed by this: it is a new helper,
 *  and only push_ups() uses it.
 *===================================================================*/
/*  ---- WHY THIS TAKES TWO PLAIN ARRAYS AND NOT A struct --------------
 *  This is the exact cause of the
 *
 *        error: 'Seg' was not declared in this scope
 *
 *  you saw in the IDE, and it is worth writing down because it catches
 *  everybody once. A .ino file is NOT compiled as you wrote it. Before
 *  it reaches the compiler the Arduino build system scans it, generates
 *  a prototype for every function it finds, and pastes all of those
 *  prototypes in at the line of the FIRST function in the sketch --
 *  which in this sketch is up around the audio code, roughly six
 *  thousand lines above here.
 *
 *  So the compiler really sees:
 *
 *        bool segGlide(Seg* s, int n, uint32_t ms);   <-- line ~800
 *        ...
 *        struct Seg { ... };                          <-- line ~3900
 *
 *  and of course 'Seg' does not exist yet at line 800. Nothing is wrong
 *  with the code you can see; the generated line is what fails, which is
 *  why the error points at the CALLS and not at the struct.
 *
 *  Renaming the struct (Seg -> PushupSeg) cannot help -- the generated
 *  prototype is renamed with it. The only reliable cures are to keep
 *  user-defined types out of the signature, or to hand-write the
 *  prototype above the struct. This uses the first, because it is
 *  immune to every Arduino/ArduinoDroid version: the parameters are now
 *  a list of JOINTS[] indices and a list of target angles, and uint8_t
 *  and int16_t are always already declared.
 *
 *  Behaviour is identical to before -- same easing, same STOP handling.
 *===================================================================*/
#define SEG_MAX 16                    // most joints one phase can carry

bool segGlide(const uint8_t* jix, const int16_t* to, int n, uint32_t ms) {
  if (n <= 0)      return !gAbort;
  if (n > SEG_MAX) n = SEG_MAX;
  int16_t from[SEG_MAX];
  for (int k = 0; k < n; k++) from[k] = JOINTS[jix[k]].last;

  uint32_t total = scaled(ms);
  if (total < (uint32_t)SEG_STEP_MS) total = SEG_STEP_MS;
  int steps = (int)(total / SEG_STEP_MS);
  if (steps < 1) steps = 1;

  for (int i = 1; i <= steps; i++) {
    /*  ease in / ease out, 3t^2 - 2t^3, done in integer maths on a
     *  0..1000 scale and grouped so nothing can overflow.            */
    int32_t t = (int32_t)i * 1000 / steps;
    int32_t e = (3 * t * t) / 1000 - ((2 * t * t / 1000) * t) / 1000;
    if (e < 0)    e = 0;
    if (e > 1000) e = 1000;
    for (int k = 0; k < n; k++) {
      int32_t d = (int32_t)to[k] - (int32_t)from[k];
      JOINTS[jix[k]].writeNow((int)((int32_t)from[k] + d * e / 1000));
    }
    if (!naptime(SEG_STEP_MS)) return false;      /* STOP was pressed */
  }
  for (int k = 0; k < n; k++) JOINTS[jix[k]].writeNow(to[k]);
  return !gAbort;
}

/*  A tiny wrapper so a phase reads as one line and the trace print
 *  costs nothing when PUSHUP_TRACE is 0.                             */
#if PUSHUP_TRACE
  #define SEGRUN(tag, jarr, aarr, ms) do {                             \
      Serial.println(F("push-up: " tag));                              \
      if (!segGlide(jarr, aarr,                                        \
                    (int)(sizeof(jarr) / sizeof(jarr[0])), ms))        \
        { Serial.println(F("push-up: stopped by STOP")); return; }      \
    } while (0)
#else
  #define SEGRUN(tag, jarr, aarr, ms) do {                             \
      if (!segGlide(jarr, aarr,                                        \
                    (int)(sizeof(jarr) / sizeof(jarr[0])), ms))        \
        return;                                                        \
    } while (0)
#endif

//********************push ups********************//
/*====================================================================
 *  PART 5b. THE PUSH-UP, REBUILT SO IT ALWAYS FINISHES
 *
 *  Same movement, same seven phases, same final angles, same ten reps.
 *  What changed:
 *
 *   1. It GLIDES INTO phase 1 from wherever the robot is standing, so
 *      there is no lurch at the very start.
 *   2. Each phase moves all of its joints together (segGlide) instead of
 *      writing them one after another, so both legs and both arms track
 *      each other smoothly the whole way down and the whole way up.
 *   3. The dead ten-second pause in the middle is now PUSHUP_HOLD_MS
 *      (1200 ms), which is what made it look like it "stopped halfway".
 *   4. Every phase and every rep prints a line, so if it ever really
 *      does stall you can see exactly where.
 *   5. Nothing in here can be aborted by the tilt guard: the tilt guard
 *      is disabled while a command is running (TILT_GUARD_MOVING 0) and
 *      the fall-recovery task refuses to run while gBusy is set. The
 *      only thing that stops this routine is you pressing STOP.
 *===================================================================*/
void push_ups() {
  /*  JOINTS[] index reminder, so these tables stay readable:
   *      head 0
   *      lh1 1  lh2 2  lh3 3      rh1  9  rh2 10  rh3 11
   *      ll1 4  ll2 5  ll3 6      rl1 12  rl2 13  rl3 14
   *      ll4 7  ll5 8             rl4 15  rl5 16                     */

  /* --- phase 0: get there gently from the standing pose ------------ */
  static const uint8_t p0j[] = {  2, 10,  9,  1, 11,  3,
                                 13,  5, 14,  6, 15,  7, 12,  4 };
  static const int16_t p0a[] = { 180,  0,  0,180, 20,160,
                                 150, 30, 30,150, 30,150, 90, 90 };
  SEGRUN("1/8 taking up the start position", p0j, p0a, 1400);

  /* --- phase 1: arms forward, hips and knees fold ------------------ */
  static const uint8_t p1j[] = {  2, 10,  9,  1, 11,  3,
                                 13,  5, 14,  6, 15,  7 };
  static const int16_t p1a[] = { 150, 30, 30,150, 50,130,
                                  90, 90,150, 30, 90, 90 };
  SEGRUN("2/8 folding down", p1j, p1a, 5400);

  /* --- phase 2: hips roll out, arms come back under the body ------- */
  static const uint8_t p2j[] = { 12, 16,  4,  8,  1,  9 };
  static const int16_t p2a[] = { 30, 30,150,150, 90, 90 };
  SEGRUN("3/8 rolling the hips out", p2j, p2a, 2900);

  /* --- phase 3: hips all the way over ----------------------------- */
  static const uint8_t p3j[] = { 12,  4 };
  static const int16_t p3a[] = {  0,180 };
  SEGRUN("4/8 hips over", p3j, p3a, 1400);

  /* --- phase 4: elbows out, legs straighten behind ----------------- */
  static const uint8_t p4j[] = {  11,  3, 13,  5 };
  static const int16_t p4a[] = { 110, 70,180,  0 };
  SEGRUN("5/8 legs out behind", p4j, p4a, 4300);

  /* --- phase 5: settle onto the toes, hips square ------------------ */
  static const uint8_t p5j[] = {  13,  5, 16,  8, 12,  4 };
  static const int16_t p5a[] = { 150, 30, 90, 90, 90, 90 };
  SEGRUN("6/8 squaring up on the toes", p5j, p5a, 4300);

  /* --- phase 6: knees and ankles into the plank -------------------- */
  static const uint8_t p6j[] = { 14,  6, 15,  7 };
  static const int16_t p6a[] = { 30,150, 30,150 };
  SEGRUN("7/8 into the plank", p6j, p6a, 2900);

  /* --- a short brace, NOT the old ten-second dead stop ------------- */
  Serial.println(F("push-up: braced -- starting the reps"));
  DLY(PUSHUP_HOLD_MS);

  /* --- the reps: the same four arm joints, two sets of angles ------ */
  static const uint8_t repj[] = { 10,  2, 11,  3 };   // rh2 lh2 rh3 lh3
  static const int16_t upa [] = { 60,120,110, 70 };   // pressed up
  static const int16_t dna [] = { 30,150, 80,100 };   // lowered down

  if (!segGlide(repj, upa, 4, 900)) return;   /* line the arms up first */

  for (int i = 1; i <= PUSHUP_REPS; i++) {
    Serial.printf("push-up: 8/8 rep %d/%d\n", i, PUSHUP_REPS);
    if (!segGlide(repj, dna, 4, 1500)) return;     /* lower  */
    if (!segGlide(repj, upa, 4, 1500)) return;     /* press  */
  }
  Serial.printf("push-up: finished all %d reps\n", PUSHUP_REPS);
}

//********************gangnam style********************//
void gangnam_style_dance() {
  gangnam_style1();        ABORTCHK();
  gangnam_style2_right();  ABORTCHK();
  gangnam_style3();        ABORTCHK();
  gangnam_style2_left();   ABORTCHK();
  gangnam_style3();
}

void gangnam_style1() {
  for (int i = 0; i <= 30; i++) {
    rh2.write(0 + i);
    lh2.write(180 - i);
    if (i <= 25) {
      rh3.write(20 + i * 2);
      lh3.write(160 - i * 2);
    }
    DLY(20);
  }
  for (int i = 1; i <= 5; i++) {
    for (int i = 0; i <= 20; i++) {
      ll2.write(30 + i);
      ll3.write(150 - (i * 2));
      ll4.write(150 - i);
      DLY(30);
    }
    for (int i = 0; i <= 20; i++) {
      ll2.write(50 - i);
      ll3.write(110 + (i * 2));
      ll4.write(130 + i);
      DLY(30);
    }
  }
  for (int i = 0; i <= 30; i++) {
    rh2.write(30 - i);
    lh2.write(150 + i);
    if (i <= 25) {
      rh3.write(70 - i * 2);
      lh3.write(110 + i * 2);
    }
    DLY(20);
  }
}

//////////////////////////////////
void gangnam_style2_right() {
  for (int i = 0; i <= 180; i++) {
    rh1.write(0 + i);
    if (i <= 80) {
      lh1.write(180 - i);
      lh3.write(160 - i);
    }
    DLY(15);
  }
  for (int i = 1; i <= 10; i++) {
    for (int i = 0; i <= 20; i++) {
      rh1.write(180 - i);
      ll5.write( 90 + i);
      rl5.write( 90 - i);
      DLY(30);
    }
    for (int i = 0; i <= 20; i++) {
      rh1.write(160 + i);
      ll5.write(  110 - i);
      rl5.write( 70 + i);
      DLY(30);
    }
  }
  for (int i = 0; i <= 180; i++) {
    rh1.write(180 - i);
    if (i <= 80) {
      lh1.write(100 + i);
      lh3.write(80 + i);
    }
    DLY(15);
  }
  DLY(500);
}

////////////////////////////////////////////
void gangnam_style2_left() {
  for (int i = 0; i <= 180; i++) {
    lh1.write(180 - i);
    if (i <= 80) {
      rh1.write(0 + i);
      rh3.write(20 + i);
    }
    DLY(15);
  }
  for (int i = 1; i <= 10; i++) {
    for (int i = 0; i <= 20; i++) {
      lh1.write(0 + i);
      ll5.write( 90 + i);
      rl5.write( 90 - i);
      DLY(30);
    }
    for (int i = 0; i <= 20; i++) {
      lh1.write(20 - i);
      ll5.write(  110 - i);
      rl5.write( 70 + i);
      DLY(30);
    }
  }
  for (int i = 0; i <= 180; i++) {
    lh1.write(0 + i);
    if (i <= 80) {
      rh1.write(80 - i);
      rh3.write(100 - i);
    }
    DLY(15);
  }
  DLY(500);
}

////////////////////////////////////
void gangnam_style3() {
  for (int i = 0; i <= 90; i++) {
    lh1.write(180 - i);
    if (i <= 70) {
      rh1.write(0 + i);
    }
    DLY(15);
  }
  for (int i = 0; i <= 70; i++) {
    rh3.write(20 + i);
    lh3.write(160 - i);
    DLY(15);
  }
  for (int i = 1; i <= 10; i++) {
    for (int i = 0; i <= 20; i++) {
      lh1.write(90 - i);
      rh1.write(70 + i);
      ll5.write( 90 + i);
      rl5.write( 90 - i);
      DLY(20);
    }
    for (int i = 0; i <= 20; i++) {
      lh1.write(70 + i);
      rh1.write(90 - i);
      ll5.write(  110 - i);
      rl5.write( 70 + i);
      DLY(20);
    }
  }
  for (int i = 0; i <= 70; i++) {
    rh3.write(90 - i);
    lh3.write(90 + i);
    DLY(15);
  }
  for (int i = 0; i <= 90; i++) {
    lh1.write(90 + i);
    if (i <= 70) {
      rh1.write(70 - i);
    }
    DLY(15);
  }
}

//********************dance steps********************//
void dance_steps() {
  do_bow();         ABORTCHK();
  dance_move1();    ABORTCHK();
  dance_move2();    ABORTCHK();
  dance_move3();    ABORTCHK();
  dance_move4();    ABORTCHK();
  dance_move5();    ABORTCHK();
  dance_move6();    ABORTCHK();
  gangnam_style3(); ABORTCHK();
  dance_move7();    ABORTCHK();
  do_bow();
}

void dance_move1() {
  for (int i = 1; i <= 10; i++) {
    for (int i = 0; i <= 20; i++) {
      rl5.write( 90 - i);
      DLY(25);
    }
    for (int i = 0; i <= 20; i++) {
      rl5.write( 70 + i);
      DLY(25);
    }
  }
}

//////////////////////////////////
void dance_move2() {
  for (int i = 1; i <= 5; i++) {
    for (int i = 0; i <= 30; i++) {
      lh2.write( 180 - i);
      lh3.write( 160 - i);
      DLY(15);
    }
    for (int i = 0; i <= 30; i++) {
      rh2.write( 0 + i);
      rh3.write( 20 + i);
      DLY(15);
    }
    for (int i = 0; i <= 30; i++) {
      lh2.write( 150 + i);
      lh3.write( 130 + i);
      DLY(15);
    }
    for (int i = 0; i <= 30; i++) {
      rh2.write( 30 - i);
      rh3.write( 50 - i);
      DLY(15);
    }
  }
}

///////////////////////////////
void dance_move3() {
  for (int k = 1; k <= 3; k++) {
    for (int i = 0; i <= 30; i++) {
      rh2.write( 0 + i);
      rh3.write( 20 + i);
      if (i % 3 == 0) {
        Serial.println(i / 3);
        rl1.write( 90 - i / 3);
        rl5.write( 90 - i / 3);
        ll1.write( 90 - i / 3);
        ll5.write( 90 - i / 3);
      }
      DLY(20);
    }
    for (int i = 0; i <= 30; i++) {
      rh2.write( 30 - i);
      rh3.write( 50 - i);
      if (i % 3 == 0) {
        Serial.println(i / 3);
        rl1.write( 80 + i / 3);
        rl5.write( 80 + i / 3);
        ll1.write( 80 + i / 3);
        ll5.write( 80 + i / 3);
      }
      DLY(20);
    }
    for (int i = 0; i <= 30; i++) {
      lh2.write( 180 - i);
      lh3.write( 160 - i);
      if (i % 3 == 0) {
        Serial.println(i / 3);
        rl1.write( 90 + i / 3);
        rl5.write( 90 + i / 3);
        ll1.write( 90 + i / 3);
        ll5.write( 90 + i / 3);
      }
      DLY(20);
    }
    for (int i = 0; i <= 30; i++) {
      lh2.write( 150 + i);
      lh3.write( 130 + i);
      if (i % 3 == 0) {
        Serial.println(i / 3);
        rl1.write( 100 - i / 3);
        rl5.write( 100 - i / 3);
        ll1.write( 100 - i / 3);
        ll5.write( 100 - i / 3);
      }
      DLY(20);
    }
  }
}

//////////////////////////////
void dance_move4() {
  for (int i = 0; i <= 180; i++) {
    rh1.write( 0 + i);
    lh1.write( 180 - i);
    DLY(15);
  }
  for (int i = 1; i <= 3; i++) {
    for (int i = 0; i <= 20; i++) {
      rh2.write( 0 + i * 2);
      lh2.write( 180 - i * 2);
      rh3.write( 20 + i * 2);
      lh3.write( 160 - i * 2);
      ll5.write( 90 + i);
      rl5.write( 90 - i);
      DLY(30);
    }
    for (int i = 0; i <= 20; i++) {
      rh2.write( 40 - i * 2);
      lh2.write( 140 + i * 2);
      rh3.write( 60 - i * 2);
      lh3.write( 120 + i * 2);
      ll5.write(  110 - i);
      rl5.write( 70 + i);
      DLY(30);
    }
  }
  for (int i = 0; i <= 180; i++) {
    rh1.write( 180 - i);
    lh1.write( 0 + i);
    DLY(15);
  }
}

/////////////////////////////////////////
void dance_move5() {
  for (int i = 0; i <= 180; i++) {
    lh1.write( 180 - i);
    rh1.write( 0 + i);
    if (i <= 90) {
      lh2.write( 180 - i);
      rh2.write( 0 + i);
    }
    if (i <= 70) {
      lh3.write( 160 - i);
      rh3.write( 20 + i);
    }
    DLY(15);
  }
  for (int i = 0; i <= 45; i++) {
    lh3.write( 90 - i);
    rh3.write( 90 - i);
    DLY(15);
  }
  for (int i = 1; i <= 3; i++) {
    for (int i = 0; i <= 90; i++) {
      lh3.write( 45 + i);
      rh3.write( 45 + i);
      DLY(15);
    }
    for (int i = 0; i <= 90; i++) {
      lh3.write( 135 - i);
      rh3.write( 135 - i);
      DLY(15);
    }
  }
  for (int i = 0; i <= 45; i++) {
    lh3.write( 45 + i);
    rh3.write( 45 + i);
    DLY(15);
  }
  //zigzag hand move
  for (int i = 0; i <= 180; i++) {
    lh1.write( 0 + i);
    DLY(15);
  }
  for (int i = 1; i <= 3; i++) {
    for (int i = 0; i <= 180; i++) {
      lh1.write( 180 - i);
      rh1.write( 180 - i);
      DLY(15);
    }
    for (int i = 0; i <= 180; i++) {
      lh1.write( 0 + i);
      rh1.write( 0 + i);
      DLY(15);
    }
  }
  for (int i = 0; i <= 180; i++) {
    lh1.write( 180 - i);
    DLY(15);
  }
  for (int i = 0; i <= 180; i++) {
    lh1.write( 0 + i);
    rh1.write( 180 - i);
    if (i <= 90) {
      lh2.write( 90 + i);
      rh2.write( 90 - i);
    }
    if (i <= 70) {
      lh3.write( 90 + i);
      rh3.write( 90 - i);
    }
    DLY(15);
  }
}

//////////////////////////////////////////////////
void dance_move6() {
  for (int i = 0; i <= 45; i++) {
    rh2.write(0 + i); //45
    lh2.write(180 - i); //135
    if (i <= 25) {
      rh3.write(20 + i);//45
      lh3.write(160 - i);//135
    }
    DLY(15);
  }
  for (int i = 0; i <= 10; i++) {
    rh3.write(45 - (i * 2));
    lh3.write(135 - (i * 2));
    rl1.write( 90 + i);
    rl5.write( 90 + i);
    ll1.write( 90 + i);
    ll5.write( 90 + i);
    head.write(90 + i * 4);
    DLY(50);
  }
  for (int j = 1; j <= 3; j++) {
    for (int i = 0; i <= 20; i++) {
      head.write(130 - i * 4);
      rh3.write(25 + (i * 2));
      lh3.write(115 + (i * 2));
      rl1.write( 100 - i);
      rl5.write( 100 - i);
      ll1.write( 100 - i);
      ll5.write( 100 - i);
      DLY(50);
    }
    for (int i = 0; i <= 20; i++) {
      head.write(50 + i * 4);
      rh3.write(65 - (i * 2));
      lh3.write(155 - (i * 2));
      rl1.write( 80 + i);
      rl5.write( 80 + i);
      ll1.write( 80 + i);
      ll5.write( 80 + i);
      DLY(50);
    }
  }
  for (int i = 0; i <= 10; i++) {
    head.write(130 - i * 4);
    rh3.write(25 + (i * 2));
    lh3.write(115 + (i * 2));
    rl1.write( 100 - i);
    rl5.write( 100 - i);
    ll1.write( 100 - i);
    ll5.write( 100 - i);
    DLY(50);
  }
  for (int i = 0; i <= 45; i++) {
    rh2.write(45 - i); //45
    lh2.write(135 + i); //135
    if (i <= 25) {
      rh3.write(45 - i);//45
      lh3.write(135 + i);//135
    }
    DLY(15);
  }
}

//////////////////////////////////////
void dance_move7() {
  for (int i = 0; i <= 20; i++) {
    rh3.write(20 - i);
    lh3.write(160 + i);
    DLY(15);
  }
  for (int i = 1; i <= 5; i++) {
    for (int i = 0; i <= 60; i++) {
      rh1.write(0 + i);//60
      lh2.write(180 - i);// 120
      rh3.write(0 + i);//80
      lh3.write(180 - i);//100
      DLY(10);
    }
    for (int i = 0; i <= 60; i++) {
      rh1.write(60 - i);
      lh2.write(120 + i);
      rh3.write(60 - i);
      lh3.write(120 + i);
      DLY(10);
    }
    for (int i = 0; i <= 60; i++) {
      lh1.write(180 - i);
      rh2.write(0 + i);
      rh3.write(0 + i);
      lh3.write(180 - i);
      DLY(10);
    }
    for (int i = 0; i <= 60; i++) {
      lh1.write(120 + i);
      rh2.write(60 - i);
      rh3.write(60 - i);
      lh3.write(120 + i);
      DLY(10);
    }
  }
  for (int i = 0; i <= 20; i++) {
    rh3.write(0 + i);
    lh3.write(180 - i);
    DLY(15);
  }
}

void push_ups_recover() {
  Serial.println(F("getting up..."));

  /* undo the last half-rep: back to the end of the entry stages */
  for (int i = 0; i <= 30; i++) {
    lh2.write(120 + i);
    rh2.write(60 - i);
    DLY(50);
  }
  /* reverse of entry stage 6 */
  for (int i = 0; i <= 60; i++) {
    rl3.write(30 + (i * 2));
    ll3.write(150 - (i * 2));
    rl4.write(30 + i);
    ll4.write(150 - i);
    DLY(50);
  }
  /* reverse of entry stage 5 */
  for (int i = 0; i <= 90; i++) {
    if (i % 3 == 0) {
      rl2.write(150 + i / 3);
      ll2.write(30 - i / 3);
    }
    if (i <= 60) {
      rl5.write(90 - i);
      ll5.write(90 + i);
    }
    rl1.write(90 - i);
    ll1.write(90 + i);
    DLY(50);
  }
  /* reverse of entry stage 4 */
  for (int i = 0; i <= 90; i++) {
    if (i <= 60) {
      rh3.write(110 - i);
      lh3.write(70 + i);
    }
    rl2.write(180 - i);
    ll2.write(0 + i);
    DLY(50);
  }
  /* reverse of entry stage 3 */
  for (int i = 0; i <= 30; i++) {
    rl1.write(0 + i);
    ll1.write(180 - i);
    DLY(50);
  }
  /* reverse of entry stage 2 -- now sitting */
  for (int i = 0; i <= 60; i++) {
    rl1.write(30 + i);
    rl5.write(30 + i);
    ll1.write(150 - i);
    ll5.write(150 - i);
    if (i % 2 == 0) {
      lh1.write(90 + i);
      rh1.write(90 - i);
    }
    DLY(50);
  }
  ABORTCHK();
  /* and your own stand_up() is the reverse of entry stage 1 */
  stand_up();
}

/*====================================================================
 *  13. YOUR ORIGINAL WALK AND TURNS -- KEPT FOR COMPARISON
 *
 *  These are your move_forward(), turn_left() and turn_right(), copied
 *  verbatim and only renamed. Commands: `old walk`, `old turn left`,
 *  `old turn right`.
 *
 *  I replayed old_walk() frame by frame off your file before touching
 *  anything, and here is exactly what it does over one cycle:
 *
 *    ll3 / rl3  (both knees)  150 / 30  ->  150 / 30
 *        The knees never move. Not by one degree, the entire walk. So
 *        the swinging foot has zero ground clearance and drags. That is
 *        the "leg doesn't come completely up" you described.
 *
 *    rl2  150 -> 160   (+10 left over at the end of every cycle)
 *    rl4   30 ->  40   (+10 left over at the end of every cycle)
 *        The return loops for these two count i = 0..30 while the
 *        outward loops count i = 0..20. Ten degrees are unaccounted for
 *        each cycle, so the right leg creeps, and then the next cycle's
 *        opening write snaps it back 10 degrees in one frame. Creep,
 *        snap, creep, snap.
 *
 *    lh2  180 -> 160   (arm creeps down the same way)
 *
 *    ll1 / ll5 / rl1 / rl5   90 -> 95
 *        Only five degrees of sideways weight shift, and it is left
 *        over at the end too. Five degrees is not enough to unload a
 *        foot, so the robot tries to swing a leg that is still carrying
 *        weight. It cannot, so it stalls -- "it gets stuck".
 *
 *  Left is worse than right, as you said, but not because the left side
 *  is worse: it is because the RIGHT leg is the one creeping, and the
 *  left leg then has to swing a body that is no longer over its foot.
 *
 *  The new gait in section 11 has none of this. It is built from five
 *  numbers that all return to zero, so drift is not merely fixed, it is
 *  structurally impossible.
 *===================================================================*/

//********************move forward********************//
void old_walk() {
  for (int i = 0; i <= 5; i++) {
    lh2.write(180 - (i * 2));
    rl1.write( 90 + i);
    rl5.write( 90 + i);
    ll1.write( 90 + i);
    ll5.write( 90 + i);
    DLY(50);
  }
  for (int i = 0; i <= WALK_STEPS; i++) {
    for (int i = 0; i <= 20; i++) {
      rh1.write(0 + i);
      ll2.write(30 + i);
      ll4.write(150 + i);
      DLY(50);
    }
    for (int i = 0; i <= 20; i++) {
      if (i <= 10) {
        lh2.write(160 + i * 2);
        rh2.write(0 + i * 2);
      }
      rh1.write(20 - i);
      ll2.write(50 - i);
      ll4.write(170 - i);
      if (i <= 10) {
        rl1.write( 95 - i);
        rl5.write( 95 - i);
        ll1.write( 95 - i);
        ll5.write( 95 - i);
      }
      DLY(50);
    }
    for (int i = 0; i <= 20; i++) {
      lh1.write(180 - i);
      rl2.write(150 - i);
      rl4.write(30 - i);
      DLY(50);
    }
    for (int i = 0; i <= 30; i++) {
      if (i <= 10) {
        lh2.write(180 - i * 2);
        rh2.write(20 - i * 2);
      }
      lh1.write(160 + i);
      rl2.write(130 + i);
      rl4.write(10 + i);
      if (i <= 10) {
        rl1.write( 85 + i);
        rl5.write( 85 + i);
        ll1.write( 85 + i);
        ll5.write( 85 + i);
      }
      DLY(50);
    }
  }
  for (int i = 0; i <= 5; i++) {
    lh2.write(170 + (i * 2));
    rl1.write( 95 - i);
    rl5.write( 95 - i);
    ll1.write( 95 - i);
    ll5.write( 95 - i);
    DLY(50);
  }
}

//********************turn left********************//
void old_turn_left() {
  for (int i = 1; i <= TURN_STEPS; i++) {
    for (int i = 0; i <= 5; i++) {
      rh2.write(0 + (i * 2));
      rl1.write( 90 - i);
      rl5.write( 90 - i);
      ll1.write( 90 - i);
      ll5.write( 90 - i);
      DLY(30);
    }
    for (int i = 0; i <= 10; i++) {
      lh1.write(180 - i);
      if (i % 2 == 0) {
        ll2.write(30 + i / 2);
      }
      rl2.write(150 - i);
      rl4.write(30 - i);
      DLY(30);
    }
    for (int i = 0; i <= 30; i++) {
      lh1.write(150 + i);
      if (i % 2 == 0) {
        ll2.write(45 - i / 2);
      }
      rl2.write(120 + i);
      rl4.write(0 + i);
      DLY(30);
    }
    for (int i = 0; i <= 5; i++) {
      rh2.write(10 - i * 2);
      rl1.write( 85 + i);
      rl5.write( 85 + i);
      ll1.write( 85 + i);
      ll5.write( 85 + i);
      DLY(30);
    }
  }
}

//********************turn right********************//
void old_turn_right() {
  for (int i = 1; i <= TURN_STEPS; i++) {
    for (int i = 0; i <= 5; i++) {
      lh2.write(180 - (i * 2));
      rl1.write( 90 + i);
      rl5.write( 90 + i);
      ll1.write( 90 + i);
      ll5.write( 90 + i);
      DLY(30);
    }
    for (int i = 0; i <= 10; i++) {
      rh1.write(0 + i);
      if (i % 2 == 0) {
        rl2.write(150 - i / 2);
      }
      ll2.write(30 + i);
      ll4.write(150 + i);
      DLY(30);
    }
    for (int i = 0; i <= 30; i++) {
      rh1.write(30 - i);
      if (i % 2 == 0) {
        rl2.write(135 + i / 2);
      }
      ll2.write(60 - i);
      ll4.write(180 - i);
      DLY(30);
    }
    for (int i = 0; i <= 5; i++) {
      lh2.write(170 + i * 2);
      rl1.write( 95 - i);
      rl5.write( 95 - i);
      ll1.write( 95 - i);
      ll5.write( 95 - i);
      DLY(30);
    }
  }
}

/*====================================================================
 *  14. GOING TO A POSE
 *
 *  If you already designed a transition between two poses, use it --
 *  sit_down() and stand_up() know which order to fold the joints in,
 *  and a generic simultaneous glide does not. The glide is only the
 *  fallback for pose pairs you never wrote a transition for.
 *===================================================================*/
bool goToPose(const int16_t* target) {
  if (poseDistance(target) <= POSE_TOL) return true;

  if (target == POSE_STAND && poseDistance(POSE_SIT) <= POSE_TOL) {
    Serial.println(F("standing up (your stand_up sequence, slowed)"));
    faceSet(F_BUSY, "STANDING UP");
    uint16_t s = gTimePct; gTimePct = SPEED_SIT;
    stand_up();
    gTimePct = s;
    if (gAbort) return false;
    if (poseDistance(POSE_STAND) <= POSE_TOL) return true;
  }
  if (target == POSE_SIT && poseDistance(POSE_STAND) <= POSE_TOL) {
    Serial.println(F("sitting down (your sit_down sequence, slowed)"));
    faceSet(F_SIT, "SITTING DOWN");
    uint16_t s = gTimePct; gTimePct = SPEED_SIT;
    sit_down();
    gTimePct = s;
    if (gAbort) return false;
    if (poseDistance(POSE_SIT) <= POSE_TOL) return true;
  }
  return glideTo(target, -1);
}

/*====================================================================
 *  15. THE AUTONOMOUS SEQUENCE
 *
 *  You asked for the robot to sit down, get up and walk on its own
 *  with nobody holding it. That is this one command. It settles fully
 *  between stages, because the thing that makes an unsupported robot
 *  fall is not any single move -- it is starting the next move while
 *  the last one is still swinging.
 *
 *  Run it the first time with the robot hanging or with a hand loosely
 *  behind it. Not holding it up: just there to catch it.
 *===================================================================*/
void auto_demo() {
  uint16_t s = gTimePct;

  Serial.println(F("--- autonomous sequence: stand, sit, get up, walk ---"));

  faceSet(F_IDLE, "DEMO");
  if (!goToPose(POSE_STAND)) return;
  if (!naptime(SETTLE_MS))   return;

  Serial.println(F("[1/4] sitting down"));
  faceSet(F_SIT, "SITTING");
  gTimePct = SPEED_SIT;  sit_down();  gTimePct = s;
  if (gAbort) return;
  if (!naptime(1800)) return;

  Serial.println(F("[2/4] getting up"));
  faceSet(F_BUSY, "GETTING UP");
  gTimePct = SPEED_SIT;  stand_up();  gTimePct = s;
  if (gAbort) return;
  if (!naptime(SETTLE_MS)) return;

  Serial.println(F("[3/4] settling before the walk"));
  if (!goToPose(POSE_STAND)) return;
  if (!naptime(SETTLE_MS))   return;

  Serial.println(F("[4/4] walking"));
  walk_forward();
  if (gAbort) return;

  faceSet(F_HAPPY, "DONE");
  Serial.println(F("--- sequence complete ---"));
}

/*====================================================================
 *  16. SETUP AND CALIBRATION TOOLS
 *===================================================================*/
int findJoint(const String &n) {
  for (int i = 0; i < NJ; i++) if (n.equalsIgnoreCase(JOINTS[i].name)) return i;
  return -1;
}

void listJoints() {
  Serial.println(F("\nname   ch   inv  trim  bias   asked  ->  sent to servo"));
  Serial.println(F("------------------------------------------------------"));
  for (int i = 0; i < NJ; i++) {
    Joint &j = JOINTS[i];
    Serial.printf("%-5s %3d   %-3s  %4d  %4d   %5d  ->  %3d\n",
                  j.name, j.ch, j.inv ? "YES" : "no",
                  j.trim, j.bias, j.last, j.physical(j.last));
  }
  Serial.println(F("ch -1 = head, driven from GPIO 25"));
  Serial.println(F("bias = live balance trim, not saved, changes on its own"));
  Serial.printf("position: %s\n\n", whereAmI());
}

void printPose() {
  Serial.print(F("pose:"));
  for (int i = 0; i < NJ; i++) Serial.printf(" %d", JOINTS[i].last);
  Serial.printf("\n      (%s)\n", whereAmI());
}

/*  Move one joint slowly to a logical angle. */
void jogJoint(int idx, int target) {
  Joint &j = JOINTS[idx];
  int from = j.last;
  target = constrain(target, 0, 180);
  int d = (target >= from) ? 1 : -1;
  for (int a = from; a != target; a += d) { j.writeNow(a); delay(12); }
  j.writeNow(target);
  Serial.printf("%s: asked %d -> servo sees %d\n", j.name, target, j.physical(target));
}

/*  Sweep one joint alone so you can watch which way it actually turns. */
void sweepJoint(int idx, int a, int b) {
  Joint &j = JOINTS[idx];
  Serial.printf("sweeping %s  %d -> %d   (servo %d -> %d)\n",
                j.name, a, b, j.physical(a), j.physical(b));
  jogJoint(idx, a);
  delay(600);
  int d = (b >= a) ? 1 : -1;
  for (int v = a; v != b; v += d) { j.writeNow(v); delay(35); }
  j.writeNow(b);
  Serial.println(F("done"));
}

void relaxAll() {
  for (int i = 0; i < NJ; i++) if (JOINTS[i].ch >= 0) pwm.setPWM(JOINTS[i].ch, 0, 0);
  headOff();
}

/*  THE JITTER METER.
 *
 *  Stand the robot up, leave it completely alone, then type  quiet.
 *  It counts how many pulse changes actually reach the servos over
 *  three seconds. Standing still, that number should be ZERO. Not
 *  small -- zero. Every write in that window is the firmware telling a
 *  servo to move while you asked it to stand still, and every one of
 *  them is audible.
 *
 *  If it says zero and you can still hear buzzing, the noise is not
 *  coming from the firmware. It is the servos hunting because the
 *  supply cannot hold them, and only a bigger supply fixes that.     */
void quietReport() {
  Serial.printf("measuring for %d ms -- do not touch anything\n", QUIET_REPORT_MS);
  uint32_t w0 = gWrites, s0 = gSkipped;
  uint32_t t0 = millis();
  while (millis() - t0 < QUIET_REPORT_MS) { pumpIO(); delay(2); }
  uint32_t w = gWrites - w0, s = gSkipped - s0;
  Serial.printf("pulse changes sent to servos : %lu\n", (unsigned long)w);
  Serial.printf("duplicate writes suppressed  : %lu\n", (unsigned long)s);
  if (w == 0) {
    Serial.println(F("PERFECT -- the firmware is completely silent while idle."));
    Serial.println(F("Any buzzing you can still hear is the power supply, not"));
    Serial.println(F("the code. See the note at the top of this sketch."));
  } else if (w <= 8) {
    Serial.println(F("nearly silent -- that will be the balance trim nudging."));
    Serial.println(F("Type  balance off  and measure again to confirm."));
  } else {
    Serial.println(F("TOO MANY. Something is still driving the servos while"));
    Serial.println(F("idle. Check DEDUPE_WRITES is 1 and balance is off."));
  }
}

#define CFG_MAGIC 0xA5A50002UL

void saveConfig() {
  uint32_t bits = 0;
  int8_t trims[NJ];
  for (int i = 0; i < NJ; i++) {
    if (JOINTS[i].inv) bits |= (1UL << i);
    trims[i] = JOINTS[i].trim;
  }
  prefs.putUInt("inv", bits);
  prefs.putBytes("trim", trims, sizeof(trims));
  prefs.putUInt("magic", CFG_MAGIC);
  Serial.println(F("config saved to flash"));
}

void loadConfig() {
  /*  Magic word rather than isKey() so this also builds on older cores,
   *  and so a half-written config can never wipe the defaults below.  */
  if (prefs.getUInt("magic", 0) != CFG_MAGIC) {
    Serial.println(F("no saved config -- using the table in this sketch"));
    return;
  }
  uint32_t bits = prefs.getUInt("inv", 0);
  int8_t trims[NJ];
  if (prefs.getBytes("trim", trims, sizeof(trims)) == sizeof(trims))
    for (int i = 0; i < NJ; i++) JOINTS[i].trim = trims[i];
  for (int i = 0; i < NJ; i++) JOINTS[i].inv = (bits >> i) & 1UL;
  gPitchZero = prefs.getFloat("pz", 0.0f);
  gRollZero  = prefs.getFloat("rz", 0.0f);
  Serial.println(F("config loaded from flash"));
}

void printHelp() {
  Serial.println(F("\n================ COMMANDS ================"));
  Serial.println(F("Every move first glides to its correct starting"));
  Serial.println(F("pose, waits 2 s, then runs. Nothing snaps."));
  Serial.println(F("\nAUTONOMOUS"));
  Serial.println(F("  demo              stand, sit, get up, walk. Hands off."));
  Serial.println(F("\nMOVES"));
  Serial.println(F("  stand | stop | freeze"));
  Serial.println(F("  say hi            shake hand        hands up"));
  Serial.println(F("  hands down        right bicep       left bicep"));
  Serial.println(F("  double biceps     maan karate       bow"));
  Serial.println(F("  ape move          exercise one      side bend"));
  Serial.println(F("  sit down          stand up          dab"));
  Serial.println(F("  walk              turn left         turn right"));
  Serial.println(F("  push ups          gangnam style     dance"));
  Serial.println(F("  introduce         wave, hand on the chest, arms open,"));
  Serial.println(F("                    a bow, and a voice on every phase"));
  Serial.println(F("  old walk | old turn left | old turn right"));
  Serial.println(F("                    your original versions, for comparison"));
  Serial.println(F("\nSTEADINESS"));
  Serial.println(F("  quiet             count servo writes while standing"));
  Serial.println(F("                    still. Should be ZERO. Read this one."));
  Serial.println(F("  steps <n>         half-steps per  walk  (start with 2)"));
  Serial.println(F("  stance <deg>      widen the feet, 0..20, more = steadier"));
  Serial.println(F("  balance on|off    MPU6050 standing trim"));
  Serial.println(F("  bal zero          learn 'upright' -- hold it level first"));
  Serial.println(F("  baltest roll      check the correction sign, roll axis"));
  Serial.println(F("  baltest pitch     same for pitch"));
  Serial.println(F("  tilt              live pitch / roll"));
  Serial.println(F("\nSPEED"));
  Serial.println(F("  speed <pct>       100 = original speed, higher = slower"));
  Serial.println(F("  speed auto        slow for sit/stand, mid for push-ups"));
  Serial.println(F("\nSETUP / FIXING A WRONG-DIRECTION SERVO"));
  Serial.println(F("  list              every joint + which way it is wired"));
  Serial.println(F("  where             am I standing or sitting?"));
  Serial.println(F("  pose              print all 17 angles"));
  Serial.println(F("  flip <name>       reverse one servo   e.g.  flip ll2"));
  Serial.println(F("  trim <name> <d>   small offset        e.g.  trim ll2 -5"));
  Serial.println(F("  jog <name> <a>    move one joint      e.g.  jog ll2 90"));
  Serial.println(F("  sweep <name> <a> <b>  watch one joint travel"));
  Serial.println(F("  save | load | defaults    flip/trim in flash"));
  Serial.println(F("  limiter on|off    the anti-slam rate limiter"));
  Serial.println(F("\nFACE, VOICE AND HEAD  (all three move together)"));
  Serial.println(F("  feelings          list every mood, sound and head move"));
  Serial.println(F("  <mood>            just say it:  happy  angry  sad  cool"));
  Serial.println(F("                    wink  dizzy  love  sleepy  proud  bye"));
  Serial.println(F("                    curious  determined  music  surprised"));
  Serial.println(F("  feel <mood>       the same thing, spelled out"));
  Serial.println(F("  sound <name>      play one sound on its own"));
  Serial.println(F("  head <name>       nod | shake | tilt | scan | beat ..."));
  Serial.println(F("  head centre       put the head straight again"));
  Serial.println(F("  vol <0..200>      speaker volume, 100 = full scale,"));
  Serial.println(F("                    above 100 is soft-limited extra gain"));
  Serial.println(F("  mute | unmute"));
  Serial.println(F("  wifi              show the web address"));
  Serial.println(F("  relax             cut all pulses (ROBOT COLLAPSES)"));
  Serial.println(F("  help              this list"));
  Serial.println(F("=========================================="));
  Serial.println(F("Any command typed during a move interrupts it.\n"));
}

/*====================================================================
 *  16b. "INTRODUCE YOURSELF"   (new)
 *
 *  A single social routine: wave, hand to the chest, both arms opened
 *  out, then a small bow -- with a different voice on each phase and a
 *  head movement to match, so it reads as a person introducing himself
 *  rather than a servo sequence.
 *
 *  It is built entirely out of parts that already existed:
 *    segGlide()    moves a whole phase's joints together on an ease
 *                  curve, so nothing snaps and the rate limiter never
 *                  has to fight it,
 *    soundPlay()   one sound, once, at the start of its phase -- never
 *                  in a loop,
 *    faceSet()     the OLED expression,
 *    headGesture() the head, running underneath the arms.
 *
 *  Only the two arms and the head move. No leg joint is touched, so the
 *  robot cannot be pushed off balance by it, and STOP still works
 *  instantly because every glide goes through naptime().
 *
 *  Arm index reminder (JOINTS[]):  left 1,2,3   right 9,10,11
 *  and the two arms mirror each other:  left = 180 - right.
 *===================================================================*/
void introduce_yourself() {
  /* ---- 1. raise the right hand -------------------------------- */
  faceSet(F_HAPPY, "HELLO");
  soundPlay(S_HI);
  headGesture(HG_PERK);
  static const uint8_t  upj[] = {   9, 10, 11 };      /* rh1 rh2 rh3 */
  static const int16_t  upa[] = { 155, 45, 60 };
  if (!segGlide(upj, upa, 3, 900)) return;

  /* ---- 2. wave, three times ----------------------------------- */
  static const uint8_t  wvj[] = { 11 };               /* rh3 only    */
  static const int16_t  wvo[] = {  95 };
  static const int16_t  wvi[] = {  25 };
  for (int i = 0; i < 3; i++) {
    if (!segGlide(wvj, wvo, 1, 300)) return;
    if (!segGlide(wvj, wvi, 1, 300)) return;
  }

  /* ---- 3. hand on the chest, "I am your robot" ---------------- */
  faceSet(F_PROUD, "I AM");
  soundPlay(S_HAPPY);
  headGesture(HG_NOD);
  static const uint8_t  chj[] = {  9, 10, 11 };
  static const int16_t  cha[] = { 55, 85, 85 };
  if (!segGlide(chj, cha, 3, 900)) return;
  DLY(500);

  /* ---- 4. both arms opened out, the "here I am" pose ---------- */
  faceSet(F_COOL, "ROBOT");
  soundPlay(S_WIN);
  headGesture(HG_SCAN);
  static const uint8_t  opj[] = {  9, 10, 11,   1,   2,   3 };
  static const int16_t  opa[] = { 95, 35, 45,  85, 145, 135 };
  if (!segGlide(opj, opa, 6, 1100)) return;
  DLY(700);

  /* ---- 5. a small bow to finish ------------------------------- */
  faceSet(F_HAPPY, "NICE 2 C U");
  soundPlay(S_BOW);
  headGesture(HG_BOW_H);
  static const uint8_t  bwj[] = {  9, 10, 11,   1,   2,   3 };
  static const int16_t  bwa[] = { 25, 60, 70, 155, 120, 110 };
  if (!segGlide(bwj, bwa, 6, 900)) return;
  DLY(600);

  /*  The table's post action glides everything back to standing, so
   *  there is nothing to undo here.                                */
  soundPlay(S_READY);
}

/*====================================================================
 *  17. WHAT POSE DOES EACH ROUTINE NEED TO START FROM?
 *
 *  This table is the actual fix for "some motors have a problem in
 *  standing and sitting". Every routine's first frame was measured;
 *  `pre` is the pose that first frame belongs to. The robot glides
 *  there before the routine starts, so the first write is always a
 *  zero-degree change instead of a slam.
 *
 *  post: what to do when the routine finishes.
 *    POST_STAND   glide back to standing
 *    POST_STAY    leave it where it ended (sitting, hands up)
 *    POST_RECOVER run push_ups_recover()
 *===================================================================*/
enum PostAct { POST_STAND, POST_STAY, POST_RECOVER };

struct CmdEntry {
  const char*    words;
  void          (*fn)();
  const int16_t* pre;
  uint16_t       speed;
  PostAct        post;
  Face           face;    /*  what the OLED shows            */
  Snd            sound;   /*  what the speaker says          */
  HeadGest       head;    /*  how the head moves, meanwhile  */
};

/*  ---- THE ACTION -> VOICE MAP ----------------------------------------
 *  Every row fires its face, its voice and its head movement together,
 *  through express(), at the moment the command starts -- once, never in
 *  a loop. soundPlay() only sets a byte; the audio task on core 0 does
 *  the work, so a routine is never slowed down by its own voice.
 *
 *      greeting      say hi, shake hand, introduce   S_HI / S_HAPPY
 *      pride         biceps, double biceps           S_FLEX
 *      effort        exercise, side bend, push ups   S_GRUNT
 *      fighting      maan karate                     S_KARATE
 *      manners       bow                             S_BOW
 *      posture       sit down / stand up             S_SIT / S_STAND
 *      travel        walk, turns, old walk/turns     S_WALK
 *      music         dance, gangnam, ape move        S_MUSIC / S_GIGGLE
 *      surprise      hands up                        S_SURPRISE
 *      swagger       dab                             S_COOL
 *      finishing     introduce yourself, at the end  S_WIN + S_READY
 *
 *  To change what an action says, change the sound column here -- that is
 *  the only place any of it is decided.                                */
const CmdEntry CMDS[] = {
/*  words             routine              start pose    speed          afterwards    face          sound       head       */
  { "demo",           auto_demo,           POSE_STAND,   SPEED_DEFAULT, POST_STAND,   F_BUSY,       S_READY,    HG_SCAN    },
  { "say hi",         say_hi,              POSE_STAND,   SPEED_DEFAULT, POST_STAND,   F_HAPPY,      S_HI,       HG_NOD     },
  { "shake hand",     shake_hand,          POSE_STAND,   SPEED_DEFAULT, POST_STAND,   F_HAPPY,      S_HAPPY,    HG_NOD     },
  { "hands up",       hands_up,            POSE_STAND,   SPEED_DEFAULT, POST_STAY,    F_SURPRISE,   S_SURPRISE, HG_PERK    },
  { "hands down",     hands_down,          POSE_HANDSUP, SPEED_DEFAULT, POST_STAND,   F_BUSY,       S_BEEP,     HG_BOW_H   },
  { "right bicep",    right_bicep,         POSE_STAND,   SPEED_DEFAULT, POST_STAND,   F_PROUD,      S_FLEX,     HG_LOOK_R  },
  { "left bicep",     left_bicep,          POSE_STAND,   SPEED_DEFAULT, POST_STAND,   F_PROUD,      S_FLEX,     HG_LOOK_L  },
  { "double biceps",  double_biceps,       POSE_STAND,   SPEED_DEFAULT, POST_STAND,   F_PROUD,      S_FLEX,     HG_PERK    },
  { "maan karate",    maan_karate,         POSE_STAND,   SPEED_DEFAULT, POST_STAND,   F_DETERMINED, S_KARATE,   HG_SHAKE   },
  { "bow",            do_bow,              POSE_STAND,   SPEED_DEFAULT, POST_STAND,   F_HAPPY,      S_BOW,      HG_BOW_H   },
  { "ape move",       ape_move,            POSE_STAND,   SPEED_DEFAULT, POST_STAND,   F_MUSIC,      S_GIGGLE,   HG_BEAT    },
  { "exercise one",   exercise_one,        POSE_STAND,   SPEED_DEFAULT, POST_STAND,   F_DETERMINED, S_GRUNT,    HG_NOD     },
  { "side bend",      side_bend,           POSE_STAND,   SPEED_DEFAULT, POST_STAND,   F_DETERMINED, S_GRUNT,    HG_SCAN    },
  { "dab",            do_dab,              POSE_STAND,   SPEED_DEFAULT, POST_STAND,   F_COOL,       S_COOL,     HG_LOOK_L  },
  { "sit down",       sit_down,            POSE_STAND,   SPEED_SIT,     POST_STAY,    F_SIT,        S_SIT,      HG_DROOP   },
  { "stand up",       stand_up,            POSE_SIT,     SPEED_SIT,     POST_STAND,   F_BUSY,       S_STAND,    HG_PERK    },
  { "move forward",   walk_forward,        POSE_STAND,   SPEED_WALK,    POST_STAND,   F_WALK,       S_WALK,     HG_SCAN    },
  { "turn left",      turn_left_new,       POSE_STAND,   SPEED_WALK,    POST_STAND,   F_WALK,       S_WALK,     HG_LOOK_L  },
  { "turn right",     turn_right_new,      POSE_STAND,   SPEED_WALK,    POST_STAND,   F_WALK,       S_WALK,     HG_LOOK_R  },
  { "old walk",       old_walk,            POSE_STAND,   SPEED_WALK,    POST_STAND,   F_WALK,       S_WALK,     HG_SCAN    },
  { "old turn left",  old_turn_left,       POSE_STAND,   SPEED_WALK,    POST_STAND,   F_WALK,       S_WALK,     HG_LOOK_L  },
  { "old turn right", old_turn_right,      POSE_STAND,   SPEED_WALK,    POST_STAND,   F_WALK,       S_WALK,     HG_LOOK_R  },
  { "push ups",       push_ups,            POSE_STAND,   SPEED_PUSHUP,  POST_RECOVER, F_DETERMINED, S_GRUNT,    HG_NOD     },
  { "gangnam style",  gangnam_style_dance, POSE_STAND,   SPEED_DEFAULT, POST_STAND,   F_MUSIC,      S_MUSIC,    HG_BEAT    },
  { "dance",          dance_steps,         POSE_STAND,   SPEED_DEFAULT, POST_STAND,   F_MUSIC,      S_MUSIC,    HG_BEAT    },
  /*  NEW: the social routine. Added at the END on purpose -- every
   *  existing web button, Blynk pin and voice name keeps the index it
   *  already had, so nothing that used to work moves.                */
  { "introduce",      introduce_yourself,  POSE_STAND,   SPEED_DEFAULT, POST_STAND,   F_HAPPY,      S_HI,       HG_NOD     }
};
#define NCMD ((int)(sizeof(CMDS) / sizeof(CMDS[0])))

/*  Position is genuinely unknown at power-up: the sketch knows what
 *  angles it last *sent*, but not where you left the robot. So the
 *  first move eases into standing under the rate limiter rather than
 *  assuming anything.                                                */
bool   gPosUnknown = true;
bool   gFreeze     = false;
String gRunning    = "";

const char* whereAmIsafe() {
  if (gPosUnknown) return "unknown (not moved since power-up)";
  return whereAmI();
}

/*  Accept the spellings the Mega sketch accepted, plus a few obvious
 *  ones people actually type.                                        */
String normalise(String v) {
  v.trim();
  v.toLowerCase();
  while (v.indexOf("  ") >= 0) v.replace("  ", " ");
  if (v == "exercise 1")                              v = "exercise one";
  if (v == "exercise 2" || v == "exercise two" || v == "exercise to") v = "side bend";
  if (v == "gangnam style dance" || v == "gangnam")   v = "gangnam style";
  if (v == "hi" || v == "wave" || v == "hello")       v = "say hi";
  if (v == "pushups" || v == "pushup" || v == "push up") v = "push ups";
  if (v == "sit")                                     v = "sit down";
  if (v == "get up" || v == "standup")                v = "stand up";
  if (v == "initial" || v == "initial position" || v == "home" || v == "straight")
                                                      v = "stand";
  if (v == "forward" || v == "walk" || v == "walk forward") v = "move forward";
  if (v == "left")                                    v = "turn left";
  if (v == "right")                                   v = "turn right";
  if (v == "auto" || v == "autonomous" || v == "sequence") v = "demo";
  if (v == "walk old" || v == "old move forward")     v = "old walk";
  /*  The EXPERIMENTAL WALK button on the web page. It runs old_walk(),
   *  which is your move_forward() copied out of your file line for line
   *  -- same channels, same angles, same DLY(50). Kept as its own name
   *  so the button stays separate from the normal controls.            */
  if (v == "experimental walk" || v == "experimental") v = "old walk";
  if (v == "bal zero" || v == "balance zero" || v == "zero") v = "balzero";
  /*  the new social routine, under every name anyone would try */
  if (v == "introduce yourself" || v == "introduce your self" ||
      v == "introduction"       || v == "intro" ||
      v == "who are you"        || v == "introduce yourself please")
                                                      v = "introduce";
  return v;
}

void queueCommand(const String &raw) {
  String t = normalise(raw);
  if (t.length() == 0) return;
  if (t == "stop") {
    gCmd = "";
    gFreeze = false;
    gAbort = true;
    Serial.println(F("STOP"));
    return;
  }
  if (t == "freeze") {
    gCmd = "";
    gFreeze = true;
    gAbort = true;
    Serial.println(F("FREEZE -- holding this position"));
    return;
  }
  /*  ---- PART 5c: a repeat of the SAME command no longer kills it -----
   *  A new command is still allowed to interrupt whatever is running --
   *  that behaviour is deliberate and unchanged. But asking for the
   *  command that is ALREADY running used to abort it, and that is the
   *  second way a push-up could die halfway through: a double-tap on the
   *  phone, a fat finger on a touchscreen, a browser retrying a fetch it
   *  thought had timed out, or the page's own repeat of the last button.
   *  Any of those arrives as a second "push ups" while the first one is
   *  still going, and the old line below turned it into an abort.
   *  Now a duplicate is simply ignored and the movement carries on.   */
  if (gBusy && gRunning.length() && t == gRunning) {
    Serial.printf("already doing \"%s\" -- carrying on (press STOP to cancel)\n",
                  t.c_str());
    return;
  }
  gCmd = t;
  if (gBusy) gAbort = true;      // a DIFFERENT command interrupts the move
}

/*====================================================================
 *  18. WEB PAGE  --  the ESP32 hosts this itself, no cloud, no account
 *===================================================================*/
#if USE_WEB
/*  The dashboard. Written as ordinary concatenated string literals
 *  rather than a raw string: some .ino preprocessors (ArduinoDroid
 *  among them) do not understand raw string literals and then try to
 *  compile the JavaScript as C++ -- that is where the
 *  "'function' does not name a type" errors came from. Every //
 *  and comment marker is split across two literals for the same
 *  reason. The text the browser receives is unchanged.          */
const char PAGE[] PROGMEM =
  "<!DOCTYPE html><html><head>\n"
  "<meta name=viewport content=\"width=device-width,initial-scale=1\">\n"
  "<title>Humanoid</title><style>\n"
  ":root{color-scheme:dark}*{box-sizing:border-box}\n"
  "body{margin:0 auto;max-width:560px;padding:14px;background:#0f1216;color:#e8eaed;\n"
  "font:16px/1.4 system-ui,-apple-system,sans-serif}\n"
  "h1{font-size:19px;margin:4px 0 2px}\n"
  "#st{font-size:13px;color:#9aa3ad;min-height:34px}\n"
  "h2{font-size:11px;letter-spacing:.09em;text-transform:uppercase;color:#7d868f;margin:16px 0 7px}\n"
  ".g{display:grid;grid-template-columns:repeat(auto-fit,minmax(118px,1fr));gap:8px}\n"
  "button{font:600 14px system-ui;padding:14px 8px;border:0;border-radius:11px;\n"
  "background:#1e252d;color:#e8eaed;cursor:pointer}\n"
  "button:active{background:#333e4a}\n"
  ".p button{background:#17352a}\n"
  ".d button{background:#2a2413}\n"
  ".e button{background:#2e1f38}\n"
  ".e button:active{background:#453055}\n"
  "#mh{display:none;background:#241a10;border:1px solid #5a4321;border-radius:11px;\n"
  "padding:11px 12px;margin:8px 0;font-size:13.5px;line-height:1.5;color:#e6d5bd}\n"
  "#mh b{color:#ffd79a}#mh ol{margin:6px 0 8px;padding-left:20px}\n"
  "#mh code{background:#12171d;padding:1px 5px;border-radius:4px;\n"
  "font:12px monospace;color:#9fd7ff;word-break:break-all}\n"
  "#mh input{width:100%;margin:5px 0;background:#12171d;color:#9fd7ff;border:1px\n"
  "solid #2b333c;border-radius:7px;padding:8px;font:12px monospace}\n"
  "#mh button{padding:9px;font-size:13px;background:#3a2a12}\n"
  ".hb{display:flex;gap:7px;margin-top:5px}.hb button{flex:1}\n"
  "#vol{width:100%}\n"
  "#stop{width:100%;padding:17px;background:#7c1d1d;font-size:17px;margin:12px 0 4px}\n"
  "#auto{width:100%;padding:16px;background:#1d4d7c;font-size:16px;margin:4px 0 8px}\n"
  "#mic{width:100%;padding:18px;background:#1f5c3a;font-size:17px;margin:4px 0 4px}\n"
  "#xw{width:100%;padding:15px;background:#4a3a10;border:1px solid #6d5518;\n"
  "font-size:15px;margin:8px 0 2px}\n"
  "#xw:active{background:#6d5518}#mic.on{background:#7c1d1d}\n"
  "#vs{font-size:13px;color:#9aa3ad;min-height:20px;margin-bottom:4px}\n"
  "input[type=range]{width:100%;margin-top:4px}\n"
  ".r{display:flex;gap:8px;align-items:center;margin-top:6px}\n"
  ".r button{flex:1}\n"
  ".jr{display:flex;align-items:center;gap:12px;margin:11px 0;\n"
  "padding:9px 11px;background:#151a20;border:1px solid #232a32;\n"
  "border-radius:13px}\n"
  ".jn{width:52px;font-size:13.5px;color:#9aa3ad;font-weight:600}\n"
  ".jv{width:40px;text-align:right;font-size:14.5px;color:#cfd5db;\n"
  "font-variant-numeric:tabular-nums}\n"
  ".jr input[type=range]{flex:1;margin:0;height:42px;touch-action:none;\n"
  "-webkit-appearance:none;appearance:none;background:transparent}\n"
  ".jr input[type=range]::-webkit-slider-runnable-track{height:9px;\n"
  "background:#2b3239;border-radius:5px}\n"
  ".jr input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;\n"
  "width:32px;height:32px;margin-top:-12px;border-radius:50%;\n"
  "background:#4a9bd8;border:2px solid #0e1216}\n"
  ".jr input[type=range]::-moz-range-track{height:9px;background:#2b3239;\n"
  "border-radius:5px}\n"
  ".jr input[type=range]::-moz-range-thumb{width:30px;height:30px;\n"
  "border:2px solid #0e1216;border-radius:50%;background:#4a9bd8}\n"
  "/" "* the flex glove card *" "/\n"
  "#fs{font-size:13px;color:#9aa3ad;margin:4px 0 2px}\n"
  "#fb{display:flex;gap:8px;margin:6px 0}#fb button{flex:1}\n"
  "#fon{background:#1f5c3a}#foff{background:#4a2020}\n"
  ".fx{background:#151b21;border:1px solid #232c35;border-radius:10px;\n"
  "padding:8px 9px;margin:6px 0}\n"
  ".fx .t{display:flex;align-items:center;gap:8px;font-size:13px;color:#cfd5db}\n"
  "  .fx .t b{color:#8ecbff;font-size:13.5px}\n"
  ".fx .rv{margin-left:auto;font:12px monospace;color:#ffd79a}\n"
  ".fx select,.fx input{background:#12171d;color:#e8eaed;border:1px solid #2b333c;\n"
  "border-radius:7px;padding:6px;font:13px system-ui}\n"
  ".fx input{width:56px}\n"
  ".fx .rw{display:flex;gap:6px;align-items:center;margin-top:6px;flex-wrap:wrap}\n"
  ".fx button{padding:7px 10px;font-size:12.5px}\n"
  "/" "* PART 1: the per-motor speed and deadband sliders *" "/\n"
  ".fx .lb{width:82px;font-size:12px;color:#9aa3ad}\n"
  ".fx input.sl{flex:1;min-width:120px;width:auto;padding:0;height:34px;\n"
  "background:transparent;border:0;touch-action:none}\n"
  ".fx .nv{width:52px;text-align:right;font:12px monospace;color:#ffd79a}\n"
  "/" "* PART 2: the Manual Servo Control master switch *" "/\n"
  ".mmr{display:flex;align-items:center;gap:12px;margin:11px 0;padding:11px;\n"
  "background:#161d24;border:1px solid #2b3a45;border-radius:13px}\n"
  ".mmr button{flex:0 0 auto;padding:12px 16px}\n"
  "#js.lk{opacity:.42;pointer-events:none}\n"
  "textarea{width:100%;margin-top:9px;background:#161b21;color:#cfd5db;\n"
  "border:1px solid #2b333c;border-radius:9px;padding:9px;font:12.5px monospace}\n"
  "</style></head><body>\n"
  "<h1>Humanoid</h1><div id=st>connecting...</div>\n"
  "<button id=stop onclick=\"stopTap()\">STOP</button>\n"
  "<button id=auto onclick=\"go('demo')\">Sit, get up and walk by itself</button>\n"
  "<button id=mic onclick=\"micTap()\">&#127908; Tap and speak</button>\n"
  "<div id=vs>voice: ready</div>\n"
  "\n"
  "<div id=mh>\n"
  "<b>The microphone needs one Chrome setting</b><br>\n"
  "Chrome only allows the mic on <code>https</code> pages, and the robot has\n"
  "no certificate to serve <code>https</code> with. So you tell Chrome to\n"
  "trust this one address. It takes about thirty seconds and it is\n"
  "permanent &mdash; you never do it again.\n"
  "<ol>\n"
  "<li>Copy this and open it in a new Chrome tab:\n"
  "<input id=mf readonly value=\"chrome:/" "/flags/#unsafely-treat-insecure-origin-as-secure\">\n"
  "<div class=hb><button onclick=\"cp(mf)\">Copy the settings address</button></div>\n"
  "</li>\n"
  "<li>In the box on that page, paste the robot's address:\n"
  "<input id=mo readonly value=\"\">\n"
  "<div class=hb><button onclick=\"cp(mo)\">Copy the robot address</button></div>\n"
  "</li>\n"
  "<li>Change the dropdown next to it from <b>Default</b> to <b>Enabled</b>.</li>\n"
  "<li>Tap <b>Relaunch</b> at the bottom.</li>\n"
  "<li>Come back to this page and tap the mic. Allow it when asked.</li>\n"
  "</ol>\n"
  "<b>Or, with no settings at all:</b> open <code>voice_remote.html</code>\n"
  "from your phone's Downloads. A page opened from the phone's own storage\n"
  "is trusted, so the mic just works, and it talks to the robot over the\n"
  "same WiFi.\n"
  "<div class=hb><button onclick=\"mh.style.display='none'\">Hide this</button>\n"
  "<button onclick=\"micTap()\">Try the mic again</button></div>\n"
  "</div>\n"
  "\n"
  "<h2>Feelings &mdash; face, voice and head together</h2>\n"
  "<div class=\"g e\" id=ge></div>\n"
  "<h2>Speaker volume &mdash; <span id=vv>100</span></h2>\n"
  "<input type=range min=0 max=200 step=5 value=100 id=vol\n"
  " oninput=\"vv.textContent=this.value\" onchange=\"go('vol '+this.value)\">\n"
  "<div style=\"font-size:12px;color:#7d868f;margin:-4px 0 8px\">\n"
  "100 = full scale. Above 100 a soft limiter adds real loudness without\n"
  "the crackle of hard clipping &mdash; try 150. Use 200 for the absolute\n"
  "maximum this amplifier and speaker can give.</div>\n"
  "<div class=r><button onclick=\"go('mute')\">Mute</button>\n"
  "<button onclick=\"go('unmute')\">Unmute</button>\n"
  "<button onclick=\"go('sound win')\">Test sound</button></div>\n"
  "\n"
  "<h2>Posture</h2><div class=\"g p\" id=gp></div>\n"
  "<h2>Steadiness</h2>\n"
  "<div class=r><button onclick=\"go('balance on')\">Balance on</button>\n"
  "<button onclick=\"go('balance off')\">Balance off</button></div>\n"
  "<div class=r><button onclick=\"go('quiet')\">Jitter test</button>\n"
  "<button onclick=\"go('bal zero')\">Learn upright</button></div>\n"
  "<div class=r><button id=aupb onclick=\"aupSet(1)\">Auto stand-up ON</button>\n"
  "<button onclick=\"aupSet(0)\">Auto stand-up OFF</button></div>\n"
  "<button onclick=\"say('/aupnow')\" style=\"width:100%;background:#17352a;padding:13px\">\n"
  "&#9878; Stand up now</button>\n"
  "<div style=\"font-size:12.5px;color:#7d868f\">Auto stand-up watches the MPU6050 and\n"
  "only fires on a real fall &mdash; more than 45&deg; held for a moment. It runs the\n"
  "normal Stand Up routine, tries three times, then stops and waits for you. Balance\n"
  "trim is held off while it works, so the two never fight.</div>\n"
  "<h2>Moves</h2><div class=g id=gm></div>\n"
  "<h2>Your original walk, for comparison</h2><div class=\"g d\" id=gd></div>\n"
  "<button id=xw onclick=\"go('experimental walk')\">&#128300; EXPERIMENTAL WALK</button>\n"
  "<h2>Motor speed &mdash; <span id=sv>80</span> <span style=color:#5c646c>(0 = slowest, 100 = fastest)</span></h2>\n"
  "<input type=range min=0 max=100 step=5 value=80 id=sl\n"
  " oninput=\"sv.textContent=this.value\" onchange=\"setSpeed(this.value)\">\n"
  "<button style=\"margin-top:8px\" onclick=\"sl.value=80;sv.textContent=80;go('speed auto')\">Auto speed</button>\n"
  "\n"
  "<h2>Servo angles &mdash; one motor at a time</h2>\n"
  "<div id=mmw>\n"
  "<div class=mmr><span style=\"flex:1\"><b>Manual Servo Control</b><br>\n"
  "<span id=mms style=\"font-size:12px;color:#7d868f\">checking...</span></span>\n"
  "<button id=mmb onclick=\"mSet(!manOn)\">&mdash;</button></div></div>\n"
  "<div id=js></div>\n"
  "<div class=r><button onclick=\"readBack()\">Read robot angles</button>\n"
  "<button onclick=\"savePos()\">Save as new position</button></div>\n"
  "<textarea id=tx rows=4 readonly placeholder=\"Saved angles appear here so you can copy them\"></textarea>\n"
  "<h2>Saved positions</h2><div class=g id=gs></div>\n"
  "\n"
  "<h2>Draw on your phone &rarr; the robot's face</h2>\n"
  "<canvas id=cv width=128 height=64\n"
  " style=\"width:100%;height:auto;background:#000;border-radius:11px;touch-action:none;\n"
  "image-rendering:pixelated;border:1px solid #2b3239\"></canvas>\n"
  "<div class=g style=\"margin-top:8px\">\n"
  "<button id=dpen onclick=\"dMode(1)\">Pen</button>\n"
  "<button onclick=\"dMode(0)\">Eraser</button>\n"
  "<button id=dsz onclick=\"dBrush()\">Brush 3</button>\n"
  "<button onclick=\"dClear()\">Clear</button></div>\n"
  "<div class=g style=\"margin-top:8px\">\n"
  "<button onclick=\"dSend()\" style=\"background:#1d4d7c\">Send drawing to OLED</button>\n"
  "<button onclick=\"say('/oface')\">Back to faces</button></div>\n"
  "<div class=r style=\"margin-top:8px\">\n"
  "<button onclick=\"dSlot(0)\" id=dsl0>Slot 1</button>\n"
  "<button onclick=\"dSlot(1)\" id=dsl1>Slot 2</button>\n"
  "<button onclick=\"dSlot(2)\" id=dsl2>Slot 3</button></div>\n"
  "<div class=g style=\"margin-top:8px\">\n"
  "<button onclick=\"dSave()\" style=\"background:#17352a\">Save permanently</button>\n"
  "<button onclick=\"dLoad()\">Load saved</button>\n"
  "<button onclick=\"dDel()\" style=\"background:#3a1f1f\">Delete saved</button>\n"
  "<button onclick=\"dSlide()\">Slideshow</button></div>\n"
  "<input id=dtx maxlength=40 placeholder=\"type words for the OLED\"\n"
  " style=\"width:100%;margin-top:8px;padding:12px;border:0;border-radius:10px;\n"
  "background:#1a2027;color:#e8eaed;font:15px system-ui\">\n"
  "<div class=g style=\"margin-top:8px\">\n"
  "<button onclick=\"dText()\">Show the words</button>\n"
  "<button onclick=\"dBanD(-1)\">Scroll &larr;</button>\n"
  "<button onclick=\"dBanD(1)\">Scroll &rarr;</button></div>\n"
  "<div class=jr><span class=jn>Left</span>\n"
  "<input id=txx type=range min=-40 max=127 value=6 oninput=\"tPrev()\"\n"
  " onchange=\"tPush()\"><span class=jv id=txxv>6</span></div>\n"
  "<div class=jr><span class=jn>Top</span>\n"
  "<input id=txy type=range min=0 max=56 value=24 oninput=\"tPrev()\"\n"
  " onchange=\"tPush()\"><span class=jv id=txyv>24</span></div>\n"
  "<div class=jr><span class=jn>Size</span>\n"
  "<input id=txs type=range min=1 max=4 value=2 oninput=\"tPrev()\"\n"
  " onchange=\"tPush()\"><span class=jv id=txsv>2</span></div>\n"
  "<div class=jr><span class=jn>Speed</span>\n"
  "<input id=txv type=range min=5 max=200 value=40 oninput=\"tPrev()\"\n"
  " onchange=\"tPush()\"><span class=jv id=txvv>40</span></div>\n"
  "<div class=g style=\"margin-top:8px\">\n"
  "<button id=tdrag onclick=\"tDrag()\">Drag the text: off</button>\n"
  "<button id=tover onclick=\"tOver()\">Over the drawing: no</button></div>\n"
  "<div class=g style=\"margin-top:8px\">\n"
  "<button onclick=\"tCent()\">Centre it</button>\n"
  "<button onclick=\"say('/otext?t=')\">Clear the words</button></div>\n"
  "<div id=dst style=\"font-size:12.5px;color:#7d868f;margin-top:6px\">\n"
  "Draw with your finger. Saving keeps it in the ESP32 flash, so it comes back after\n"
  "the power has been off, and only Delete removes it.</div>\n"
  "\n"
  "<h2>Record your voice &mdash; the robot plays it back</h2>\n"
  "<div class=g>\n"
  "<button onclick=\"mStart()\" style=\"background:#3a1f1f\">&#9210; Record</button>\n"
  "<button onclick=\"mStop()\">&#9209; Stop</button>\n"
  "<button onclick=\"say('/vplay')\" style=\"background:#17352a\">&#9654; Play</button>\n"
  "<button onclick=\"say('/vstop')\">Silence</button></div>\n"
  "<div class=g style=\"margin-top:8px\">\n"
  "<button onclick=\"say('/vsave')\">Keep after restart</button>\n"
  "<button onclick=\"say('/vclear')\" style=\"background:#3a1f1f\">Delete recording</button></div>\n"
  "<div id=vst style=\"font-size:12.5px;color:#7d868f;margin-top:6px\">\n"
  "Uses your phone's microphone, which is the one microphone that is guaranteed to be\n"
  "wired up. About two seconds, played through the MAX98357A.</div>\n"
  "\n"
  "<h2>Stored pictures &amp; sounds &mdash; as many as the flash holds</h2>\n"
  "<input id=fnm maxlength=20 placeholder=\"name it, e.g. smile1\"\n"
  " style=\"width:100%;padding:12px;border:0;border-radius:10px;\n"
  "background:#1a2027;color:#e8eaed;font:15px system-ui\">\n"
  "<div class=g style=\"margin-top:8px\">\n"
  "<button onclick=\"fSave('i')\" style=\"background:#17352a\">Store this drawing</button>\n"
  "<button onclick=\"fSave('a')\" style=\"background:#17352a\">Store this recording</button></div>\n"
  "<div class=g style=\"margin-top:8px\">\n"
  "<button onclick=\"fList()\">Refresh the list</button></div>\n"
  "<div id=fls style=\"font-size:12.5px;color:#7d868f;margin-top:8px\">\n"
  "press Refresh the list</div>\n"
  "\n"
  "<h2>Hand controller &mdash; Wrist, Elbow and Inside Elbow motors</h2>\n"
  "<div id=fs>checking...</div>\n"
  "<div id=fb><button id=fon onclick=\"fOn(1)\">Flex Control ON</button>\n"
  "<button id=foff onclick=\"fOn(0)\">Flex Control OFF</button></div>\n"
  "<button onclick=\"fConn()\" style=\"width:100%;background:#1d4d7c;padding:13px\">\n"
  "Connect ESP8266 &mdash; check the glove</button>\n"
  "<div style=\"font-size:12.5px;color:#7d868f\">\n"
  "Wear the glove, power it up, it joins this hotspot by itself. Nothing to type.\n"
  "The switch on the glove picks the arm: the same three sensors drive\n"
  "<b>rh1 rh2 rh3</b> or <b>lh1 lh2 lh3</b>.\n"
  "Flex Control never blocks the robot &mdash; press any movement or STOP and that\n"
  "wins; the glove takes over again when the movement ends.</div>\n"
  "<div style=\"font-size:12.5px;color:#7d868f;margin-top:6px\">\n"
  "To calibrate: hold the finger <b>straight</b> and press Straight, then\n"
  "<b>bend it fully</b> and press Bent. One calibration per finger, it serves both\n"
  "arms. Saved permanently.</div>\n"
  "<div id=fxw></div>\n"
  "<script>\n"
  "var P=[\"stand\",\"sit down\",\"stand up\",\"move forward\",\"turn left\",\"turn right\",\"push ups\"];\n"
  "var M=[\"introduce\",\"say hi\",\"shake hand\",\"hands up\",\"hands down\",\"right bicep\",\n"
  "\"left bicep\",\"double biceps\",\"maan karate\",\"bow\",\"ape move\",\"exercise one\",\n"
  "\"side bend\",\"dab\",\"gangnam style\",\"dance\",\"smile\"];\n"
  "var D=[\"old walk\",\"old turn left\",\"old turn right\"];\n"
  "/" "*  These go straight down /run as bare words. The sketch looks a bare\n"
  " *  word up in its mood table last of all, so a mood can never shadow a\n"
  " *  real move -- see handleConfigCommand().                            *" "/\n"
  "var E=[\"happy\",\"smile\",\"angry\",\"sad\",\"surprised\",\"love\",\"wink\",\"cool\",\n"
  "\"proud\",\"curious\",\"determined\",\"dizzy\",\"sleepy\",\"music\",\"bye\",\"confused\",\n"
  "\"alert\",\"neutral\"];\n"
  "function mk(e,a){a.forEach(function(c){var b=document.createElement(\"button\");\n"
  "b.textContent=c;b.onclick=function(){go(c)};e.appendChild(b)})}\n"
  "mk(gp,P);mk(gm,M);mk(gd,D);mk(ge,E);\n"
  "mo.value=location.origin;\n"
  "\n"
  "/" "*  navigator.clipboard is itself an https-only API, so on this page it\n"
  " *  is usually missing. Selecting the text is the reliable fallback:\n"
  " *  the phone then offers Copy in its own menu.                        *" "/\n"
  "function cp(el){\n"
  "  el.focus();el.setSelectionRange(0,el.value.length);\n"
  "  var done=false;\n"
  "  try{done=document.execCommand(\"copy\")}catch(e){}\n"
  "  if(!done&&navigator.clipboard){try{navigator.clipboard.writeText(el.value);\n"
  "    done=true}catch(e){}}\n"
  "  vs.textContent=done?\"copied\":\"selected - now tap Copy\";\n"
  "}\n"
  "function go(c){fetch(\"/run?c=\"+encodeURIComponent(c));st.textContent=\"sent: \"+c}\n"
  "\n"
  "/" "* ---------- VOICE ----------\n"
  " * The ESP32 does no speech work at all. The browser turns your voice\n"
  " * into text, this table turns the text into one of the robot commands,\n"
  " * and then it goes down the same /run path as the buttons. So anything\n"
  " * a button can do, your voice can do.\n"
  " * Longest matching phrase wins, so \"stand up\" beats \"stand\".         *" "/\n"
  "var V=[\n"
  "[\"stop\",\"stop\"],[\"freeze\",\"stop\"],[\"halt\",\"stop\"],\n"
  "[\"stand up\",\"stand up\"],[\"get up\",\"stand up\"],[\"stand straight\",\"stand\"],\n"
  "[\"standing\",\"stand\"],[\"stand\",\"stand\"],\n"
  "[\"sit down\",\"sit down\"],[\"sit\",\"sit down\"],\n"
  "[\"walk\",\"move forward\"],[\"move forward\",\"move forward\"],[\"go forward\",\"move forward\"],\n"
  "[\"forward\",\"move forward\"],[\"walk forward\",\"move forward\"],\n"
  "[\"turn left\",\"turn left\"],[\"left turn\",\"turn left\"],\n"
  "[\"turn right\",\"turn right\"],[\"right turn\",\"turn right\"],\n"
  "[\"push up\",\"push ups\"],[\"push ups\",\"push ups\"],[\"pushups\",\"push ups\"],\n"
  "[\"do it yourself\",\"demo\"],[\"by yourself\",\"demo\"],[\"demo\",\"demo\"],\n"
  "[\"show me\",\"demo\"],[\"automatic\",\"demo\"],\n"
  "[\"say hi\",\"say hi\"],[\"hi\",\"say hi\"],[\"hello\",\"say hi\"],[\"wave\",\"say hi\"],\n"
  "[\"introduce yourself\",\"introduce\"],[\"introduce\",\"introduce\"],\n"
  "[\"introduction\",\"introduce\"],[\"who are you\",\"introduce\"],\n"
  "[\"shake hand\",\"shake hand\"],[\"handshake\",\"shake hand\"],\n"
  "[\"hands up\",\"hands up\"],[\"hands down\",\"hands down\"],\n"
  "[\"right bicep\",\"right bicep\"],[\"left bicep\",\"left bicep\"],\n"
  "[\"double bicep\",\"double biceps\"],[\"flex\",\"double biceps\"],\n"
  "[\"karate\",\"maan karate\"],[\"bow\",\"bow\"],[\"ape\",\"ape move\"],\n"
  "[\"exercise\",\"exercise one\"],[\"side bend\",\"side bend\"],[\"dab\",\"dab\"],\n"
  "[\"gangnam\",\"gangnam style\"],[\"dance\",\"dance\"],[\"smile\",\"smile\"],\n"
  "[\"balance on\",\"balance on\"],[\"balance off\",\"balance off\"],\n"
  "[\"turn on balance\",\"balance on\"],[\"turn off balance\",\"balance off\"],\n"
  "[\"jitter\",\"quiet\"],[\"quiet\",\"quiet\"],[\"relax\",\"relax\"],\n"
  "/" "*  moods. Saying one is enough -- no keyword needed.                 *" "/\n"
  "[\"be happy\",\"happy\"],[\"happy\",\"happy\"],[\"smile\",\"smile\"],\n"
  "[\"angry\",\"angry\"],[\"get angry\",\"angry\"],[\"be angry\",\"angry\"],\n"
  "[\"sad\",\"sad\"],[\"love\",\"love\"],[\"i love you\",\"love\"],\n"
  "[\"surprised\",\"surprised\"],[\"surprise\",\"surprised\"],\n"
  "[\"wink\",\"wink\"],[\"be cool\",\"cool\"],[\"cool\",\"cool\"],\n"
  "[\"proud\",\"proud\"],[\"curious\",\"curious\"],[\"determined\",\"determined\"],\n"
  "[\"dizzy\",\"dizzy\"],[\"sleepy\",\"sleepy\"],[\"go to sleep\",\"sleepy\"],\n"
  "[\"music\",\"music\"],[\"bye\",\"bye\"],[\"goodbye\",\"bye\"],[\"good bye\",\"bye\"],\n"
  "[\"confused\",\"confused\"],[\"neutral\",\"neutral\"],[\"normal\",\"neutral\"],\n"
  "[\"mute\",\"mute\"],[\"be quiet\",\"mute\"],[\"unmute\",\"unmute\"],\n"
  "[\"louder\",\"vol 80\"],[\"quieter\",\"vol 25\"],[\"volume up\",\"vol 80\"],\n"
  "[\"volume down\",\"vol 25\"],[\"nod\",\"head nod\"],[\"shake your head\",\"head shake\"],\n"
  "[\"look left\",\"head look left\"],[\"look right\",\"head look right\"]];\n"
  "\n"
  "function match(t){\n"
  "  t=t.toLowerCase().replace(/[^a-z0-9 ]/g,\" \").replace(/\\s+/g,\" \").trim();\n"
  "  var best=null,len=0;\n"
  "  for(var i=0;i<V.length;i++){\n"
  "    if(t.indexOf(V[i][0])>=0 && V[i][0].length>len){len=V[i][0].length;best=V[i][1]}\n"
  "  }\n"
  "  return best;\n"
  "}\n"
  "\n"
  "var SR=window.SpeechRecognition||window.webkitSpeechRecognition,rec=null,on=false;\n"
  "/" "*  Tell the user up front rather than letting them tap and fail. An\n"
  " *  http page is not a secure context, and that is the whole problem. *" "/\n"
  "if(!window.isSecureContext){\n"
  "  vs.textContent=\"Mic needs one Chrome setting - tap to see how\";\n"
  "  vs.style.color=\"#ffb95e\";vs.style.cursor=\"pointer\";\n"
  "  vs.onclick=function(){mh.style.display=\"block\";\n"
  "    mh.scrollIntoView({behavior:\"smooth\"})};\n"
  "}\n"
  "function micTap(){\n"
  "  if(!SR){vs.textContent=\"This browser cannot listen. Use Chrome, or open voice_remote.html\";return}\n"
  "  if(!window.isSecureContext&&mh.style.display!=\"block\"){\n"
  "    mh.style.display=\"block\";mh.scrollIntoView({behavior:\"smooth\"});\n"
  "  }\n"
  "  if(on){rec.stop();return}\n"
  "  if(!rec){\n"
  "    rec=new SR();rec.lang=\"en-IN\";rec.interimResults=false;\n"
  "    rec.maxAlternatives=4;rec.continuous=false;\n"
  "    rec.onstart=function(){on=true;mic.className=\"on\";\n"
  "      mic.innerHTML=\"&#127908; Listening - speak now\";vs.textContent=\"listening...\"};\n"
  "    rec.onend=function(){on=false;mic.className=\"\";\n"
  "      mic.innerHTML=\"&#127908; Tap and speak\"};\n"
  "    rec.onerror=function(e){\n"
  "      if(e.error==\"not-allowed\"||e.error==\"service-not-allowed\"){\n"
  "        vs.textContent=\"Chrome is blocking the mic. Here is the fix:\";\n"
  "        mh.style.display=\"block\";mh.scrollIntoView({behavior:\"smooth\"});\n"
  "      }else if(e.error==\"no-speech\"){\n"
  "        vs.textContent=\"did not hear anything - tap and speak closer\";\n"
  "      }else{vs.textContent=\"mic error: \"+e.error}};\n"
  "    rec.onresult=function(e){\n"
  "      var r=e.results[0],hit=null,heard=r[0].transcript;\n"
  "      for(var i=0;i<r.length;i++){var m=match(r[i].transcript);\n"
  "        if(m){hit=m;heard=r[i].transcript;break}}\n"
  "      if(hit){vs.textContent=\"heard \\\"\"+heard+\"\\\" -> \"+hit;go(hit)}\n"
  "      else{vs.textContent=\"heard \\\"\"+heard+\"\\\" - no matching command\"}};\n"
  "  }\n"
  "  try{rec.start()}catch(err){vs.textContent=\"mic busy, tap again\"}\n"
  "}\n"
  "var busy=false;\n"
  "/" "* Tapping STOP while it is moving aborts. Tapping STOP when it is\n"
  " * already stopped brings it back to the initial standing position. *" "/\n"
  "function stopTap(){if(busy){go(\"stop\")}else{go(\"stand\")}}\n"
  "/" "* 0..100 -> the sketch's timing percent, where higher = slower. *" "/\n"
  "function setSpeed(v){go(\"speed \"+Math.round(300-2.5*v))}\n"
  "\n"
  "var JN=[],JA=[];\n"
  "/" "* ---- PART 2: the Manual Servo Control master switch -----------------\n"
  " *  manOn is a copy of the robot's own gManualOn, refreshed every time\n"
  " *  the sliders are (re)built. When it is OFF the whole slider block is\n"
  " *  greyed out, every single input carries the real HTML  disabled\n"
  " *  attribute, the block also gets  pointer-events:none  so a stray\n"
  " *  touch or drag cannot reach a slider at all, and setJoint() refuses\n"
  " *  to send anything. The robot refuses too, in  handleJoint() , so\n"
  " *  even an old browser tab or a bookmarked  /joint?  URL cannot move a\n"
  " *  servo while the switch is OFF. Nothing else on this page is\n"
  " *  affected: poses, movements, sounds, faces and the flex glove all\n"
  " *  keep working exactly as before.                                  *" "/\n"
  "var manOn=true;\n"
  "function manPaint(){\n"
  " var b=document.getElementById(\"mmb\"),s=document.getElementById(\"mms\");\n"
  " if(b){b.textContent=manOn?\"Turn OFF\":\"Turn ON\";\n"
  "  b.style.background=manOn?\"#1f3a2a\":\"#3a1f24\";\n"
  "  b.style.borderColor=manOn?\"#3f7d5c\":\"#7d3f4a\"}\n"
  " if(s){s.innerHTML=manOn\n"
  "  ?\"<b style='color:#7ee2a8'>ON</b> &mdash; the sliders below move the servos.\"\n"
  "  :\"<b style='color:#ff9a9a'>OFF</b> &mdash; the sliders below are locked; \"+\n"
  "   \"touching or dragging them cannot move or change any servo.\"}\n"
  " js.className=manOn?\"\":\"lk\";\n"
  " var L=js.getElementsByTagName(\"input\"),k;\n"
  " for(k=0;k<L.length;k++){L[k].disabled=!manOn}\n"
  "}\n"
  "function mSet(v){\n"
  " manOn=v?true:false;manPaint();\n"
  " fetch(\"/manual?v=\"+(v?1:0)).then(function(r){return r.text()})\n"
  "  .then(function(t){st.textContent=t;buildJoints()})\n"
  "  .catch(function(){st.textContent=\"could not reach the robot\"})}\n"
  "function buildJoints(){\n"
  " fetch(\"/angles\").then(function(r){return r.json()}).then(function(j){\n"
  "  JN=j.n;JA=j.a;js.innerHTML=\"\";\n"
  "  JN.forEach(function(n,i){\n"
  "   var d=document.createElement(\"div\");d.className=\"jr\";\n"
  "   var l=document.createElement(\"span\");l.className=\"jn\";l.textContent=n;\n"
  "   var r=document.createElement(\"input\");r.type=\"range\";r.min=0;r.max=180;r.value=JA[i];\n"
  "   var vv=document.createElement(\"span\");vv.className=\"jv\";vv.textContent=JA[i];\n"
  "   r.oninput=function(){vv.textContent=this.value};\n"
  "   r.onchange=function(){setJoint(i,this.value)};\n"
  "   d.appendChild(l);d.appendChild(r);d.appendChild(vv);js.appendChild(d);\n"
  "  });\n"
  "  if(j.man!==undefined)manOn=(j.man!=0);\n"
  "  manPaint();\n"
  " }).catch(function(){});\n"
  "}\n"
  "function setJoint(i,a){\n"
  " if(!manOn){st.textContent=\n"
  "  \"Manual Servo Control is OFF -- turn it ON to use the sliders\";\n"
  "  buildJoints();return}\n"
  " JA[i]=+a;fetch(\"/joint?j=\"+i+\"&a=\"+a);\n"
  " st.textContent=JN[i]+\" to \"+a+\"\\u00b0\"}\n"
  "function readBack(){buildJoints();st.textContent=\"read the robot's angles\"}\n"
  "function savePos(){\n"
  " var nm=prompt(\"Name this position:\",\"my pose\");\n"
  " if(!nm)return;\n"
  " fetch(\"/plist\").then(function(r){return r.json()}).then(function(L){\n"
  "  var f=-1,i;\n"
  "  for(i=0;i<L.length;i++){if(!L[i].used){f=i;break}}\n"
  "  if(f<0){f=parseInt(prompt(\"All slots full. Overwrite which one? (1-\"+L.length+\")\",\"1\"),10)-1}\n"
  "  if(isNaN(f)||f<0||f>=L.length)return;\n"
  "  fetch(\"/psave?i=\"+f+\"&n=\"+encodeURIComponent(nm)).then(function(){\n"
  "   var t=\"\";for(var k=0;k<JN.length;k++){t+=JN[k]+\" \"+JA[k]+(k<JN.length-1?\", \":\"\")}\n"
  "   tx.value=nm+\"\\n\"+t;loadSlots();st.textContent=\"saved: \"+nm});\n"
  " });\n"
  "}\n"
  "function loadSlots(){\n"
  " fetch(\"/plist\").then(function(r){return r.json()}).then(function(L){\n"
  "  gs.innerHTML=\"\";var any=false;\n"
  "  L.forEach(function(s){\n"
  "   if(!s.used)return;any=true;\n"
  "   var b=document.createElement(\"button\");b.textContent=s.name;\n"
  "   b.onclick=function(){go(\"slot \"+(s.i+1))};\n"
  "   b.oncontextmenu=function(e){e.preventDefault();\n"
  "    if(confirm(\"Delete \\\"\"+s.name+\"\\\"?\")){fetch(\"/pdel?i=\"+s.i).then(loadSlots)}};\n"
  "   gs.appendChild(b);\n"
  "  });\n"
  "  if(!any){gs.innerHTML=\"<div style='font-size:13px;color:#7d868f'>\"+\n"
  "   \"None yet. Set the sliders above, then tap Save as new position.\"+\n"
  "   \"</div>\"}\n"
  " }).catch(function(){});\n"
  "}\n"
  "\n"
  "/" "* ---- the ESP8266 flex glove, 3 sensors, right or left arm -----------\n"
  " *  All the calibration lives on the robot, so the glove itself only\n"
  " *  ever sends three raw numbers and which way its switch is set.     *" "/\n"
  "var fxBuilt=false;\n"
  "function fOn(v){fetch(v?\"/fon\":\"/foff\").then(function(r){return r.text()})\n"
  " .then(function(t){st.textContent=t;fPoll()})}\n"
  "function fConn(){\n"
  " fetch(\"/fstat\").then(function(r){return r.json()}).then(function(d){\n"
  "  st.textContent=d.link\n"
  "   ?(\"ESP8266 glove connected, \"+(d.side?\"LEFT\":\"RIGHT\")+\" arm selected. \"+\n"
  "     d.pkts+\" packets so far.\")\n"
  "   :(\"ESP8266 not seen yet. Power the glove on -- it joins this hotspot \"+\n"
  "     \"by itself, give it about 5 seconds.\");\n"
  "  fPoll()})}\n"
  "function fCal(i,w){fetch(\"/fcal?i=\"+i+\"&w=\"+w).then(function(r){return r.text()})\n"
  " .then(function(t){st.textContent=t;fPoll()})}\n"
  "function fSet(i,sd){\n"
  " var a=document.getElementById((sd?\"fl\":\"fr\")+\"a\"+i).value;\n"
  " var b=document.getElementById((sd?\"fl\":\"fr\")+\"b\"+i).value;\n"
  " fetch(\"/fset?i=\"+i+\"&side=\"+sd+\"&al=\"+a+\"&ah=\"+b)\n"
  "  .then(function(r){return r.text()}).then(function(t){st.textContent=t})}\n"
  "function fShow(id,v){var e=document.getElementById(id);if(e)e.textContent=v}\n"
  "/" "* PART 1: one speed limiter and one deadband PER MOTOR. Only the\n"
  " * motor whose slider you touched is sent to the robot, and the robot\n"
  " * only ever changes that one motor's numbers, so adjusting the Wrist\n"
  " * Motor can never nudge the Elbow Motor or the Inside Elbow Motor. *" "/\n"
  "function fLim(i){\n"
  " var sp=document.getElementById(\"fsp\"+i).value;\n"
  " var db=document.getElementById(\"fdb\"+i).value;\n"
  " fShow(\"fspv\"+i,sp+\"\\u00b0\");fShow(\"fdbv\"+i,db+\"\\u00b0\");\n"
  " fetch(\"/flim?i=\"+i+\"&sp=\"+sp+\"&db=\"+db)\n"
  "  .then(function(r){return r.text()}).then(function(t){st.textContent=t})\n"
  "  .catch(function(){})}\n"
  "function fBuild(d){\n"
  " fxw.innerHTML=\"\";\n"
  " d.s.forEach(function(x,i){\n"
  "  var w=document.createElement(\"div\");w.className=\"fx\";\n"
  "  w.innerHTML=\n"
  "   \"<div class=t><b>\"+x.nm+\"</b>\"+\n"
  "   \"<span style='color:#7d868f;font-size:11.5px'>&nbsp;sensor \"+(i+1)+\"</span>\"+\n"
  "   \"<span class=rv id=fv\"+i+\">--</span></div>\"+\n"
  "   \"<div class=rw><button onclick=\\\"fCal(\"+i+\",'lo')\\\">Straight</button>\"+\n"
  "   \"<button onclick=\\\"fCal(\"+i+\",'hi')\\\">Bent</button>\"+\n"
  "   \"<span id=fc\"+i+\" style='font:11.5px monospace;color:#7d868f'></span></div>\"+\n"
  "   \"<div class=rw><span class=lb>Speed limit</span>\"+\n"
  "   \"<input class=sl type=range id=fsp\"+i+\" min=1 max=\"+d.spmax+\n"
  "   \" step=1 value=\"+x.sp+\n"
  "   \" oninput=\\\"fShow('fspv\"+i+\"',this.value+'\\\\u00b0')\\\"\"+\n"
  "   \" onchange=\\\"fLim(\"+i+\")\\\">\"+\n"
  "   \"<span class=nv id=fspv\"+i+\">\"+x.sp+\"\\u00b0</span></div>\"+\n"
  "   \"<div class=rw><span class=lb>Deadband</span>\"+\n"
  "   \"<input class=sl type=range id=fdb\"+i+\" min=0 max=\"+d.dbmax+\n"
  "   \" step=1 value=\"+x.db+\n"
  "   \" oninput=\\\"fShow('fdbv\"+i+\"',this.value+'\\\\u00b0')\\\"\"+\n"
  "   \" onchange=\\\"fLim(\"+i+\")\\\">\"+\n"
  "   \"<span class=nv id=fdbv\"+i+\">\"+x.db+\"\\u00b0</span></div>\"+\n"
  "   \"<div class=rw><span style='width:74px;font-size:12.5px;color:#8ecbff'>\"+\n"
  "   x.nr+\"</span>straight <input id=fra\"+i+\" type=number min=0 max=180 value=\"+\n"
  "   x.alr+\">bent <input id=frb\"+i+\" type=number min=0 max=180 value=\"+x.ahr+\n"
  "   \"><button onclick=\\\"fSet(\"+i+\",0)\\\">Save</button></div>\"+\n"
  "   \"<div class=rw><span style='width:74px;font-size:12.5px;color:#ffb0e0'>\"+\n"
  "   x.nl+\"</span>straight <input id=fla\"+i+\" type=number min=0 max=180 value=\"+\n"
  "   x.all+\">bent <input id=flb\"+i+\" type=number min=0 max=180 value=\"+x.ahl+\n"
  "   \"><button onclick=\\\"fSet(\"+i+\",1)\\\">Save</button></div>\";\n"
  "  fxw.appendChild(w);\n"
  " });\n"
  " fxBuilt=true;\n"
  "}\n"
  "function fPoll(){\n"
  " fetch(\"/fstat\").then(function(r){return r.json()}).then(function(d){\n"
  "  if(!fxBuilt)fBuild(d);\n"
  "  fs.innerHTML=\"Flex Control <b style='color:\"+(d.on?\"#7ee2a8\":\"#ff9a9a\")+\"'>\"+\n"
  "   (d.on?\"ON\":\"OFF\")+\"</b> &nbsp; glove <b style='color:\"+\n"
  "   (d.link?\"#7ee2a8\":\"#ff9a9a\")+\"'>\"+(d.link?\"connected\":\"not connected\")+\n"
  "   \"</b> &nbsp; switch <b style='color:\"+(d.side?\"#ffb0e0\":\"#8ecbff\")+\"'>\"+\n"
  "   (d.side?\"LEFT ARM\":\"RIGHT ARM\")+\"</b> &nbsp; packets \"+d.pkts+\n"
  "   (d.busy?\" &nbsp; (movement running &mdash; glove waiting)\":\"\");\n"
  "  fon.style.opacity=d.on?1:.55;foff.style.opacity=d.on?.55:1;\n"
  "  d.s.forEach(function(x,i){\n"
  "   var e=document.getElementById(\"fv\"+i);if(e)e.textContent=x.raw;\n"
  "   var c=document.getElementById(\"fc\"+i);\n"
  "   if(c)c.textContent=\"straight \"+x.lo+\"  bent \"+x.hi;\n"
  "  });\n"
  " }).catch(function(){fs.textContent=\"flex status unavailable\"});\n"
  "}\n"
  "setInterval(fPoll,700);fPoll();\n"
  "\n"
  "var volSet=false;\n"
  "function poll(){fetch(\"/status\").then(function(r){return r.json()}).then(function(j){\n"
  "busy=j.busy;\n"
  "st.textContent=(j.busy?\"running: \"+j.cmd:\"idle\")+\" — \"+j.state+\n"
  "\" — speed \"+j.speed+\"%\"+(j.mpu?\" — tilt \"+j.pitch+\"/\"+j.roll+\"°\"+\n"
  "(j.bal?\" — balancing\":\"\"):\"\")+\n"
  "(j.snd&&j.snd!=\"none\"?\" — sound: \"+j.snd:\"\")+\n"
  "(j.head&&j.head!=\"none\"?\" — head: \"+j.head:\"\")+\n"
  "(j.audio?\"\":\" — NO AMP FOUND\")+(j.mute?\" — muted\":\"\")+\n"
  "(j.fallen?\"  FALLEN\":\"\");\n"
  "/" "*  Take the robot's real volume once, so the slider starts truthful\n"
  " *  instead of showing 100 when flash says otherwise.                 *" "/\n"
  "if(!volSet&&typeof j.vol==\"number\"){volSet=true;vol.value=j.vol;\n"
  " vv.textContent=j.vol}\n"
  "})\n"
  ".catch(function(){st.textContent=\"no connection to robot\"})}\n"
  "/" "*======================= phone drawing =========================*" "/\n"
  "var cx=cv.getContext(\"2d\"),dpn=1,dbr=3,ddn=false,dslot=0,tdg=0,tov=0;\n"
  "var oc=document.createElement(\"canvas\");oc.width=128;oc.height=64;\n"
  "var ox=oc.getContext(\"2d\");\n"
  "function dWipe(){ox.fillStyle=\"#000\";ox.fillRect(0,0,128,64);dRender()}\n"
  "function tV(){return[+txx.value,+txy.value,+txs.value,+txv.value]}\n"
  "/" "*  The preview is the drawing layer with the words drawn on top of it,\n"
  " *  the same order the OLED uses, so what you drag is what you get.  *" "/\n"
  "function dRender(){cx.fillStyle=\"#000\";cx.fillRect(0,0,128,64);\n"
  " cx.drawImage(oc,0,0);\n"
  " var t=dtx.value;if(!t)return;var v=tV();\n"
  " cx.fillStyle=\"#fff\";cx.textBaseline=\"top\";\n"
  " cx.font=(8*v[2])+\"px monospace\";cx.fillText(t,v[0],v[1]);\n"
  " if(tdg){cx.strokeStyle=\"#4a9bd8\";cx.lineWidth=1;\n"
  "  cx.strokeRect(v[0]-1,v[1]-1,t.length*6*v[2]+2,8*v[2]+2)}}\n"
  "function tPrev(){var v=tV();txxv.textContent=v[0];txyv.textContent=v[1];\n"
  " txsv.textContent=v[2];txvv.textContent=v[3];dRender()}\n"
  "function tPush(){tPrev();dText()}\n"
  "function tSet(q){var t=dtx.value.length*6*(+txs.value);\n"
  " txx.value=Math.round(q[0]-t/2);txy.value=Math.round(q[1]-4*(+txs.value));tPrev()}\n"
  "function tDrag(){tdg=tdg?0:1;tdrag.textContent=\"Drag the text: \"+(tdg?\"on\":\"off\");\n"
  " tdrag.style.background=tdg?\"#1d4d7c\":\"#1e252d\";dRender();\n"
  " dst.textContent=tdg?\"drag the words on the picture above\":\"back to drawing with your finger\"}\n"
  "function tOver(){tov=tov?0:1;\n"
  " tover.textContent=\"Over the drawing: \"+(tov?\"yes\":\"no\");\n"
  " tover.style.background=tov?\"#17352a\":\"#1e252d\";tPush()}\n"
  "function tCent(){var t=dtx.value.length*6*(+txs.value);\n"
  " txx.value=Math.max(-40,Math.round((128-t)/2));\n"
  " txy.value=Math.round((64-8*(+txs.value))/2);tPush()}\n"
  "dWipe();\n"
  "function rt(r){return r.text()}\n"
  "function say(u){fetch(u).then(rt).then(function(t){dst.textContent=t;dstat()})\n"
  " .catch(function(){dst.textContent=\"no connection to robot\"})}\n"
  "function dxy(e){var r=cv.getBoundingClientRect();\n"
  " var t=(e.touches&&e.touches.length)?e.touches[0]:e;\n"
  " return [Math.round((t.clientX-r.left)/r.width*128),\n"
  "         Math.round((t.clientY-r.top)/r.height*64)]}\n"
  "function dpt(q){ox.fillStyle=dpn?\"#fff\":\"#000\";\n"
  " var h=dbr>>1;ox.fillRect(q[0]-h,q[1]-h,dbr,dbr);dRender()}\n"
  "function dDown(e){ddn=true;if(tdg)tSet(dxy(e));else dpt(dxy(e));\n"
  " e.preventDefault()}\n"
  "function dMove(e){if(!ddn)return;if(tdg)tSet(dxy(e));else dpt(dxy(e));\n"
  " e.preventDefault()}\n"
  "function dUp(){if(ddn&&tdg)tPush();ddn=false}\n"
  "cv.addEventListener(\"touchstart\",dDown,{passive:false});\n"
  "cv.addEventListener(\"touchmove\",dMove,{passive:false});\n"
  "cv.addEventListener(\"touchend\",dUp);\n"
  "cv.addEventListener(\"mousedown\",dDown);\n"
  "cv.addEventListener(\"mousemove\",dMove);\n"
  "window.addEventListener(\"mouseup\",dUp);\n"
  "function dMode(v){dpn=v;dpen.style.background=v?\"#1d4d7c\":\"#1e252d\";\n"
  " dst.textContent=v?\"pen\":\"eraser\"}\n"
  "function dBrush(){dbr=dbr>=7?1:dbr+2;dsz.textContent=\"Brush \"+dbr}\n"
  "function dClear(){dWipe();dst.textContent=\"cleared -- the OLED keeps what it has until you send again\"}\n"
  "function dSlot(i){dslot=i;dstat();dst.textContent=\"slot \"+(i+1)+\" selected\"}\n"
  "/" "*  Pack the canvas into the OLED frame buffer layout: one byte is eight\n"
  " *  stacked pixels, page = y>>3, bit = y&7. Same order the ESP32 uses,\n"
  " *  so it only has to memcpy what arrives.                            *" "/\n"
  "function dPack(){var d=ox.getImageData(0,0,128,64).data,b=new Array(1024);\n"
  " for(var i=0;i<1024;i++)b[i]=0;\n"
  " for(var y=0;y<64;y++)for(var x=0;x<128;x++){var o=(y*128+x)*4;\n"
  "  if(d[o]+d[o+1]+d[o+2]>250)b[(y>>3)*128+x]|=(1<<(y&7));}\n"
  " var h=\"\";for(var k=0;k<1024;k++)h+=(\"0\"+b[k].toString(16)).slice(-2);\n"
  " return h}\n"
  "function dSend(){var h=dPack();dst.textContent=\"sending...\";\n"
  " (function nx(o){if(o>=1024){say(\"/odraw?o=9999\");return}\n"
  "  fetch(\"/odraw?o=\"+o+\"&d=\"+h.substr(o*2,512)).then(function(){nx(o+256)})\n"
  "  .catch(function(){dst.textContent=\"upload failed -- try again\"})})(0)}\n"
  "function dSave(){var h=dPack();dst.textContent=\"saving...\";\n"
  " (function nx(o){if(o>=1024){say(\"/dsave?s=\"+dslot);return}\n"
  "  fetch(\"/odraw?o=\"+o+\"&d=\"+h.substr(o*2,512)).then(function(){nx(o+256)})})(0)}\n"
  "function dLoad(){say(\"/dload?s=\"+dslot)}\n"
  "function dDel(){say(\"/ddel?s=\"+dslot)}\n"
  "function dSlide(){say(\"/dslide?v=1\")}\n"
  "function dText(){var v=tV();\n"
  " say(\"/otext?t=\"+encodeURIComponent(dtx.value)+\"&x=\"+v[0]+\"&y=\"+v[1]+\n"
  "  \"&s=\"+v[2]+\"&o=\"+(tov?1:0))}\n"
  "function dBan(){dBanD(-1)}\n"
  "function dBanD(d){var v=tV();\n"
  " say(\"/oban?t=\"+encodeURIComponent(dtx.value)+\"&y=\"+v[1]+\"&s=\"+v[2]+\n"
  "  \"&d=\"+d+\"&v=\"+v[3]+\"&o=\"+(tov?1:0))}\n"
  "/" "*==================== stored pictures and sounds ================*" "/\n"
  "function fSave(k){var n=fnm.value||\"untitled\";\n"
  " if(k==\"a\"){say(\"/fsave?k=a&n=\"+encodeURIComponent(n));\n"
  "  setTimeout(fList,400);return}\n"
  " var h=dPack();dst.textContent=\"storing the picture...\";\n"
  " (function nx(o){if(o>=1024){say(\"/fsave?k=i&n=\"+encodeURIComponent(n));\n"
  "   setTimeout(fList,400);return}\n"
  "  fetch(\"/odraw?o=\"+o+\"&d=\"+h.substr(o*2,512)).then(function(){nx(o+256)})\n"
  "  .catch(function(){dst.textContent=\"upload failed -- try again\"})})(0)}\n"
  "function fList(){fetch(\"/fsl\").then(function(r){return r.json()})\n"
  " .then(function(j){\n"
  "  if(!j.ok){fls.textContent=\"the flash file store did not mount\";return}\n"
  "  var h=\"<div style='margin-bottom:6px'>\"+Math.round(j.free/1024)+\n"
  "   \" kB free of \"+Math.round(j.total/1024)+\" kB &mdash; \"+j.f.length+\n"
  "   \" stored</div>\";\n"
  "  if(!j.f.length)h+=\"nothing stored yet\";\n"
  "  j.f.forEach(function(e){\n"
  "   h+=\"<div class=jr><span style='flex:1;color:#cfd5db'>\"+\n"
  "    (e.k==\"a\"?\"&#9834; \":\"&#9635; \")+e.n+\" <span style='color:#7d868f'>\"+\n"
  "    e.s+\" B</span></span><button onclick=\\\"fGo('\"+e.k+\"','\"+e.n+\n"
  "    \"')\\\">\"+(e.k==\"a\"?\"Play\":\"Show\")+\"</button>\"+\n"
  "    \"<button style='background:#3a1f1f' onclick=\\\"fDel('\"+e.k+\"','\"+\n"
  "    e.n+\"')\\\">Delete</button></div>\"});\n"
  "  fls.innerHTML=h})\n"
  " .catch(function(){fls.textContent=\"no connection to robot\"})}\n"
  "function fGo(k,n){say(\"/fload?k=\"+k+\"&n=\"+encodeURIComponent(n))}\n"
  "function fDel(k,n){if(!confirm(\"Delete \"+n+\" for good?\"))return;\n"
  " say(\"/fdel?k=\"+k+\"&n=\"+encodeURIComponent(n));setTimeout(fList,400)}\n"
  "function aupSet(v){say(\"/aup?v=\"+v)}\n"
  "/" "*======================= phone microphone ======================*" "/\n"
  "var mRec=false,mBuf=[],mCtx=null,mStr=null,mNode=null,mGain=null;\n"
  "function mStart(){if(mRec)return;\n"
  " if(!navigator.mediaDevices){vst.textContent=\"this browser will not give the page a microphone\";return}\n"
  " navigator.mediaDevices.getUserMedia({audio:true}).then(function(s){\n"
  "  mStr=s;mCtx=new (window.AudioContext||window.webkitAudioContext)();\n"
  "  var src=mCtx.createMediaStreamSource(s);\n"
  "  mNode=mCtx.createScriptProcessor(4096,1,1);\n"
  "  mGain=mCtx.createGain();mGain.gain.value=0;\n"
  "  mBuf=[];mRec=true;\n"
  "  var step=mCtx.sampleRate/11025,pos=0;\n"
  "  mNode.onaudioprocess=function(e){if(!mRec)return;\n"
  "   var d=e.inputBuffer.getChannelData(0);\n"
  "   for(;pos<d.length;pos+=step){var v=d[Math.floor(pos)]*1.6;\n"
  "    if(v>1)v=1;if(v<-1)v=-1;\n"
  "    if(mBuf.length<24000)mBuf.push(Math.round(v*127)+128)}\n"
  "   pos-=d.length;\n"
  "   vst.textContent=\"recording \"+(mBuf.length/11025).toFixed(1)+\" s of 2.2\";\n"
  "   if(mBuf.length>=24000)mStop()};\n"
  "  src.connect(mNode);mNode.connect(mGain);mGain.connect(mCtx.destination);\n"
  "  fetch(\"/vrec?o=-1\").then(rt).then(function(t){vst.textContent=t})\n"
  " }).catch(function(){vst.textContent=\"microphone refused -- allow it, or use http:/" "/192.168.4.1\"})}\n"
  "function mStop(){if(!mRec)return;mRec=false;\n"
  " try{mNode.disconnect();mGain.disconnect();\n"
  "  mStr.getTracks().forEach(function(t){t.stop()});mCtx.close()}catch(e){}\n"
  " if(!mBuf.length){vst.textContent=\"nothing captured\";return}\n"
  " (function nx(o){if(o>=mBuf.length){\n"
  "   fetch(\"/vrec?o=9999\").then(rt).then(function(t){vst.textContent=t+\" -- press Play\"});return}\n"
  "  var h=\"\";for(var k=o;k<o+256&&k<mBuf.length;k++)h+=(\"0\"+mBuf[k].toString(16)).slice(-2);\n"
  "  vst.textContent=\"uploading \"+Math.round(o*100/mBuf.length)+\"%\";\n"
  "  fetch(\"/vrec?o=\"+o+\"&d=\"+h).then(function(){nx(o+256)})\n"
  "  .catch(function(){vst.textContent=\"upload failed\"})})(0)}\n"
  "/" "*  the little status line for the new cards *" "/\n"
  "function dstat(){fetch(\"/dstat\").then(function(r){return r.json()}).then(function(j){\n"
  " for(var i=0;i<3;i++){var b=document.getElementById(\"dsl\"+i);\n"
  "  b.textContent=\"Slot \"+(i+1)+(j[\"s\"+i]?\" *\":\"\");\n"
  "  b.style.background=(i==dslot)?\"#1d4d7c\":(j[\"s\"+i]?\"#17352a\":\"#1e252d\")}\n"
  " aupb.style.background=j.aup?\"#17352a\":\"#1e252d\";\n"
  " if(j.aupRun)dst.textContent=\"getting up... attempt \"+j.tries;\n"
  " if(j.vlen>0&&!mRec)vst.textContent=(j.vlen/11025).toFixed(1)+\" s recorded\"+\n"
  "  (j.vplay?\" -- playing\":\"\")}).catch(function(){})}\n"
  "setInterval(dstat,2000);dstat();\n"
  "setInterval(poll,1200);poll();buildJoints();loadSlots();\n"
  "</script></body></html>"
  ;


/*====================================================================
 *  SAVED POSITIONS
 *
 *  The sliders at the bottom of the web page drive one servo at a
 *  time through jointSetSafe(), which still honours the rate limiter
 *  so a big slider drag ramps instead of slamming. "Save as new
 *  position" copies wherever the robot is standing right now into one
 *  of NSLOT flash slots, and after that the slot appears as its own
 *  button -- tapping it glides back to those exact angles.
 *===================================================================*/
#define NSLOT       8
#define SLOTNAME   18
int16_t gSlotAng[NSLOT][NJ];
char    gSlotName[NSLOT][SLOTNAME];
bool    gSlotUsed[NSLOT];

void slotsSave() {
  uint32_t used = 0;
  for (int i = 0; i < NSLOT; i++) if (gSlotUsed[i]) used |= (1UL << i);
  prefs.putBytes("slotA", gSlotAng,  sizeof(gSlotAng));
  prefs.putBytes("slotN", gSlotName, sizeof(gSlotName));
  prefs.putUInt ("slotU", used);
}

void slotsLoad() {
  memset(gSlotAng,  0, sizeof(gSlotAng));
  memset(gSlotName, 0, sizeof(gSlotName));
  for (int i = 0; i < NSLOT; i++) gSlotUsed[i] = false;
  if (prefs.getBytes("slotA", gSlotAng, sizeof(gSlotAng)) != sizeof(gSlotAng))
    return;
  prefs.getBytes("slotN", gSlotName, sizeof(gSlotName));
  uint32_t used = prefs.getUInt("slotU", 0);
  for (int i = 0; i < NSLOT; i++) {
    gSlotUsed[i] = (used >> i) & 1UL;
    gSlotName[i][SLOTNAME - 1] = 0;
  }
}

/*====================================================================
 *  FLEX GLOVE -- state, calibration and the tick that applies it
 *===================================================================*/
bool     gFlexOn   = false;          // the ON/OFF switch on the web page
uint32_t gFlexRxMs = 0;              // millis() of the last packet
uint32_t gFlexLogAt = 0;             // throttles the serial trace to ~1 Hz
bool     gFlexLinked = false;        // true only while packets keep coming
/*  Says "link lost" once, when the glove stops talking, and "link is
 *  up" once when it starts again. Called from pumpIO(), costs nothing. */
void flexLinkTick() {
  if (gFlexRxMs == 0) return;
  bool up = (millis() - gFlexRxMs) < FLEX_STALE_MS;
  if (up == gFlexLinked) return;
  gFlexLinked = up;
  Serial.println(up ? F("ESP8266 link is up again")
                    : F("ESP8266 link lost -- waiting for the glove..."));
}
uint32_t gFlexPkts = 0;              // packets received, for the status line
uint8_t  gFlexSide = 0;              // 0 = RIGHT arm, 1 = LEFT arm.
                                     // Set by the switch on the glove.
int16_t  gFlexRaw[NFLEX];            // last raw ADC value per sensor
int16_t  gFlexLo[NFLEX];             // reading with the finger STRAIGHT
int16_t  gFlexHi[NFLEX];             // reading with the finger BENT

/*  PART 1.  The three motors, and one independent set of numbers each.
 *  FLEXNAME is the only place their names are written down, so the web
 *  page, the JSON and the serial monitor can never disagree.          */
const char* const FLEXNAME[NFLEX] = { "Wrist Motor",
                                      "Elbow Motor",
                                      "Inside Elbow Motor" };
uint8_t  gFlexStep[NFLEX];           // speed limiter, degrees per tick
uint8_t  gFlexDead[NFLEX];           // deadband, degrees of mapped angle
int32_t  gFlexSm  [NFLEX];           // smoothed raw value, x256
int16_t  gFlexTgt [NFLEX];           // the angle this motor is aiming at
bool     gFlexHas [NFLEX];           // false until the first reading

/*  One row per side, so the same three fingers can drive either arm.
 *  [0] = right arm, [1] = left arm.                                  */
int8_t   gFlexJ [2][NFLEX];          // which JOINTS[] index
int16_t  gFlexAL[2][NFLEX];          // servo angle with the finger straight
int16_t  gFlexAH[2][NFLEX];          // servo angle with the finger bent

/*  Defaults. The left arm is the mirror of the right, which is why its
 *  two angles run the other way -- lh1 rests at 180 where rh1 rests at
 *  0. All six numbers are editable on the web page.
 *      JOINTS[] indices:  rh1 9, rh2 10, rh3 11
 *                         lh1 1, lh2  2, lh3  3                      */
void flexDefaults() {
  const int8_t  jR[3] = {   9,  10,  11 };     // rh1 rh2 rh3
  const int8_t  jL[3] = {   1,   2,   3 };     // lh1 lh2 lh3
  const int16_t alR[3] = {   0,   0,  20 }, ahR[3] = { 130, 120, 120 };
  const int16_t alL[3] = { 180, 180, 160 }, ahL[3] = {  50,  60,  60 };
  for (int i = 0; i < NFLEX; i++) {
    gFlexRaw[i] = 0;
    gFlexLo[i]  = 250;               // ESP8266 ADC is 0..1023
    gFlexHi[i]  = 750;
    gFlexJ [0][i] = (i < 3) ? jR[i]  : -1;
    gFlexJ [1][i] = (i < 3) ? jL[i]  : -1;
    gFlexAL[0][i] = (i < 3) ? alR[i] : 90;
    gFlexAH[0][i] = (i < 3) ? ahR[i] : 90;
    gFlexAL[1][i] = (i < 3) ? alL[i] : 90;
    gFlexAH[1][i] = (i < 3) ? ahL[i] : 90;
    /*  one independent limiter, deadband, filter and target per motor */
    gFlexStep[i] = FLEX_STEP_DEF;
    gFlexDead[i] = FLEX_DEAD_DEF;
    gFlexSm  [i] = 0;
    gFlexTgt [i] = 90;
    gFlexHas [i] = false;
  }
}

/*  Calibration lives in the same NVS the saved poses use, so it
 *  survives a power cut.                                             */
void flexSave() {
  prefs.putBytes("fxLo",  gFlexLo, sizeof(gFlexLo));
  prefs.putBytes("fxHi",  gFlexHi, sizeof(gFlexHi));
  prefs.putBytes("fxJ2",  gFlexJ,  sizeof(gFlexJ));
  prefs.putBytes("fxAL2", gFlexAL, sizeof(gFlexAL));
  prefs.putBytes("fxAH2", gFlexAH, sizeof(gFlexAH));
  prefs.putBytes("fxSp",  gFlexStep, sizeof(gFlexStep));
  prefs.putBytes("fxDb",  gFlexDead, sizeof(gFlexDead));
  prefs.putUChar("fxOn", gFlexOn ? 1 : 0);
}

void flexLoad() {
  flexDefaults();
  /*  Size-checked, so an old blob from a different sensor count is
   *  ignored rather than half-loaded.                                */
  if (prefs.getBytesLength("fxLo")  == sizeof(gFlexLo) &&
      prefs.getBytesLength("fxAL2") == sizeof(gFlexAL)) {
    prefs.getBytes("fxLo",  gFlexLo, sizeof(gFlexLo));
    prefs.getBytes("fxHi",  gFlexHi, sizeof(gFlexHi));
    prefs.getBytes("fxJ2",  gFlexJ,  sizeof(gFlexJ));
    prefs.getBytes("fxAL2", gFlexAL, sizeof(gFlexAL));
    prefs.getBytes("fxAH2", gFlexAH, sizeof(gFlexAH));
    Serial.println(F("flex calibration restored"));
  }
  /*  The per-motor speed limiter and deadband, if they were ever saved.
   *  Each one is range-checked on the way in, so a blob from an older
   *  build can never leave a motor with a silly limit.                */
  if (prefs.getBytesLength("fxSp") == sizeof(gFlexStep))
    prefs.getBytes("fxSp", gFlexStep, sizeof(gFlexStep));
  if (prefs.getBytesLength("fxDb") == sizeof(gFlexDead))
    prefs.getBytes("fxDb", gFlexDead, sizeof(gFlexDead));
  for (int i = 0; i < NFLEX; i++) {
    if (gFlexStep[i] < 1 || gFlexStep[i] > FLEX_STEP_MAX)
      gFlexStep[i] = FLEX_STEP_DEF;
    if (gFlexDead[i] > FLEX_DEAD_MAX) gFlexDead[i] = FLEX_DEAD_DEF;
  }
  gFlexOn = prefs.getUChar("fxOn", 0) ? true : false;
}

bool flexLive() { return gFlexPkts && (millis() - gFlexRxMs < FLEX_STALE_MS); }

const char* flexSideName() { return gFlexSide ? "LEFT arm" : "RIGHT arm"; }

/*  Called from pumpIO(), which means it also runs while a movement is
 *  in progress -- and the first line is what makes this an extra layer
 *  instead of a mode: the movement owns the servos, so we stand aside.
 *  No delay(), no waiting for the glove, nothing that can stall.     */
void flexTick() {
  if (!gFlexOn || gBusy || gFreeze) return;
  if (!flexLive()) return;
  static uint32_t next = 0;
  uint32_t now = millis();
  if ((int32_t)(now - next) < 0) return;
  next = now + FLEX_APPLY_MS;

  uint8_t sd = gFlexSide ? 1 : 0;
  for (int i = 0; i < NFLEX; i++) {
    int j = gFlexJ[sd][i];
    if (j < 0 || j >= NJ) continue;              // sensor switched off
    int lo = gFlexLo[i], hi = gFlexHi[i];
    if (hi == lo) continue;                      // not calibrated yet

    /*  ---- 1. this motor's own smoothing -------------------------
     *  Its own accumulator, so a spike on one sensor -- and the mux
     *  on the glove does produce the odd one -- can never travel into
     *  the other two motors.                                        */
    int32_t sm = gFlexSm[i];
    int32_t in = (int32_t)gFlexRaw[i] * 256;
    if (sm == 0) sm = in;
    sm += (in - sm) / (int32_t)(FLEX_SMOOTH < 1 ? 1 : FLEX_SMOOTH);
    gFlexSm[i] = sm;
    int rawS = (int)(sm / 256);

    /*  ---- 2. this motor's own two angles ------------------------ */
    int a0 = gFlexAL[sd][i], a1 = gFlexAH[sd][i];
    long a = (long)(rawS - lo) * (a1 - a0) / (hi - lo) + a0;
    int amin = a0 < a1 ? a0 : a1;
    int amax = a0 < a1 ? a1 : a0;
    if (a < amin) a = amin;
    if (a > amax) a = amax;

    /*  ---- 3. this motor's own DEADBAND, with real HYSTERESIS -----
     *  Until the finger asks for more than gFlexDead[i] degrees of
     *  change, the committed target is not touched at all. That is
     *  the separation you asked for: a motor you are not working
     *  stays exactly where it is, to the degree, while you move the
     *  other two.
     *
     *  ANTI-JITTER: when it DOES move, the target is left at the EDGE
     *  of the deadband rather than snapped onto the raw value. That is
     *  the difference between a deadband and hysteresis, and it matters:
     *  snapping onto the raw value put the next noisy sample straight
     *  back outside the band, so the motor crept back and forth for
     *  ever. Trailing by the deadband means noise produces exactly zero
     *  further movement until the finger genuinely moves again.       */
    if (!gFlexHas[i]) { gFlexHas[i] = true; gFlexTgt[i] = (int16_t)a; }
    else {
      int dead = (int)gFlexDead[i];
      int diff = (int)a - (int)gFlexTgt[i];
      if      (diff >  dead) gFlexTgt[i] = (int16_t)(a - dead);
      else if (diff < -dead) gFlexTgt[i] = (int16_t)(a + dead);
    }

    /*  ---- 4. this motor's own SPEED LIMITER --------------------
     *  Degrees per tick, set by this motor's slider on the web page.
     *  Small steps are what keep an MG996R quiet, and they are the
     *  same reason the rest of the sketch rate-limits every write.  */
    int cur = JOINTS[j].last, d = (int)gFlexTgt[i] - cur;
    if (d == 0) continue;
    int mx = (int)gFlexStep[i];
    if (mx < 1)              mx = 1;
    if (mx > FLEX_STEP_MAX)  mx = FLEX_STEP_MAX;
    if (d >  mx) d =  mx;
    if (d < -mx) d = -mx;
    JOINTS[j].writeNow(cur + d);
  }
}

/*  Move ONE joint, ramped. Deliberately does not call pumpIO(): this
 *  runs inside a web handler, and pumping the server from in here
 *  would re-enter the handler.                                       */
void jointSetSafe(int j, int a) {
  a = constrain(a, 0, 180);
  int from = JOINTS[j].last, diff = a - from;
  int n = (abs(diff) + MAX_STEP_DEG - 1) / MAX_STEP_DEG;
  if (n < 1) n = 1;
  for (int k = 1; k <= n; k++) {
    JOINTS[j].writeNow(from + diff * k / n);
    if (k < n) delay(SUBSTEP_MS);
  }
}

/*====================================================================
 *  PART 2.  MANUAL SERVO CONTROL -- the master switch for the servo
 *  angle sliders at the bottom of the page.
 *
 *  When it is OFF the lock is applied in THREE places, because a
 *  browser is not something to trust with a robot's arms:
 *
 *    1. every slider gets the HTML  disabled  attribute,
 *    2. the whole block gets  pointer-events:none , so a finger
 *       dragging across the screen cannot even grab a thumb,
 *    3. /joint refuses to move anything and says so.
 *
 *  So a stray touch, a phone in a pocket, an old browser tab left open
 *  or a bookmarked /joint URL cannot move a servo or change its
 *  position. Switching it back ON re-reads the robot's real angles
 *  first, so no stale slider value is ever pushed out, and then the
 *  sliders work exactly as they always did.
 *
 *  Nothing else is affected: the motor-speed slider, the volume
 *  slider, the three flex-motor sliders, the saved positions and
 *  every button carry on working with the switch either way.
 *  The setting is remembered in flash.
 *===================================================================*/
bool gManualOn = true;
void manualSave() { prefs.putUChar("manOn", gManualOn ? 1 : 0); }
void manualLoad() { gManualOn = prefs.getUChar("manOn", 1) ? true : false; }

void handleAngles() {                 // build the sliders / read them back
  String s = "{\"n\":[";
  for (int i = 0; i < NJ; i++) {
    s += "\""; s += JOINTS[i].name; s += "\"";
    if (i < NJ - 1) s += ",";
  }
  s += "],\"a\":[";
  for (int i = 0; i < NJ; i++) {
    s += String((int)JOINTS[i].last);
    if (i < NJ - 1) s += ",";
  }
  s += "],\"man\":";
  s += gManualOn ? 1 : 0;
  s += "}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", s);
}

/*  The master switch itself. No argument = just report the state.    */
void handleManual() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("v")) {
    bool want = server.arg("v").toInt() ? true : false;
    if (want != gManualOn) {
      gManualOn = want;
      manualSave();
      Serial.printf("Manual Servo Control %s\n",
                    gManualOn ? "ON -- the sliders drive the servos"
                              : "OFF -- the sliders are locked");
    }
  }
  server.send(200, "text/plain",
              gManualOn ? "Manual Servo Control ON -- sliders active"
                        : "Manual Servo Control OFF -- sliders locked");
}

void handleJoint() {                  // one slider moved
  server.sendHeader("Access-Control-Allow-Origin", "*");
  /*  The master switch, enforced on the robot itself. This is the
   *  layer that makes the lock real rather than cosmetic.            */
  if (!gManualOn) {
    server.send(200, "text/plain",
                "Manual Servo Control is OFF -- nothing moved");
    return;
  }
  if (gBusy) { server.send(200, "text/plain", "busy"); return; }
  int j = server.arg("j").toInt();
  int a = server.arg("a").toInt();
  if (j < 0 || j >= NJ) { server.send(400, "text/plain", "bad joint"); return; }
  jointSetSafe(j, a);
  gPosUnknown = true;                 // no longer in a named pose
  server.send(200, "text/plain", "ok");
}

void handlePlist() {
  String s = "[";
  for (int i = 0; i < NSLOT; i++) {
    if (i) s += ",";
    s += "{\"i\":"; s += String(i);
    s += ",\"used\":"; s += (gSlotUsed[i] ? "1" : "0");
    s += ",\"name\":\""; s += (gSlotUsed[i] ? gSlotName[i] : ""); s += "\"}";
  }
  s += "]";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", s);
}

void handlePsave() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  int i = server.arg("i").toInt();
  if (i < 0 || i >= NSLOT) { server.send(400, "text/plain", "bad slot"); return; }
  String nm = server.hasArg("n") ? server.arg("n") : String("");
  if (nm.length() == 0) nm = String("position ") + String(i + 1);
  for (int k = 0; k < NJ; k++) gSlotAng[i][k] = JOINTS[k].last;
  nm.toCharArray(gSlotName[i], SLOTNAME);
  gSlotName[i][SLOTNAME - 1] = 0;
  gSlotUsed[i] = true;
  slotsSave();
  Serial.printf("saved slot %d as \"%s\"\n", i + 1, gSlotName[i]);
  server.send(200, "text/plain", "saved");
}

void handlePdel() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  int i = server.arg("i").toInt();
  if (i >= 0 && i < NSLOT) { gSlotUsed[i] = false; slotsSave(); }
  server.send(200, "text/plain", "ok");
}

/*====================================================================
 *  FLEX GLOVE -- web endpoints
 *===================================================================*/
/*  The glove calls this. Reply is one character so the glove can light
 *  its LED: "1" = flex control is ON, "0" = OFF (still connected).   */
/*====================================================================
 *  AUTO STAND-UP / RECOVERY
 *  Runs only on a genuine fall: the tilt has to be real (gFallen, which
 *  already needs FALL_DEG held for FALL_HOLD_MS) AND still there a
 *  moment later. Then it asks for the existing "stand up" routine
 *  through the normal queue, so there is no duplicated motion code and
 *  a manual STOP still wins. Three attempts, then it stays down and
 *  says so rather than thrashing the servos.
 *===================================================================*/
void autoUpTick() {
#if USE_MPU
  if (!gAutoUp || !gMpuOk) return;
  uint32_t now = millis();

  if (gAutoUpRun) {                       /* a recovery is running */
    if (gBusy) return;                    /* let the routine finish */
    bool level = (fabsf(gPitch - gPitchZero) < (FALL_DEG / 2) &&
                  fabsf(gRoll  - gRollZero)  < (FALL_DEG / 2));
    if (level) {                          /* up again: stop, and stay stopped */
      gAutoUpRun = false;
      gAupTries  = 0;
      gFallen    = false;
      balReset();                         /* drop any trim it had wound up */
      faceSet(F_PROUD, "UP AGAIN");
      soundPlay(S_WIN);
      Serial.println(F("auto stand-up: recovered."));
      return;
    }
    if (now - gAupAt < AUP_GAP_MS) return;
    gAutoUpRun = false;                   /* fall through and try again */
    return;
  }

  if (!gFallen) { gAupTries = 0; return; }
  if (now - gAupAt < AUP_GAP_MS) return;
  if (gBusy) return;
  if (gAupTries >= AUP_TRIES) {
    if (gAupTries == AUP_TRIES) {
      gAupTries++;
      faceSet(F_SAD, "HELP ME");
      soundPlay(S_ERROR);
      Serial.println(F("auto stand-up: gave up after 3 tries -- pick me up."));
    }
    return;
  }

  gAupTries++;
  gAupAt     = now;
  gAutoUpRun = true;
  gAbort     = false;
  balReset();
  faceSet(F_DETERMINED, "GET UP");
  soundPlay(S_GRUNT);
  Serial.printf("auto stand-up: attempt %d\n", (int)gAupTries);
  queueCommand("stand up");
#endif
}

/*  P4 / P10.  A packet only counts as the glove if it carries NFLEX
 *  numbers and every one of them is inside the ADC's range. A stray
 *  browser request, a half-finished packet or a rebooting glove is
 *  rejected and nothing is reported as connected.                    */
void handleFlex() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String v = server.hasArg("v") ? server.arg("v") : String("");
  int16_t tmp[NFLEX];
  int got = 0, from = 0;
  while (got < NFLEX && from <= (int)v.length()) {
    int c = v.indexOf(',', from);
    String t = (c < 0) ? v.substring(from) : v.substring(from, c);
    t.trim();
    if (t.length()) {
      long q = t.toInt();
      if (q < 0 || q > 4095) {
        Serial.println(F("ignored a /flex packet -- value out of range"));
        server.send(400, "text/plain", "bad value");
        return;
      }
      tmp[got++] = (int16_t)q;
    }
    if (c < 0) break;
    from = c + 1;
  }
  if (got < NFLEX) {
    Serial.printf("ignored a /flex packet -- %d of %d values\n", got, NFLEX);
    server.send(400, "text/plain", "need three values");
    return;
  }
  uint32_t now = millis();
  bool firstAgain = (gFlexRxMs == 0) || (now - gFlexRxMs > FLEX_STALE_MS);
  /*  s=0 right arm, s=1 left arm -- the switch on the glove.
   *  D2 DIAGNOSTIC: say so, loudly, the moment it changes. If you press
   *  D2 on the glove and this line never appears here, the problem is on
   *  the glove side, not the robot side -- and the glove prints its own
   *  line at the same moment, so the two together tell you exactly which
   *  half of the link to look at.                                     */
  if (server.hasArg("s")) {
    uint8_t ns = server.arg("s").toInt() ? 1 : 0;
    if (ns != gFlexSide) {
      gFlexSide = ns;
      Serial.printf("\n*** GLOVE SWITCHED ARMS -> now driving the %s ***\n",
                    flexSideName());
      Serial.printf("    %s -> %s,  %s -> %s,  %s -> %s\n",
        FLEXNAME[0], (gFlexJ[ns][0] >= 0 && gFlexJ[ns][0] < NJ)
                       ? JOINTS[gFlexJ[ns][0]].name : "none",
        FLEXNAME[1], (gFlexJ[ns][1] >= 0 && gFlexJ[ns][1] < NJ)
                       ? JOINTS[gFlexJ[ns][1]].name : "none",
        FLEXNAME[2], (gFlexJ[ns][2] >= 0 && gFlexJ[ns][2] < NJ)
                       ? JOINTS[gFlexJ[ns][2]].name : "none");
      /*  Forget the old arm's committed targets so the new arm starts
       *  from the finger's real position instead of stepping there from
       *  whatever the other arm happened to be holding.               */
      for (int i = 0; i < NFLEX; i++) gFlexHas[i] = false;
      faceSet(gFlexSide ? F_WINK : F_HAPPY, gFlexSide ? "LEFT" : "RIGHT");
    }
  }
  for (int i = 0; i < NFLEX; i++) gFlexRaw[i] = tmp[i];
  gFlexRxMs = now;
  gFlexPkts++;
  if (firstAgain) {
    Serial.println(F("ESP8266 connected"));
    gFlexLinked = true;
    gFlexLogAt  = 0;
  }
  if (gFlexLogAt == 0 || (int32_t)(now - gFlexLogAt) >= 0) {
    gFlexLogAt = now + 1000;         /* one report a second, not 20 */
    Serial.println(F("Flex data received"));
    Serial.printf("Flex1: %03d\nFlex2: %03d\nFlex3: %03d\n",
                  (int)gFlexRaw[0], (int)gFlexRaw[1], (int)gFlexRaw[2]);
    Serial.printf("  arm: %s   flex control: %s\n",
                  gFlexSide ? "LEFT" : "RIGHT", gFlexOn ? "ON" : "OFF");
  }
  server.send(200, "text/plain", gFlexOn ? "1" : "0");
}

void handleFon() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  gFlexOn = true;  flexSave();
  server.send(200, "text/plain", "flex control ON");
}
void handleFoff() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  gFlexOn = false; flexSave();
  server.send(200, "text/plain", "flex control OFF");
}

void handleFstat() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  uint8_t sd = gFlexSide ? 1 : 0;
  String s = "{\"on\":"; s += gFlexOn ? 1 : 0;
  s += ",\"link\":";    s += flexLive() ? 1 : 0;
  s += ",\"pkts\":";    s += gFlexPkts;
  s += ",\"busy\":";    s += gBusy ? 1 : 0;
  s += ",\"side\":";    s += sd;
  s += ",\"spmax\":";   s += FLEX_STEP_MAX;
  s += ",\"dbmax\":";   s += FLEX_DEAD_MAX;
  s += ",\"s\":[";
  for (int i = 0; i < NFLEX; i++) {
    if (i) s += ",";
    int jr = gFlexJ[0][i], jl = gFlexJ[1][i];
    s += "{\"raw\":"; s += gFlexRaw[i];
    s += ",\"lo\":";  s += gFlexLo[i];
    s += ",\"hi\":";  s += gFlexHi[i];
    s += ",\"nm\":\""; s += FLEXNAME[i]; s += "\"";
    s += ",\"sp\":";  s += (int)gFlexStep[i];
    s += ",\"db\":";  s += (int)gFlexDead[i];
    s += ",\"tg\":";  s += (int)gFlexTgt[i];
    s += ",\"nr\":\""; s += (jr >= 0 && jr < NJ) ? JOINTS[jr].name : "off"; s += "\"";
    s += ",\"nl\":\""; s += (jl >= 0 && jl < NJ) ? JOINTS[jl].name : "off"; s += "\"";
    s += ",\"alr\":"; s += gFlexAL[0][i];
    s += ",\"ahr\":"; s += gFlexAH[0][i];
    s += ",\"all\":"; s += gFlexAL[1][i];
    s += ",\"ahl\":"; s += gFlexAH[1][i];
    s += "}";
  }
  s += "]}";
  server.send(200, "application/json", s);
}

/*  Capture the CURRENT reading as straight (w=lo) or bent (w=hi).
 *  One calibration per finger -- it is the same finger whichever arm
 *  the switch has selected, so this does not need doing twice.       */
void handleFcal() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  int i = server.arg("i").toInt();
  if (i < 0 || i >= NFLEX) { server.send(400, "text/plain", "bad sensor"); return; }
  if (!flexLive()) { server.send(400, "text/plain", "glove not connected"); return; }
  if (server.arg("w") == "hi") gFlexHi[i] = gFlexRaw[i];
  else                         gFlexLo[i] = gFlexRaw[i];
  flexSave();
  server.send(200, "text/plain", String("Flex ") + (i + 1) + " " +
                                 server.arg("w") + " = " + gFlexRaw[i]);
}

/*  The two servo angles a finger maps to, per side.
 *  side=0 right arm, side=1 left arm.                                */
void handleFset() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  int i  = server.arg("i").toInt();
  int sd = server.arg("side").toInt() ? 1 : 0;
  if (i < 0 || i >= NFLEX) { server.send(400, "text/plain", "bad sensor"); return; }
  if (server.hasArg("al")) gFlexAL[sd][i] = (int16_t)constrain(server.arg("al").toInt(), 0, 180);
  if (server.hasArg("ah")) gFlexAH[sd][i] = (int16_t)constrain(server.arg("ah").toInt(), 0, 180);
  flexSave();
  server.send(200, "text/plain", String(sd ? "left" : "right") + " arm mapping saved");
}

/*  PART 1.  One motor's speed limiter and deadband.  ONE motor per
 *  call -- i is 0 Wrist Motor, 1 Elbow Motor, 2 Inside Elbow Motor --
 *  so moving one slider writes one array element and cannot possibly
 *  disturb the position or the settings of the other two.            */
void handleFlim() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  int i = server.arg("i").toInt();
  if (i < 0 || i >= NFLEX) { server.send(400, "text/plain", "bad motor"); return; }
  if (server.hasArg("sp"))
    gFlexStep[i] = (uint8_t)constrain(server.arg("sp").toInt(), 1, FLEX_STEP_MAX);
  if (server.hasArg("db"))
    gFlexDead[i] = (uint8_t)constrain(server.arg("db").toInt(), 0, FLEX_DEAD_MAX);
  flexSave();
  String m = String(FLEXNAME[i]) + ": speed limit " + (int)gFlexStep[i] +
             " (" + (int)((long)gFlexStep[i] * 1000 / FLEX_APPLY_MS) +
             " deg/s), deadband " + (int)gFlexDead[i] + " deg";
  Serial.println(m);
  server.send(200, "text/plain", m);
}

/*====================================================================
 *  P7. FILE STORE -- as many pictures and sounds as the flash holds
 *
 *  LittleFS uses the spare flash inside the ESP32 (usually a little
 *  over a megabyte with the default partition table). There is NO
 *  fixed limit on the number of files: you keep storing until the free
 *  space runs out, and the dashboard shows you exactly how much is
 *  left. A picture is 1024 bytes, so a spare megabyte is roughly a
 *  thousand faces. A sound is one byte per sample at 11025 Hz, about
 *  11 kB a second.
 *      /i_<name>.bin   a 1024-byte OLED picture
 *      /a_<name>.raw   8-bit unsigned mono sound, 11025 Hz
 *  Everything survives a restart, everything can be listed, shown or
 *  played, and each file has its own Delete button.
 *===================================================================*/
bool gFsOk = false;

void fsStart() {
  gFsOk = LittleFS.begin(true);      /* true = format it if it is new */
  if (!gFsOk) {
    Serial.println(F("LittleFS would not mount -- pictures and sounds cannot be stored"));
    return;
  }
  Serial.printf("file store ready: %u bytes free of %u\n",
                (unsigned)(LittleFS.totalBytes() - LittleFS.usedBytes()),
                (unsigned)LittleFS.totalBytes());
}
uint32_t fsFree() {
  if (!gFsOk) return 0;
  size_t t = LittleFS.totalBytes(), u = LittleFS.usedBytes();
  return (t > u) ? (uint32_t)(t - u) : 0;
}
/*  Names are filtered down to letters, digits, dash and underscore, so
 *  nothing typed on the phone can escape the folder or upset the file
 *  system.                                                           */
static String fsSafe(const String& n) {
  String o;
  for (unsigned i = 0; i < n.length() && o.length() < 20; i++) {
    char c = n.charAt(i);
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') || c == '-' || c == '_') o += c;
    else if (c == ' ' || c == '.') o += '_';
  }
  if (!o.length()) o = "untitled";
  return o;
}
static String fsPath(bool audio, const String& safe) {
  return audio ? (String("/a_") + safe + ".raw")
               : (String("/i_") + safe + ".bin");
}
static bool fsIsAudio() {
  return server.hasArg("k") && server.arg("k") == "a";
}

/*  The list, with the free space, as JSON for the dashboard. */
void handleFsl() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (!gFsOk) { server.send(200, "application/json", "{\"ok\":0,\"f\":[]}"); return; }
  String j = "{\"ok\":1,\"free\":";
  j += String((unsigned long)fsFree());
  j += ",\"total\":";
  j += String((unsigned long)LittleFS.totalBytes());
  j += ",\"f\":[";
  File root = LittleFS.open("/");
  bool first = true;
  if (root) {
    File f = root.openNextFile();
    while (f) {
      String n = String(f.name());
      if (n.startsWith("/")) n = n.substring(1);
      int dot = n.lastIndexOf('.');
      if ((n.startsWith("i_") || n.startsWith("a_")) && dot > 2) {
        if (!first) j += ",";
        first = false;
        j += "{\"k\":\"";
        j += (n.charAt(0) == 'a') ? "a" : "i";
        j += "\",\"n\":\"";
        j += n.substring(2, dot);
        j += "\",\"s\":";
        j += String((unsigned long)f.size());
        j += "}";
      }
      f = root.openNextFile();
    }
  }
  j += "]}";
  server.send(200, "application/json", j);
}

/*  Store what is in memory right now: the last picture the phone sent,
 *  or the last thing you recorded.                                   */
void handleFsave() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (!gFsOk) { server.send(500, "text/plain", "no file store"); return; }
  bool audio = fsIsAudio();
  String safe = fsSafe(server.hasArg("n") ? server.arg("n") : String(""));
  String path = fsPath(audio, safe);
  uint32_t need = audio ? gVLen : 1024;
  if (audio && (!gVBuf || gVLen == 0)) {
    server.send(400, "text/plain", "record something first");
    return;
  }
  if (need + 4096 > fsFree() && !LittleFS.exists(path)) {
    server.send(507, "text/plain", "the flash is full -- delete something first");
    return;
  }
  File f = LittleFS.open(path, "w");
  if (!f) { server.send(500, "text/plain", "could not open the file"); return; }
  size_t wrote = audio ? f.write(gVBuf, gVLen) : f.write(gOledBmp, 1024);
  f.close();
  if (wrote != need) {
    LittleFS.remove(path);
    server.send(507, "text/plain", "the flash ran out while writing");
    return;
  }
  char m[96];
  snprintf(m, sizeof(m), "stored \"%s\" (%u bytes), %u kB still free",
           safe.c_str(), (unsigned)wrote, (unsigned)(fsFree() / 1024));
  Serial.println(m);
  server.send(200, "text/plain", m);
}

/*  Show a stored picture on the OLED, or load a stored sound into the
 *  clip buffer and play it.                                          */
void handleFload() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (!gFsOk) { server.send(500, "text/plain", "no file store"); return; }
  bool audio = fsIsAudio();
  String safe = fsSafe(server.hasArg("n") ? server.arg("n") : String(""));
  String path = fsPath(audio, safe);
  File f = LittleFS.open(path, "r");
  if (!f) { server.send(404, "text/plain", "no such file"); return; }
  if (!audio) {
    size_t got = f.read(gOledBmp, 1024);
    f.close();
    if (got != 1024) { server.send(500, "text/plain", "that picture is damaged"); return; }
    gBmpHave  = true;
    gSlideOn  = false;
    gTxOver   = false;
    gOledMode = 2;
    server.send(200, "text/plain", String("showing ") + safe);
    return;
  }
  uint32_t n = (uint32_t)f.size();
  if (n > VREC_MAX) n = VREC_MAX;
  if (!vAlloc(n)) { f.close(); server.send(500, "text/plain", "not enough memory"); return; }
  gVOn  = false;
  gVLen = (uint32_t)f.read(gVBuf, n);
  f.close();
  gVPos = 0;
  gVOn  = gVLen > 0;
  server.send(200, "text/plain", String("playing ") + safe);
}

/*  One file at a time, on purpose -- nothing here can wipe the lot. */
void handleFdel() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (!gFsOk) { server.send(500, "text/plain", "no file store"); return; }
  bool audio = fsIsAudio();
  String safe = fsSafe(server.hasArg("n") ? server.arg("n") : String(""));
  String path = fsPath(audio, safe);
  if (!LittleFS.exists(path)) { server.send(404, "text/plain", "no such file"); return; }
  LittleFS.remove(path);
  server.send(200, "text/plain", String("deleted ") + safe);
}

/*====================================================================
 *  OLED DRAWING / TEXT / VOICE / AUTO STAND-UP  handlers
 *  The phone sends the 1024-byte frame buffer as hex in 256-byte
 *  chunks (?o=offset&d=hex) and then ?o=9999 to show it. Small chunks
 *  keep every URL well inside the web server's limit.
 *===================================================================*/
static uint8_t hexb(const char* h) {
  uint8_t v = 0;
  for (int k = 0; k < 2; k++) {
    char c = h[k]; v <<= 4;
    if (c >= '0' && c <= '9') v |= (uint8_t)(c - '0');
    else if (c >= 'a' && c <= 'f') v |= (uint8_t)(c - 'a' + 10);
    else if (c >= 'A' && c <= 'F') v |= (uint8_t)(c - 'A' + 10);
  }
  return v;
}

void handleOdraw() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  long o = server.hasArg("o") ? server.arg("o").toInt() : 0;
  if (o == 9999) {
    gBmpHave = true; gSlideOn = false; gOledMode = 2;
    server.send(200, "text/plain", "drawing is on the OLED");
    return;
  }
  String d = server.hasArg("d") ? server.arg("d") : String("");
  const char* c = d.c_str();
  int cnt = (int)(d.length() / 2);
  for (int i = 0; i < cnt; i++) {
    long idx = o + i;
    if (idx < 0 || idx >= 1024) break;
    gOledBmp[idx] = hexb(c + i * 2);
  }
  server.send(200, "text/plain", "ok");
}

/*  Shared by both text routes: t = the words, x / y = the top-left
 *  corner in OLED pixels, s = size 1..4, o = 1 to sit over the drawing,
 *  d = scroll direction, v = scroll speed in pixels per second. Every
 *  argument is optional, so an old link like /otext?t=hi still behaves
 *  exactly as it always did.                                         */
static void textArgs() {
  if (server.hasArg("x")) gTxX = (int16_t)server.arg("x").toInt();
  if (server.hasArg("y")) gTxY = (int16_t)server.arg("y").toInt();
  if (server.hasArg("s")) {
    int v = server.arg("s").toInt();
    gTxSc = (uint8_t)(v < 1 ? 1 : (v > 4 ? 4 : v));
  }
  if (server.hasArg("o")) gTxOver = server.arg("o").toInt() ? true : false;
  if (server.hasArg("d")) gScrDir = server.arg("d").toInt() < 0 ? -1 : 1;
  if (server.hasArg("v")) {
    int v = server.arg("v").toInt();
    gScrSpd = (uint8_t)(v < 5 ? 5 : (v > 200 ? 200 : v));
  }
  if (gTxX < -160) gTxX = -160;
  if (gTxX >  127) gTxX =  127;
  if (gTxY <    0) gTxY =    0;
  if (gTxY >   63) gTxY =   63;
}
void handleOtext() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String t = server.hasArg("t") ? server.arg("t") : String("");
  t.toCharArray(gOledText, sizeof(gOledText));
  gOledText[sizeof(gOledText) - 1] = 0;
  textArgs();
  gSlideOn = false;
  gOledMode = strlen(gOledText) ? 1 : 0;
  char m[80];
  snprintf(m, sizeof(m), "text at x %d y %d, size %d%s",
           (int)gTxX, (int)gTxY, (int)gTxSc,
           gTxOver ? ", over your drawing" : "");
  server.send(200, "text/plain", m);
}

void handleOban() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String t = server.hasArg("t") ? server.arg("t") : String("");
  t.toCharArray(gOledText, sizeof(gOledText));
  gOledText[sizeof(gOledText) - 1] = 0;
  textArgs();
  gSlideOn = false;
  gOledMode = strlen(gOledText) ? 3 : 0;
  char m[80];
  snprintf(m, sizeof(m), "scrolling %s at %d px/s, size %d",
           gScrDir < 0 ? "right to left" : "left to right",
           (int)gScrSpd, (int)gTxSc);
  server.send(200, "text/plain", m);
}

void handleOface() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  gOledMode = 0; gSlideOn = false;
  faceSet(F_HAPPY, "");
  server.send(200, "text/plain", "faces are back");
}

void handleDsave() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  int sl = server.hasArg("s") ? server.arg("s").toInt() : 0;
  if (sl < 0 || sl >= NDRAW) { server.send(400, "text/plain", "bad slot"); return; }
  bool ok = drawSave(sl);
  server.send(200, "text/plain", ok ? "saved permanently -- survives power off"
                                    : "flash write failed");
}
void handleDload() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  int sl = server.hasArg("s") ? server.arg("s").toInt() : 0;
  if (sl < 0 || sl >= NDRAW) { server.send(400, "text/plain", "bad slot"); return; }
  if (!drawLoad(sl)) { server.send(200, "text/plain", "that slot is empty"); return; }
  gSlideOn = false; gOledMode = 2;
  server.send(200, "text/plain", "loaded from flash");
}
void handleDdel() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  int sl = server.hasArg("s") ? server.arg("s").toInt() : 0;
  if (sl < 0 || sl >= NDRAW) { server.send(400, "text/plain", "bad slot"); return; }
  drawDel(sl);
  server.send(200, "text/plain", "deleted");
}
void handleDslide() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  int v = server.hasArg("v") ? server.arg("v").toInt() : 1;
  gSlideOn = v ? true : false;
  gSlideAt = 0;
  if (!gSlideOn) { gOledMode = 0; }
  server.send(200, "text/plain", gSlideOn ? "slideshow on" : "slideshow off");
}

/*  One small JSON for everything the new cards need to show. */
void handleDstat() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  char buf[200];
  snprintf(buf, sizeof(buf),
           "{\"mode\":%d,\"slide\":%d,\"s0\":%d,\"s1\":%d,\"s2\":%d,"
           "\"aup\":%d,\"aupRun\":%d,\"tries\":%d,\"vlen\":%u,\"vplay\":%d}",
           (int)gOledMode, gSlideOn ? 1 : 0,
           drawHas(0) ? 1 : 0, drawHas(1) ? 1 : 0, drawHas(2) ? 1 : 0,
           gAutoUp ? 1 : 0, gAutoUpRun ? 1 : 0, (int)gAupTries,
           (unsigned)gVLen, gVOn ? 1 : 0);
  server.send(200, "application/json", buf);
}

void handleAup() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  int v = server.hasArg("v") ? server.arg("v").toInt() : 1;
  gAutoUp = v ? true : false;
  gAupTries = 0;
  gAupAt = 0;
  if (!gAutoUp) gAutoUpRun = false;
  server.send(200, "text/plain", gAutoUp ? "auto stand-up armed"
                                         : "auto stand-up off");
}
/*  Force one recovery now, whatever the sensor thinks. */
void handleAupNow() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  gAbort = false;
  gAupTries = 0;
  gAupAt = 0;
  balReset();
  faceSet(F_DETERMINED, "GET UP");
  queueCommand("stand up");
  server.send(200, "text/plain", "standing up");
}

/* ---- recorded voice: the phone's microphone does the capture ------- */
void handleVrec() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  long o = server.hasArg("o") ? server.arg("o").toInt() : 0;
  if (o == -1) {                       /* start */
    gVOn = false;
    if (!vAlloc(VREC_MAX)) { server.send(500, "text/plain", "no memory"); return; }
    gVLen = 0;
    server.send(200, "text/plain", "recording");
    return;
  }
  if (o == 9999) {                     /* finished */
    server.send(200, "text/plain", String("stored ") + String((unsigned)gVLen) + " bytes");
    return;
  }
  if (!gVBuf) { server.send(500, "text/plain", "not started"); return; }
  String d = server.hasArg("d") ? server.arg("d") : String("");
  const char* c = d.c_str();
  int cnt = (int)(d.length() / 2);
  for (int i = 0; i < cnt; i++) {
    uint32_t idx = (uint32_t)o + (uint32_t)i;
    if (idx >= gVCap) break;
    gVBuf[idx] = hexb(c + i * 2);
    if (idx + 1 > gVLen) gVLen = idx + 1;
  }
  server.send(200, "text/plain", "ok");
}
void handleVplay() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (!gVBuf || !gVLen) { server.send(200, "text/plain", "nothing recorded"); return; }
  soundStop();
  gVPos = 0; gVOn = true;
  faceSet(F_MUSIC, "");
  server.send(200, "text/plain", "playing your recording");
}
void handleVstop() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  gVOn = false;
  server.send(200, "text/plain", "stopped");
}
void handleVsave() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  vSave();
  server.send(200, "text/plain", "recording kept in flash");
}
void handleVclear() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  vClear();
  server.send(200, "text/plain", "recording deleted");
}

void handleRoot()   {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send_P(200, "text/html", PAGE);
}
void handleRun() {
  String c = server.hasArg("c") ? server.arg("c") : String("");
  queueCommand(c);
  /*  Allow voice_remote.html (opened from a local file, so a different
   *  origin) to POST commands here.                                   */
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "ok");
}
void handleStatus() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  char buf[420];
  snprintf(buf, sizeof(buf),
           "{\"busy\":%s,\"cmd\":\"%s\",\"state\":\"%s\",\"speed\":%u,"
           "\"mpu\":%d,\"pitch\":%.0f,\"roll\":%.0f,\"bal\":%d,\"fallen\":%d,"
           "\"oled\":%d,\"steps\":%d,\"stance\":%d,"
           "\"audio\":%d,\"vol\":%d,\"mute\":%d,\"snd\":\"%s\","
           "\"head\":\"%s\",\"face\":%d}",
           gBusy ? "true" : "false", gRunning.c_str(), whereAmIsafe(),
           (unsigned)(gSpeedOverride > 0 ? gSpeedOverride : gTimePct),
           gMpuOk ? 1 : 0, gPitch, gRoll, gBalOn ? 1 : 0, gFallen ? 1 : 0,
           gOledOk ? 1 : 0, (int)gWalkHalf, (int)gStance,
           gAudioOk ? 1 : 0, (int)gVol, gMute ? 1 : 0,
           SND_NAMES[gSndNow < S_COUNT ? gSndNow : 0],
           HG_NAMES[gHG < HG_COUNT ? gHG : 0], (int)gFaceWant);
  server.send(200, "application/json", buf);
}
#endif  /* USE_WEB */

#if NEED_WIFI
void wifiInit() {
  if (strlen(HOME_SSID) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(HOME_SSID, HOME_PASS);
    Serial.print(F("joining "));
    Serial.print(HOME_SSID);
    for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print(F("open  http://"));
      Serial.println(WiFi.localIP());
#if WIFI_LOW_POWER
      WiFi.setTxPower(WIFI_POWER_11dBm);
#endif
      return;
    }
    Serial.println(F("could not join -- making my own hotspot instead"));
  }
  /*==================================================================
   *  THE HOTSPOT, HARDENED.  This is the fix for a glove that can SEE
   *  "Humanoid" at -54 dBm and still says "connection attempt timed
   *  out". Nothing was wrong with the name, the password or the range:
   *
   *   1. MODEM SLEEP.  By default the ESP32 radio dozes between
   *      beacons. The client's WPA2 handshake arrives while the radio
   *      is asleep, gets no answer, and the client gives up with a
   *      timeout -- exactly what you saw. WiFi.setSleep(false) is not
   *      optional on an access point.
   *   2. LOW TRANSMIT POWER.  11 dBm is plenty for a phone lying next
   *      to the robot and marginal for a handshake with the little
   *      antenna on a D1 mini. The access point now always runs at
   *      full power; WIFI_LOW_POWER still applies when the robot joins
   *      your home router instead, which is where the servo-twitch
   *      argument for backing the radio off actually came from.
   *   3. A PINNED ADDRESS AND CHANNEL.  192.168.4.1 on AP_CHANNEL, so
   *      the glove's WiFi.gatewayIP() and your dashboard link can
   *      never move, and the glove can aim straight at one channel
   *      instead of hunting across all thirteen.
   *=================================================================*/
  WiFi.mode(WIFI_AP);
  delay(50);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1),
                    IPAddress(192, 168, 4, 1),
                    IPAddress(255, 255, 255, 0));
  bool apOk = WiFi.softAP(AP_SSID,
                          AP_OPEN ? (const char*)NULL : (const char*)AP_PASS,
                          AP_CHANNEL, 0, AP_MAX_CLIENTS);
  WiFi.setSleep(false);                 /* an AP must never doze */
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  /* full power for the handshake */
  delay(100);
  if (!apOk) Serial.println(F("!! softAP() refused to start -- power-cycle the ESP32"));
  Serial.println(F("ESP32 hotspot started"));
  Serial.printf("SSID: %s\n", AP_SSID);
  Serial.print(F("IP: "));
  Serial.println(WiFi.softAPIP());
#if AP_OPEN
  Serial.println(F("password: none (open hotspot)"));
#else
  Serial.printf("password: %s\n", AP_PASS);
#endif
  Serial.printf("channel: %d   radio: full power, sleep off\n", AP_CHANNEL);
  Serial.print(F("dashboard: http://"));
  Serial.println(WiFi.softAPIP());
  Serial.println(F("Waiting for ESP8266..."));
  /*  Deliberately worded: the hotspot being up says NOTHING about the
   *  glove. "ESP8266 connected" is printed only when a packet with
   *  three believable sensor values actually arrives.                */
  Serial.println(F("(the glove is only reported connected when real flex data arrives)"));
}
/*  Prints the moment a board associates with the hotspot, before any
 *  HTTP happens at all. This separates "the glove cannot join the
 *  Wi-Fi" from "the glove joined but its packets are not arriving",
 *  which are two completely different faults.                        */
void apTick() {
  if (WiFi.getMode() != WIFI_AP) return;
  static uint8_t  was  = 255;
  static uint32_t next = 0;
  if ((int32_t)(millis() - next) < 0) return;
  next = millis() + 500;
  uint8_t n = WiFi.softAPgetStationNum();
  if (n == was) return;
  was = n;
  if (n) Serial.printf("a board joined the hotspot (%u connected)\n", (unsigned)n);
  else   Serial.println(F("hotspot empty -- Waiting for ESP8266..."));
}
void printWifi() {
  if (WiFi.getMode() == WIFI_AP) {
    Serial.printf("hotspot \"%s\" pass \"%s\"  ->  http://", AP_SSID, AP_PASS);
    Serial.println(WiFi.softAPIP());
  } else if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("http://"));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("wifi not connected"));
  }
}
#else
void wifiInit() {}
void apTick() {}
void printWifi() { Serial.println(F("wifi disabled in this build")); }
#endif

/*====================================================================
 *  19. BLYNK  (optional)  --  gives you the Blynk phone app
 *
 *  Set USE_BLYNK 1, install "Blynk" by Volodymyr Shymanskyy, make a
 *  template in blynk.cloud with switch datastreams V0..V9 and a
 *  Terminal on V20, then paste your three IDs below.
 *  Blynk.config() is used rather than Blynk.begin() so the local web
 *  page keeps working at the same time.
 *===================================================================*/
#if USE_BLYNK
#define BLYNK_TEMPLATE_ID   "TMPLxxxxxx"
#define BLYNK_TEMPLATE_NAME "Humanoid"
#define BLYNK_AUTH_TOKEN    "PASTE_YOUR_TOKEN"
#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp32.h>
BLYNK_WRITE(V0) { if (param.asInt()) queueCommand("stand");        }
BLYNK_WRITE(V1) { if (param.asInt()) queueCommand("say hi");       }
BLYNK_WRITE(V2) { if (param.asInt()) queueCommand("sit down");     }
BLYNK_WRITE(V3) { if (param.asInt()) queueCommand("stand up");     }
BLYNK_WRITE(V4) { if (param.asInt()) queueCommand("push ups");     }
BLYNK_WRITE(V5) { if (param.asInt()) queueCommand("dance");        }
BLYNK_WRITE(V6) { if (param.asInt()) queueCommand("demo");         }
BLYNK_WRITE(V7) { if (param.asInt()) queueCommand("move forward"); }
BLYNK_WRITE(V8) { if (param.asInt()) queueCommand("shake hand");   }
BLYNK_WRITE(V9) { if (param.asInt()) queueCommand("stop");         }
BLYNK_WRITE(V20) { queueCommand(String(param.asStr())); }   // Terminal widget
void blynkInit() { Blynk.config(BLYNK_AUTH_TOKEN); Blynk.connect(3000); }
#else
void blynkInit() {}
#endif

/*====================================================================
 *  20. SINRICPRO  (optional)  --  this is the one that gives you
 *  "Hey Google, turn on push ups". Google only understands devices,
 *  not commands, so each routine has to be its own virtual switch.
 *  Install "SinricPro" by sinricpro, create one Switch device per
 *  routine at portal.sinric.pro, and paste the IDs below.
 *===================================================================*/
#if USE_SINRIC
#include <SinricPro.h>
#include <SinricProSwitch.h>
#define SINRIC_APP_KEY    "PASTE_APP_KEY"
#define SINRIC_APP_SECRET "PASTE_APP_SECRET"
struct SinricSw { const char* id; const char* cmd; };
SinricSw SINRIC_SW[] = {
  { "PASTE_DEVICE_ID_1", "stand"        },
  { "PASTE_DEVICE_ID_2", "say hi"       },
  { "PASTE_DEVICE_ID_3", "sit down"     },
  { "PASTE_DEVICE_ID_4", "stand up"     },
  { "PASTE_DEVICE_ID_5", "move forward" },
  { "PASTE_DEVICE_ID_6", "demo"         }
};
#define NSINRIC ((int)(sizeof(SINRIC_SW) / sizeof(SINRIC_SW[0])))
bool onPowerState(const String &deviceId, bool &state) {
  for (int i = 0; i < NSINRIC; i++)
    if (deviceId == SINRIC_SW[i].id) {
      if (state) queueCommand(SINRIC_SW[i].cmd);
      else       queueCommand("stop");
      return true;
    }
  return false;
}
void sinricInit() {
  for (int i = 0; i < NSINRIC; i++) {
    SinricProSwitch &sw = SinricPro[SINRIC_SW[i].id];
    sw.onPowerState(onPowerState);
  }
  SinricPro.begin(SINRIC_APP_KEY, SINRIC_APP_SECRET);
}
#else
void sinricInit() {}
#endif

/*====================================================================
 *  21. IO PUMP  --  called from inside every delay, so a command can
 *  arrive and interrupt a move at any point
 *===================================================================*/
void pumpIO() {
  static uint32_t lastChar = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (gLineBuf.length()) { queueCommand(gLineBuf); gLineBuf = ""; }
    } else {
      gLineBuf += c;
      lastChar = millis();
    }
  }
  /*  Serial Monitors set to "no line ending" never send \n, so also
   *  accept a line once typing has paused.                          */
  if (gLineBuf.length() && millis() - lastChar > 120) {
    queueCommand(gLineBuf);
    gLineBuf = "";
  }
#if USE_WEB
  server.handleClient();
#endif
#if USE_BLYNK
  Blynk.run();
#endif
#if USE_SINRIC
  SinricPro.handle();
#endif
  mpuTick();
  faceTick();
  headGestTick();
  flexTick();          // the glove layer. Returns at once when idle.
  autoUpTick();        // fall recovery. Also returns at once when idle.
  drawSlideTick();     // saved-drawing slideshow, if it is switched on.
  balResumeTick();     // puts balance back ON once the robot stands up again.
  apTick();            // says so the moment the glove joins the hotspot.
  flexLinkTick();      // prints when the glove link comes and goes.
}

bool naptime(uint32_t ms) {
  uint32_t t0 = millis();
  for (;;) {
    pumpIO();
    if (gAbort) return false;
    if (millis() - t0 >= ms) return true;
    delay(1);
  }
}

/*====================================================================
 *  22. CONFIG COMMANDS  (anything that is not a body movement)
 *===================================================================*/
bool handleConfigCommand(const String &v) {
  /* ---- the flex glove, from the serial monitor as well ---- */
  if (v == "flex on")  { gFlexOn = true;  flexSave();
    Serial.println(F("flex control ON  (movements still take priority)"));
    return true; }
  if (v == "flex off") { gFlexOn = false; flexSave();
    Serial.println(F("flex control OFF -- robot behaves exactly as before"));
    return true; }
  if (v == "flex") {
    Serial.printf("flex control %s   glove %s   side %s   packets %lu\n",
                  gFlexOn ? "ON" : "OFF", flexLive() ? "connected" : "not seen",
                  flexSideName(), (unsigned long)gFlexPkts);
    uint8_t sd = gFlexSide ? 1 : 0;
    for (int i = 0; i < NFLEX; i++) {
      int j = gFlexJ[sd][i];
      Serial.printf("  %-19s raw %4d  straight %4d  bent %4d  -> %-5s %d..%d"
                    "   speed %2d (%3d deg/s)  deadband %2d\n",
        FLEXNAME[i], gFlexRaw[i], gFlexLo[i], gFlexHi[i],
        (j >= 0 && j < NJ) ? JOINTS[j].name : "none",
        gFlexAL[sd][i], gFlexAH[sd][i],
        (int)gFlexStep[i], (int)((long)gFlexStep[i] * 1000 / FLEX_APPLY_MS),
        (int)gFlexDead[i]);
    }
    return true;
  }
  if (v == "help" || v == "?")  { printHelp();               return true; }
  if (v == "list")              { listJoints();              return true; }
  if (v == "pose")              { printPose();               return true; }
  if (v == "quiet")             { quietReport();             return true; }
  if (v == "where") {
    Serial.printf("I am %s\n", whereAmIsafe());
    return true;
  }
  if (v == "wifi")              { printWifi();               return true; }
  if (v == "save")              { saveConfig();              return true; }
  if (v == "load")              { loadConfig(); listJoints(); return true; }
  /*  "smile" and the rest of the moods are handled by the bare-name
   *  lookup at the bottom of this function, so they work whether they
   *  arrive as  smile  or as  feel smile  from the dashboard.        */
  if (v == "tilt") {
    Serial.printf("effective tilt %.1f / %.1f deg   (this is what matters)\n",
                  gPitch - gPitchZero, gRoll - gRollZero);
    Serial.printf("raw pitch %.1f  roll %.1f   (zero %.1f / %.1f, sensor %s)\n",
                  gPitch, gRoll, gPitchZero, gRollZero,
                  gMpuOk ? "ok" : "NOT FOUND");
    Serial.printf("mounting angle does not matter -- bal zero teaches it.\n");
    Serial.printf("balance %s, live trim roll %d pitch %d, stance %d\n",
                  gBalOn ? "ON" : "off", (int)gBalRoll, (int)gBalPitch,
                  (int)gStance);
    return true;
  }
  if (v.startsWith("slot ")) {
    int i = v.substring(5).toInt() - 1;      /* 1-based for humans */
    if (i < 0 || i >= NSLOT || !gSlotUsed[i]) {
      Serial.println(F("no such saved position"));
      return true;
    }
    Serial.printf("going to \"%s\"\n", gSlotName[i]);
    gBusy = true; gAbort = false;
    balSuspend();
    glideTo(gSlotAng[i], 0);
    gBusy = false;
    balResume();
    gPosUnknown = true;
    return true;
  }
  if (v == "balzero" || v == "bal zero" || v == "balance zero") {
    balZero(); return true;
  }
  if (v == "baltest roll")      { balTest(true);             return true; }
  if (v == "baltest pitch" || v == "baltest") { balTest(false); return true; }
  if (v == "balance on") {
    gBalSaved = true;         /* so an action in progress hands it back */
    if (!gMpuOk) { Serial.println(F("no MPU6050 found -- nothing to balance with")); return true; }
    gBalOn = true;
    Serial.println(F("balance trim ON. It pauses by itself during every action"));
    Serial.println(F("and comes back when the robot is standing again."));
    Serial.println(F("It only acts while idle, only past"));
    Serial.printf("%.0f deg of tilt, and at most %d deg either way.\n",
                  (double)BAL_DEADBAND, BAL_MAX_DEG);
    return true;
  }
  if (v == "balance off") {
    gBalSaved = false;        /* and this one stays off afterwards too */
    gBalPend  = false;
    gBalOn = false;
    balReset();
    Serial.println(F("balance trim OFF and backed out to zero"));
    return true;
  }
  if (v == "defaults") {
    prefs.putUInt("magic", 0);
    Serial.println(F("saved config cleared -- reset the ESP32 to get the"));
    Serial.println(F("sketch's own JOINTS table back (ll2 inverted)."));
    return true;
  }
  if (v == "relax") {
    Serial.println(F("relaxing all servos -- HOLD THE ROBOT"));
    relaxAll();
    gPosUnknown = true;          // it may sag; do not trust the pose now
    faceSet(F_SLEEP, "RELAXED");
    return true;
  }
  if (v == "limiter on")  { gRateLim = true;  Serial.println(F("rate limiter ON"));  return true; }
  if (v == "limiter off") { gRateLim = false; Serial.println(F("rate limiter OFF -- routines can slam")); return true; }

  if (v.startsWith("steps")) {
    String a = v.substring(5); a.trim();
    if (a.length() == 0) { Serial.printf("half-steps per walk: %d\n", (int)gWalkHalf); return true; }
    int n = a.toInt();
    if (n < 1 || n > 40) { Serial.println(F("use 1..40")); return true; }
    gWalkHalf = n;
    Serial.printf("walk will now take %d half-steps\n", n);
    return true;
  }

  if (v.startsWith("stance")) {
    String a = v.substring(6); a.trim();
    if (a.length() == 0) { Serial.printf("stance: %d deg\n", (int)gStance); return true; }
    int n = a.toInt();
    if (n < 0 || n > 20) { Serial.println(F("use 0..20 degrees")); return true; }
    gStance = (int8_t)n;
    balApply();                  // takes effect immediately, flat soles kept
    Serial.printf("stance %d deg -- feet further apart, soles still flat\n", n);
    Serial.println(F("this is the cheapest stability win there is: a wider"));
    Serial.println(F("base tolerates more sway before it tips."));
    return true;
  }

  if (v.startsWith("speed")) {
    String a = v.substring(5);
    a.trim();
    if (a.length() == 0 || a == "auto") {
      gSpeedOverride = 0;
      Serial.println(F("speed: auto (slow for sit/stand, mid for push-ups)"));
    } else {
      int p = a.toInt();
      if (p < 40 || p > 400) { Serial.println(F("use 40..400, or  speed auto")); return true; }
      gSpeedOverride = p;
      Serial.printf("speed: %d%% of original timing (higher = slower)\n", p);
    }
    return true;
  }

  if (v.startsWith("flip ")) {
    int idx = findJoint(v.substring(5));
    if (idx < 0) { Serial.println(F("no such joint -- try  list")); return true; }
    JOINTS[idx].inv = !JOINTS[idx].inv;
    Serial.printf("%s inv = %s\n", JOINTS[idx].name, JOINTS[idx].inv ? "YES" : "no");
    JOINTS[idx].writeNow(JOINTS[idx].last);   // re-apply through the new direction
    Serial.println(F("type  save  to keep this after a reboot"));
    return true;
  }
  if (v.startsWith("trim ")) {
    int sp = v.indexOf(' ', 5);
    if (sp < 0) { Serial.println(F("use: trim <name> <degrees>")); return true; }
    int idx = findJoint(v.substring(5, sp));
    if (idx < 0) { Serial.println(F("no such joint")); return true; }
    JOINTS[idx].trim = v.substring(sp + 1).toInt();
    JOINTS[idx].writeNow(JOINTS[idx].last);
    Serial.printf("%s trim = %d\n", JOINTS[idx].name, JOINTS[idx].trim);
    return true;
  }
  if (v.startsWith("jog ")) {
    int sp = v.indexOf(' ', 4);
    if (sp < 0) { Serial.println(F("use: jog <name> <angle>")); return true; }
    int idx = findJoint(v.substring(4, sp));
    if (idx < 0) { Serial.println(F("no such joint")); return true; }
    jogJoint(idx, v.substring(sp + 1).toInt());
    return true;
  }
  /* ---- moods, sounds and head movements ---- */
  if (v == "feelings" || v == "moods") {
    Serial.println(F("\nmoods -- type the name, or  feel <name>"));
    for (int i = 0; i < NEMOTE; i++)
      Serial.printf("  %-11s face+sound+%s\n", EMOTES[i].name,
                    HG_NAMES[EMOTES[i].head]);
    Serial.println(F("\nsounds -- type  sound <name>"));
    for (int i = 1; i < S_COUNT; i++) Serial.printf("  %s\n", SND_NAMES[i]);
    Serial.println(F("\nhead movements -- type  head <name>"));
    for (int i = 1; i < HG_COUNT; i++) Serial.printf("  %s\n", HG_NAMES[i]);
    Serial.println();
    return true;
  }
  if (v.startsWith("feel ")) {
    String n = v.substring(5); n.trim();
    if (!expressNamed(n)) {
      Serial.printf("no mood called \"%s\" -- type  feelings\n", n.c_str());
      express(F_CURIOUS, S_ERROR, HG_SHAKE, "SORRY?");
    }
    return true;
  }
  if (v.startsWith("sound ")) {
    String n = v.substring(6); n.trim();
    if (n == "stop") { soundStop(); Serial.println(F("sound stopped")); return true; }
    if (!soundNamed(n)) Serial.printf("no sound called \"%s\"\n", n.c_str());
    return true;
  }
  if (v.startsWith("head ")) {
    String n = v.substring(5); n.trim();
    if (n == "stop")   { headGestStop(); return true; }
    if (n == "centre" || n == "center") {
      headGestStop(); JOINTS[0].write(HEAD_CENTRE); return true;
    }
    if (!headNamed(n)) Serial.printf("no head movement called \"%s\"\n", n.c_str());
    return true;
  }
  if (v.startsWith("vol")) {
    String a = v.substring(3); a.trim();
    if (a.length() == 0) {
      Serial.printf("volume %d%%%s  (amp %s)\n", (int)gVol,
                    gMute ? ", MUTED" : "", gAudioOk ? "ok" : "NOT FOUND");
      return true;
    }
    int n = a.toInt();
    if (n < 0 || n > VOL_MAX) {
      Serial.printf("use 0..%d   (100 = full scale, above that the soft "
                    "limiter keeps it clean)\n", VOL_MAX);
      return true;
    }
    gVol = (uint8_t)n;
    gMute = (n == 0);
    if (!gMute) soundPlay(S_BEEP);
    Serial.printf("volume %d%%%s\n", n,
                  n > 100 ? "  (extra gain, soft limited)" : "");
    return true;
  }
  if (v == "mute")   { soundStop(); gMute = true;  Serial.println(F("muted")); return true; }
  if (v == "unmute") { gMute = false; soundPlay(S_READY); Serial.println(F("unmuted")); return true; }

  if (v.startsWith("sweep ")) {
    int s1 = v.indexOf(' ', 6);
    if (s1 < 0) { Serial.println(F("use: sweep <name> <from> <to>")); return true; }
    int s2 = v.indexOf(' ', s1 + 1);
    if (s2 < 0) { Serial.println(F("use: sweep <name> <from> <to>")); return true; }
    int idx = findJoint(v.substring(6, s1));
    if (idx < 0) { Serial.println(F("no such joint")); return true; }
    sweepJoint(idx, v.substring(s1 + 1, s2).toInt(), v.substring(s2 + 1).toInt());
    return true;
  }

  /*  Last chance: a bare mood name. This is what makes voice work --
   *  saying "angry" or "sleepy" is enough, no keyword needed. It is
   *  checked last so a real command can never be shadowed by a mood. */
  if (expressNamed(v)) return true;

  return false;
}

/*====================================================================
 *  23. RUN A COMMAND
 *
 *  1. establish a known pose if we have not moved since power-up
 *  2. back the balance trim out to zero, so a routine never fights it
 *  3. glide to the pose this routine needs to start from
 *  4. wait SETTLE_MS  (your "after two seconds")
 *  5. run the routine at its own speed
 *  6. put the robot back to standing (or recover from the push-up)
 *===================================================================*/
bool assertKnownPose() {
  if (!gPosUnknown) return true;
  Serial.println(F("I have not moved since power-up, so I do not know"));
  Serial.println(F("where I am. Easing into standing (rate limited)."));
  faceSet(F_BUSY, "FINDING HOME");
  stand_straight();
  gPosUnknown = false;
  if (gAbort) return false;
  Serial.println(F("standing."));
  return naptime(SETTLE_MS);
}

void runCommand(const String &v) {
  if (handleConfigCommand(v)) return;

  /*  Whatever happens next, start from zero balance trim. A routine
   *  moving the legs while the balancer also holds an offset on them
   *  is two controllers on one joint, and that is how oscillation
   *  starts.                                                        */
  int8_t stanceKeep = gStance;
  gStance = 0;
  balReset();
  gStance = stanceKeep;
  balApply();

  /* ---- plain "stand": glide there, never snap ---- */
  if (v == "stand") {
    gBusy = true; gRunning = v;
    express(F_BUSY, S_STAND, HG_PERK, "STANDING");
    if (gPosUnknown) assertKnownPose();
    else {
      Serial.println(F("going to the initial standing position..."));
      goToPose(POSE_STAND);
    }
    gBusy = false; gRunning = "";
    if (gAbort) { gAbort = false; gFreeze = false; }
    express(F_IDLE, S_BEEP, HG_PERK, "READY");
    Serial.printf("done -- %s\n", whereAmIsafe());
    return;
  }

  int idx = -1;
  for (int i = 0; i < NCMD; i++) if (v == CMDS[i].words) { idx = i; break; }
  if (idx < 0) {
    /*  Say so out loud as well as on the serial port -- if you sent
     *  this by voice you are not looking at a serial monitor.        */
    express(F_CURIOUS, S_ERROR, HG_SHAKE, "SORRY?");
    Serial.printf("unknown command: \"%s\"   (type help)\n", v.c_str());
    return;
  }
  const CmdEntry &e = CMDS[idx];

  gBusy    = true;
  gRunning = v;
  gTimePct = e.speed;
  /*  Stand the balance trim down for the whole action, whatever it is.
   *  It comes back on its own afterwards -- see balResume().        */
  balSuspend();

  /*  Face, voice and head all leave from here together, off one table
   *  row, which is why they can never disagree about what the robot is
   *  doing. The walk and the dance loop their sound for as long as the
   *  routine runs; everything else plays once.                       */
  bool loopSound = (e.sound == S_WALK || e.sound == S_MUSIC);
  faceSet(e.face, v.c_str());
  if (loopSound)               soundLoop(e.sound);
  else if (e.sound != S_NONE)  soundPlay(e.sound);
  if (e.head != HG_NONE)       headGesture(e.head);

  do {
    if (!assertKnownPose()) break;

    /* already there? do not fold and unfold for nothing */
    if (v == "stand up" && poseDistance(POSE_STAND) <= POSE_TOL) {
      Serial.println(F("I am already standing."));
      break;
    }
    if (v == "sit down" && poseDistance(POSE_SIT) <= POSE_TOL) {
      Serial.println(F("I am already sitting."));
      break;
    }

    /* If push_ups was interrupted the robot is lying on its hands.
     * Sliding straight to any other pose from there would drag it, so
     * get up properly first.                                        */
    if (e.pre != POSE_PLANK && poseDistance(POSE_PLANK) <= POSE_TOL) {
      Serial.println(F("I am still in the push-up position -- getting up first"));
      push_ups_recover();
      if (gAbort) break;
    }

    /* ---- the precondition glide: this is the anti-slam fix ---- */
    if (e.pre) {
      int d = poseDistance(e.pre);
      if (d > POSE_TOL) {
        const char* want = (e.pre == POSE_SIT)     ? "sitting"
                         : (e.pre == POSE_HANDSUP) ? "standing with hands up"
                                                   : "standing";
        Serial.printf("I am %s, and \"%s\" has to start from %s.\n",
                      whereAmIsafe(), v.c_str(), want);
        Serial.printf("Moving there smoothly (%d deg to cover)...\n", d);
        if (!goToPose(e.pre)) break;
        Serial.printf("in position. settling %d ms...\n", SETTLE_MS);
        if (!naptime(SETTLE_MS)) break;
      }
    }

    if (SIT_SPREAD_DEG > 0 && e.fn == sit_down) {
      Serial.println(F("widening the stance before sitting"));
      if (!hipSpread(SIT_SPREAD_DEG)) break;
    }

    /* ---- the routine itself, exactly your Mega angles ---- */
    Serial.printf("running \"%s\" at %u%%\n", v.c_str(),
                  (unsigned)(gSpeedOverride > 0 ? gSpeedOverride : gTimePct));
    e.fn();
  } while (0);

  bool interrupted = gAbort;
  gTimePct = SPEED_DEFAULT;
  if (loopSound) soundStop();       // the looping riff ends with the move

  if (interrupted) {
    gAbort = false;
    if (gCmd.length()) {                 // something else is already queued
      Serial.println(F("interrupted"));
    } else if (gFreeze) {
      gFreeze = false;
      express(F_ALERT, S_ERROR, HG_SHIVER, "FROZEN");
      Serial.printf("frozen -- %s. Type  stand  when ready.\n", whereAmIsafe());
    } else {
      Serial.println(F("interrupted -- returning to standing"));
      express(F_BUSY, S_BEEP, HG_DROOP, "RECOVERING");
      glideTo(POSE_STAND, -1);
      gAbort = false;
    }
  } else {
    switch (e.post) {
      case POST_RECOVER:
        gTimePct = e.speed;
        push_ups_recover();
        gTimePct = SPEED_DEFAULT;
        if (gAbort) { gAbort = false; glideTo(POSE_STAND, -1); gAbort = false; }
        break;
      case POST_STAND:
        if (poseDistance(POSE_STAND) > POSE_TOL) goToPose(POSE_STAND);
        break;
      case POST_STAY:
        break;
    }
  }

  gBusy    = false;
  gRunning = "";
  balResume();                  // restore balance only if it was on before
  gLastMotionMs = millis();     // hold the balancer off until we settle
  headGestStop();
  soundPlay(interrupted ? S_ERROR : S_WIN);
  faceSet(poseDistance(POSE_SIT) <= POSE_TOL ? F_SIT : F_IDLE, "READY");
  Serial.printf("done -- %s\n\n", whereAmIsafe());
}

/*====================================================================
 *  24. SETUP
 *===================================================================*/
void setup() {
  Serial.begin(BAUD);
  delay(400);

  /*  Servo bus first, deliberately slow. 100 kHz over 30 cm of
   *  unshielded jumper wire next to seventeen motors is reliable;
   *  400 kHz is not, and a corrupted setPWM is a servo twitch.      */
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_SERVO_HZ);
  pwm.begin();
  pwm.setOscillatorFrequency(PCA_OSC_HZ);
  pwm.setPWMFreq(PCA_FREQ);
  delay(20);

  headInit();

  /*  Sound comes up before anything moves, so the boot chime plays
   *  WHILE the servos are energising rather than after.              */
  audioStart();
  soundPlay(S_BOOT);

  fsStart();             // P7: the flash file store for pictures and sounds
  prefs.begin("humanoid", false);
  slotsLoad();
  flexLoad();            // flex calibration + last ON/OFF state
#if USE_WEB
  manualLoad();          // PART 2: was Manual Servo Control left ON or OFF?
#endif
  vLoad();               // your recorded clip, if flash has one
  loadConfig();          // restores any flip/trim you saved

#if USE_OLED
  gOledOk = oledInit();
  Serial.println(gOledOk ? F("OLED found on GPIO 32/33")
                         : F("OLED not found -- check SDA 32 / SCL 33 / addr"));
  faceSet(F_BOOT, "HELLO");
  faceStart();          // the face now animates on core 0, by itself
#endif

#if USE_MPU
  gMpuOk = mpuInit();
  Serial.println(gMpuOk ? F("MPU6050 found on 0x68")
                        : F("MPU6050 NOT found on 0x68"));
  if (gMpuOk && gBalOn) Serial.println(F("balance trim is ON"));
#endif

  Serial.println(F("\n=============================================="));
  Serial.println(F(" HUMANOID  v6  --  ESP32 + PCA9685 + GPIO25"));
  Serial.println(F(" Your routines, unchanged. Quiet standing, a"));
  Serial.println(F(" real gait, balance, 20 faces, 26 sounds, and a"));
  Serial.println(F(" head that moves with every single command."));
  Serial.printf( " anti-slam limiter : %d deg per %d ms\n", MAX_STEP_DEG, SUBSTEP_MS);
  Serial.printf( " duplicate filter  : %s\n", DEDUPE_WRITES ? "on" : "OFF");
  Serial.printf( " servo I2C         : %d kHz\n", I2C_SERVO_HZ / 1000);
  Serial.printf( " sound             : %s\n",
                 gAudioOk ? "MAX98357A on 14/26/27" : "off / not found");
  Serial.printf( " head servo        : GPIO %d (the 17th joint)\n", HEAD_PIN);
  Serial.println(F("=============================================="));
  Serial.println(F(" POWER: 17 MG996R servos want 6 V at 6-10 A."));
  Serial.println(F(" On a 6 V 2 A supply they WILL buzz and sag no"));
  Serial.println(F(" matter what this code does. See the note at the"));
  Serial.println(F(" top of the sketch."));
  Serial.println(F("==============================================\n"));

  /*  Bring the servos up one at a time instead of all seventeen in the
   *  same millisecond. This alone removes the power-up jerk.         */
  energiseStaggered(POSE_STAND);
  gPosUnknown = false;         // we just physically drove every joint
  balApply();                  // apply the saved stance, if any

  wifiInit();
#if USE_WEB
  server.on("/",       handleRoot);
  server.on("/run",    handleRun);
  server.on("/status", handleStatus);
  server.on("/angles", handleAngles);
  server.on("/joint",  handleJoint);
  server.on("/plist",  handlePlist);
  server.on("/psave",  handlePsave);
  server.on("/pdel",   handlePdel);
  /* ---- the ESP8266 flex glove ---- */
  /* ---- OLED drawing, text, banner, permanent slots ---- */
  server.on("/odraw",  handleOdraw);
  server.on("/otext",  handleOtext);
  server.on("/oban",   handleOban);
  server.on("/oface",  handleOface);
  server.on("/dsave",  handleDsave);
  server.on("/dload",  handleDload);
  server.on("/ddel",   handleDdel);
  server.on("/dslide", handleDslide);
  server.on("/dstat",  handleDstat);
  server.on("/fsl",    handleFsl);
  server.on("/fsave",  handleFsave);
  server.on("/fload",  handleFload);
  server.on("/fdel",   handleFdel);
  /* ---- auto stand-up ---- */
  server.on("/aup",    handleAup);
  server.on("/aupnow", handleAupNow);
  /* ---- recorded voice ---- */
  server.on("/vrec",   handleVrec);
  server.on("/vplay",  handleVplay);
  server.on("/vstop",  handleVstop);
  server.on("/vsave",  handleVsave);
  server.on("/vclear", handleVclear);
  server.on("/flex",   handleFlex);     // the glove posts readings here
  server.on("/fon",    handleFon);
  server.on("/foff",   handleFoff);
  server.on("/fstat",  handleFstat);
  server.on("/fcal",   handleFcal);
  server.on("/fset",   handleFset);
  server.on("/flim",   handleFlim);     // PART 1: per-motor speed + deadband
  server.on("/manual", handleManual);   // PART 2: the slider master switch
  server.onNotFound(handleRoot);
  server.begin();
#endif
  blynkInit();
  sinricInit();

  listJoints();
  printHelp();

  /*  Everything is up: one bright chime, a happy face and a look
   *  around, all fired together so they land together.               */
  express(F_HAPPY, S_READY, HG_SCAN, "READY");
  gLastMotionMs = millis();

#if MOVE_ON_BOOT
  queueCommand("stand");
#else
  Serial.println(F("Standing and holding. Nothing else will move until"));
  Serial.println(F("you ask. Suggested first three commands:"));
  Serial.println(F("    quiet          <- should report ZERO writes"));
  Serial.println(F("    steps 2        <- take two half-steps only"));
  Serial.println(F("    walk"));
  Serial.println(F("If the left hip swings the wrong way:  flip ll2  then  save"));
#endif
}

/*====================================================================
 *  25. LOOP
 *===================================================================*/
void loop() {
  pumpIO();

  if (gCmd.length() == 0) { delay(2); return; }

  String c = gCmd;
  gCmd   = "";
  gAbort = false;

  Serial.printf("\n> %s\n", c.c_str());
  runCommand(c);
}
