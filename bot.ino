// LAST UPDATE 28/01/26
// BY DAG

#include <Arduino.h>
#include <Wire.h>               // screen
#include <Adafruit_GFX.h>       // screen
#include <Adafruit_SSD1306.h>   // screen
#include <Bluepad32.h>          // for controller 
#include <TB6612_ESP32.h>       // tbs' control lib
#include <math.h>               // calc inputs 

bool headingInverted = false;

// OLED 0.96 inches 128 x 64 px

#define OLED_SDA 23
#define OLED_SCL 22
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// Screen modes
enum DisplayMode { DISP_NORMAL, DISP_DEBUG };
DisplayMode currentDisplay = DISP_NORMAL;
// Controller
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
// Other values
float SvX = 0;  // get normalized value joystick X
float SvY = 0;  // get normalized value joystick Y
float SvR = 0;  // get normalized value right joystick X

// ====================== TBs ======================

// TB1 (F)

#define FL_AIN1 14  // diag(0)
#define FL_AIN2 12
#define FL_PWMA  13

#define FR_BIN1 27  // diag(1)
#define FR_BIN2 26
#define FR_PWMB  25

#define TB1_STBY 32

// TB2 (R)

#define RL_AIN1 16  // diag(3)
#define RL_AIN2 17
#define RL_PWMA  5

#define RR_BIN1 4   // diag(4)
#define RR_BIN2 19
#define RR_PWMB  15

#define TB2_STBY 33

// Diag in case 1 motors is in wrong dir, 1 forward -1 backward
#define OFFSET_FL 1
#define OFFSET_FR -1
#define OFFSET_RL 1
#define OFFSET_RR -1

// ====================== MOTOR ASSIGNMENTS ======================

Motor motorFL(FL_AIN1, FL_AIN2, FL_PWMA, OFFSET_FL, TB1_STBY, 5000, 8, 1);
Motor motorFR(FR_BIN1, FR_BIN2, FR_PWMB, OFFSET_FR, TB1_STBY, 5000, 8, 2);
Motor motorRL(RL_AIN1, RL_AIN2, RL_PWMA, OFFSET_RL, TB2_STBY, 5000, 8, 3);
Motor motorRR(RR_BIN1, RR_BIN2, RR_PWMB, OFFSET_RR, TB2_STBY, 5000, 8, 4);

// ====================== MOTOR CONTROL ==========================

const int MAX_WHEEL_SPEED = 255; //change max pmw value here, max 255.
const float ACCEL_RATE = 0.1; // for not putting power in an instant.
const int DEADZONE = 40; // controller deadzone
const float SNAP_ANGLE = 15.0; // drive assists in deg
const int JOY_MAX = 512; // max value of controller in 1 dir

// Infrom control variables
float targetSpeeds[4] = {0,0,0,0};
float currentSpeeds[4] = {0,0,0,0};
float diag[4];

// Stop
char motorMode[32] = "STOP";
float overallSpeedMagnitude = 0.0;

// Assign values to diag[n] to make stuff easier to debug
void driveMotors(float sFL, float sFR, float sRL, float sRR) {
    diag[0] = sFL; // getting values for screen
    diag[1] = sFR;
    diag[2] = sRL;
    diag[3] = sRR;

    motorFL.drive(diag[0]);
    motorRL.drive(diag[2]);
    motorFR.drive(diag[1]);
    motorRR.drive(diag[3]);
}

void hardStop() { targetSpeeds[0]=targetSpeeds[1]=targetSpeeds[2]=targetSpeeds[3]=0; }

// ====================== CONTROL ALGORITHMS ======================

void processGamepad(ControllerPtr ctl) {
    int16_t lx = ctl->axisX();                                                  // left joystick's X axis - Strafe
    int16_t ly = ctl->axisY();                                                  // left joystick's Y axis - Forward/Backward
    int16_t rx = ctl->axisRX();                                                 // right joystick X axis - Rotation
    uint8_t dpad = ctl->dpad();                                                 // dpad - pivoting 1 wheels
    bool btnY = ctl->y(), btnA = ctl->a(), btnX = ctl->x(), btnB = ctl->b();    // XYAB buttons - Testing or 1 wheel control
    bool btnL3 = ctl->thumbL();                                                 // L3 button to swap main heading
   
    // filter L3 button pressed
    static bool lastL3 = false;
    if (btnL3 && !lastL3) {
    headingInverted = !headingInverted;
    }
    lastL3 = btnL3;
    
    // -------- Deadzone / Adjust in Motor Controls ----------
    float vX = (abs(lx) < DEADZONE) ? 0 : (float)lx / JOY_MAX;
    float vY = (abs(ly) < DEADZONE) ? 0 : -(float)ly / JOY_MAX; // invert Y
    float vR = (abs(rx) < DEADZONE) ? 0 : (float)rx / JOY_MAX;

// -------- Drive assists / Adjust at SNAP_ANGLE in Motor Contrls -------------

    float mag = sqrt(vX*vX + vY*vY);

    if (mag > 0.1) {   

        float angle = atan2(vY, vX) * 180.0 / PI;
        if (angle < 0) angle += 360;

        // Horizontal
        if (fabs(angle) < SNAP_ANGLE || fabs(angle-360) < SNAP_ANGLE ||
            fabs(angle-180) < SNAP_ANGLE) {
            vY = 0;
        }

        // Vertical
        else if (fabs(angle-90) < SNAP_ANGLE || fabs(angle-270) < SNAP_ANGLE) {
            vX = 0;
        }
    }
    
    // Heading inverter
    if (headingInverted) {
    vX = -vX;
    vY = -vY;
    }
    // Normalize controls for joystick position
    vX = vX * fabs(vX);
    vY = vY * fabs(vY);
    vR = vR * fabs(vR);
    // Store deadzones value
    SvX = vX;
    SvY = vY;
    SvR = vR;

    // Braking
    float wFL=0, wFR=0, wRL=0, wRR=0;
    strcpy(motorMode, "STOP");
    overallSpeedMagnitude = 0;

    // Screen modes select
    if (dpad & DPAD_UP) {currentDisplay = DISP_NORMAL;} 
    else if (dpad & DPAD_DOWN) {currentDisplay = DISP_DEBUG;}

    // D-PAD pivoting
    if (dpad & DPAD_LEFT) { 
        wFL=0; wFR=0; wRL=1; wRR=-1; 
        strcpy(motorMode,"P-L"); 
        overallSpeedMagnitude=1; 
        }
    else if (dpad & DPAD_RIGHT) { 
        wFL=-1; wFR=1; wRL=0; wRR=0; 
        strcpy(motorMode,"P-R"); 
        overallSpeedMagnitude=1; 
        }

    // Controls using XYAB button - test each wheels
    else if (btnY) { wFR=1; strcpy(motorMode,"FR"); overallSpeedMagnitude=1; }
    else if (btnA) { wFL=1; strcpy(motorMode,"FL"); overallSpeedMagnitude=1; }
    else if (btnX) { wRL=1; strcpy(motorMode,"RL"); overallSpeedMagnitude=1; }
    else if (btnB) { wRR=1; strcpy(motorMode,"RR"); overallSpeedMagnitude=1; }

    // Joysticks controls
    else {
        wFL = vY + vX + vR; //mecanum maths to determine each wheel speed and direction
        wFR = vY - vX - vR;
        wRL = vY - vX + vR;
        wRR = vY + vX - vR;
        overallSpeedMagnitude = sqrt(vX*vX + vY*vY + vR*vR);
        if (overallSpeedMagnitude > 0.1) { // determine which states to show on screen
            if (abs(vR) > 0.1) {
                if (fmax(fabs(vX), fabs(vY)) > 0.1) strcpy(motorMode, "TURN");
                else strcpy(motorMode, "ROTATE");
            } else {
                if (fabs(vY) > fabs(vX)) strcpy(motorMode, vY > 0 ? "FW" : "BW");
                else strcpy(motorMode, vX > 0 ? "STR-R" : "STR-L");
                if (fabs(vY) > 0.1 && fabs(vX) > 0.1) strcpy(motorMode, "DIAG");
            }
        }
    }
    float maxMag = fmax(fmax(fabs(wFL), fabs(wFR)), fmax(fabs(wRL), fabs(wRR)));
    if (maxMag > 1) { wFL/=maxMag; wFR/=maxMag; wRL/=maxMag; wRR/=maxMag; }

    targetSpeeds[0] = wFL * MAX_WHEEL_SPEED;
    targetSpeeds[1] = wFR * MAX_WHEEL_SPEED;
    targetSpeeds[2] = wRL * MAX_WHEEL_SPEED;
    targetSpeeds[3] = wRR * MAX_WHEEL_SPEED;
}

// ====================== DISPLAY =================================

void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    // Debug mode
    if (currentDisplay == DISP_DEBUG) {
        display.setCursor(0,0);
        display.printf("Mode: %s\n", motorMode);
        display.printf("LX:%+.2f LY:%+.2f RX:%+.2f\n", SvX, SvY, SvR); // joystick normalized
        display.printf("FL:%4.1f/%4.1f\n", currentSpeeds[0], targetSpeeds[0]);
        display.printf("FR:%4.1f/%4.1f\n", currentSpeeds[1], targetSpeeds[1]);
        display.printf("RL:%4.1f/%4.1f\n", currentSpeeds[2], targetSpeeds[2]);
        display.printf("RR:%4.1f/%4.1f\n", currentSpeeds[3], targetSpeeds[3]);
    } 
    else { 
        // Disconnected
        bool hasController = false;
        for (auto ctl : myControllers)
            if (ctl && ctl->isConnected()) hasController = true;

        if (!hasController) {
            display.setCursor(15, 20);
            display.println("DISCONNECTED");
            display.setCursor(10, 40);
            display.println("Motors Braked.");
    }
    else {
            display.setTextSize(1);
            // Get joystick's angle
            float moveAngle = 0;
            if (overallSpeedMagnitude > 0.05) {
                moveAngle = atan2(SvY, SvX) * 180.0 / PI;
                if (moveAngle < 0) moveAngle += 360;
            }

            display.setCursor(0, 0);
            display.printf("M:%s %s %.2f",
                motorMode,
                headingInverted ? "INV" : "NOR",
                moveAngle
            );

            // Overall Speed
            int overallBarMaxWidth = 70;
            int overallBarWidth = overallSpeedMagnitude * overallBarMaxWidth;
            display.setCursor(0, 10);
            display.printf("OS:%3d%% ", (int)(overallSpeedMagnitude*100));
            display.drawRect(48, 10, overallBarMaxWidth, 6, WHITE);
            display.fillRect(48, 10, overallBarWidth, 6, WHITE);

            // Individual wheel's bars
            const char* wheelNames[4] = {"FL","FR","RL","RR"};
            display.setCursor(2, 21);
            display.printf("%s   %s   %s   %s", wheelNames[0], wheelNames[1], wheelNames[2], wheelNames[3]);
            int midY = 43;
            int maxBarHeight = 13;
            float wheelSpeeds[4] = {
                currentSpeeds[0]/MAX_WHEEL_SPEED,
                currentSpeeds[1]/MAX_WHEEL_SPEED,
                currentSpeeds[2]/MAX_WHEEL_SPEED,
                currentSpeeds[3]/MAX_WHEEL_SPEED
            };
            int barX[4] = {10, 35, 70, 95}; 
            for (int i=0;i<4;i++){
                int barLength = wheelSpeeds[i]*maxBarHeight;
                display.drawRect(barX[i], midY - maxBarHeight, 8, maxBarHeight*2, WHITE); 
                if (barLength >= 0){
                    display.fillRect(barX[i], midY - barLength, 8, barLength, WHITE); 
                }
                else {
                    display.fillRect(barX[i], midY, 8, -barLength, WHITE); 
                }
                display.setCursor(barX[i], midY + maxBarHeight + 1);
                display.printf("%3d", (int)(wheelSpeeds[i]*100));
            }
        }
    }

    display.display();
}


// ====================== BLUEPAD ======================

void processControllers() {
    for (auto ctl : myControllers) if (ctl && ctl->isConnected() && ctl->hasData()) processGamepad(ctl);
}

void onConnectedController(ControllerPtr ctl) {
    for (int i=0;i<BP32_MAX_GAMEPADS;i++) {
        if (myControllers[i]==nullptr) { myControllers[i]=ctl; break; }
    }
}
// FAILSAFE
void onDisconnectedController(ControllerPtr ctl) {
    for (int i=0;i<BP32_MAX_GAMEPADS;i++) {
        if (myControllers[i]==ctl) {
            myControllers[i]=nullptr;
            for (int j=0;j<4;j++){ targetSpeeds[j]=0; currentSpeeds[j]=0; }
            hardStop();
        }
    }
}

// ====================== SETUP ======================
void setup() {
    Serial.begin(115200);
    Wire.begin(OLED_SDA, OLED_SCL);

    for (int i=0;i<BP32_MAX_GAMEPADS;i++) myControllers[i]=nullptr;
    for (int i=0;i<4;i++) targetSpeeds[i]=currentSpeeds[i]=0;

    if(!display.begin(SSD1306_SWITCHCAPVCC,0x3C)) { 
        Serial.println(F("SSD1306 failed"));
        for(;;);
    }
    
    pinMode(TB1_STBY, OUTPUT); digitalWrite(TB1_STBY, HIGH); // remember to set to high otherwise the tb wont be turned on
    pinMode(TB2_STBY, OUTPUT); digitalWrite(TB2_STBY, HIGH);

    display.clearDisplay(); 
    display.println("Suck the Bot - Sbot");
    display.display();

    BP32.setup(&onConnectedController,&onDisconnectedController);

}

// ====================== LOOP ======================

void loop() {
    BP32.update();

    processControllers();

    for(int i=0;i<4;i++) currentSpeeds[i]+=(targetSpeeds[i]-currentSpeeds[i])*ACCEL_RATE; // calc current speed for speed scaling

    driveMotors(currentSpeeds[0], currentSpeeds[1], currentSpeeds[2], currentSpeeds[3]);

    static unsigned long lastDisplay=0;
    if (millis()-lastDisplay>100){ updateDisplay(); lastDisplay=millis(); }

}
