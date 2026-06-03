#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiServer.h>

// --- CONFIGURACIÓN WI-FI (Se mantiene en segundo plano) ---
const char *ssid = "Balancini";
const char *password = "12345678"; 
WiFiServer server(4545);           
WiFiClient client;
bool running = true;  // El robot arranca activo por defecto

// --- DEFINICIÓN DE PINES ---
// I2C (MPU6050)
#define PIN_SDA           21   
#define PIN_SCL           22   

// Motores 
#define PIN_PWM_L         14  
#define PIN_DIR_L1        5    
#define PIN_DIR_L2        4  

#define PIN_PWM_R         27   
#define PIN_DIR_R1        2    
#define PIN_DIR_R2        15   

// Encoders
#define PIN_ENC_L_A       18   
#define PIN_ENC_L_B       19   
#define PIN_ENC_R_A       33   
#define PIN_ENC_R_B       32   

// Sensores de Línea
#define PIN_SENSOR_L      35   
#define PIN_SENSOR_R      34   

// CONFIGURACIÓN PWM
#define PWM_FREQ          1000  // 1 KHz
#define PWM_RES           8     // 8 bits

// --- VARIABLES DE CONTROL (MODO AUTÓNOMO) ---
float vd = 0.0;  // Velocidad deseada en cero para que no intente avanzar
float k1 = 0.13; 
float k2 = 2.0; 
float k3 = 2.1; 
float k4 = 3.37; 
float kp = 0.68; 
float kv = 0.049; 
float Ts_received = 0.01; 

// Constantes Físicas (Fijas)
#define Ts      0.01   
#define ppr     12.0
#define pi_     3.141593
#define pi_s2   1.570796
#define Ra      3.0
#define NR      44.0
#define R       0.035
#define km      0.0008
#define b       0.09
#define calpha  -0.05 // <-- Este es el valor que debes ajustar viendo el Monitor Serie
#define tauM    0.3
#define alphaM  1.4 
#define omegaM  16.0
#define uM      11
#define uNM     11

#define deg_2_rad          0.0174533 
#define rad_2_deg          57.29578
#define accel_factor       0.0000610352
#define gyro_factor        0.0076336
#define c1 0.995

// Registros MPU
#define MPU_ADDR           0x68
#define PWR_MGMT_1         0x6B
#define CONFIG_R           0x1A
#define GYRO_CONFIG        0x1B
#define ACCEL_XOUT_H       0x3B
#define GYRO_YOUT_H        0x45

// Variables Globales
int16_t Ax_raw, Gy_raw;
uint8_t uWr, uWl;
volatile long posr = 0; 
volatile long posl = 0;
long posr_prev = 0;
long posl_prev = 0;
int16_t incr, incl;
float Sr, Sl;

float Xa=0.0, Yg=0.0, alpha=0.0;
float accelx=0.15, angulox=0.15, angulox_1=0.0, c2=1.0-c1;
float t=0.0, iTs=1.0/Ts, esc=pi_/(2.0*ppr*NR), escs=255.0/uM;
float omegar=0, omegal=0;
float ur=0.0, ul=0.0;
float v=0.0, alpha_1=0.0, theta=0.0, thetad=0.0, thetatil=0.0, taua=0.0, u=0.0;
float alphap=0.0, vtil=0.0, thetap=0.0, taur=0.0, taul=0.0; 
float Rasnkm=Ra/(NR*km), nkm=NR*km, intvtil=0.0, v2tauM=2.0*tauM;
float mv=255.0/(2.0*omegaM*R), mtheta=255.0/(2.0*0.3), malpha=255.0/(2.0*alphaM), momega=255.0/(2.0*omegaM), mu=255.0/(2.0*uM);

unsigned long previousMicros = 0;

float datos_enviar[8]; 

// --- FUNCIONES MPU ---
void MPU6050_write(int reg, int data) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void MPU6050_init() {
    MPU6050_write(PWR_MGMT_1, 0x80); delay(100);
    MPU6050_write(PWR_MGMT_1, 0x00); delay(100);
    MPU6050_write(CONFIG_R, 0x01);   delay(10);
    MPU6050_write(GYRO_CONFIG, 0x00);
}

void MPU6050_read_Ax() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    Ax_raw = Wire.read() << 8 | Wire.read();
}

void MPU6050_read_Gy() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_YOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    Gy_raw = Wire.read() << 8 | Wire.read();
}

// --- INTERRUPCIONES ---
void IRAM_ATTR isr_enc_l() {
    int A = digitalRead(PIN_ENC_L_A);
    int B = digitalRead(PIN_ENC_L_B);
    if (A == B) posl = posl + 1; else posl = posl - 1;
}

void IRAM_ATTR isr_enc_r() {
    int A = digitalRead(PIN_ENC_R_A);
    int B = digitalRead(PIN_ENC_R_B);
    if (A == B) posr = posr - 1; else posr = posr + 1; 
}

// --- FUNCIÓN PARA DETENER MOTORES ---
void stopMotors() {
    ledcWrite(PIN_PWM_L, 0);
    ledcWrite(PIN_PWM_R, 0);
    digitalWrite(PIN_DIR_L1, LOW); digitalWrite(PIN_DIR_L2, LOW);
    digitalWrite(PIN_DIR_R1, LOW); digitalWrite(PIN_DIR_R2, LOW);
    
    intvtil = 0;
    angulox = 0;
    angulox_1 = 0;
}

void setup() {
    Serial.begin(115200);
    
    // --- INICIO WI-FI AP ---
    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    server.begin();

    // --- HARDWARE ---
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000); 

    pinMode(PIN_DIR_L1, OUTPUT);
    pinMode(PIN_DIR_L2, OUTPUT);
    pinMode(PIN_DIR_R1, OUTPUT);
    pinMode(PIN_DIR_R2, OUTPUT);
    
    stopMotors();

    pinMode(PIN_ENC_L_A, INPUT_PULLUP);
    pinMode(PIN_ENC_L_B, INPUT_PULLUP);
    pinMode(PIN_ENC_R_A, INPUT_PULLUP);
    pinMode(PIN_ENC_R_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isr_enc_l, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isr_enc_r, CHANGE);

    ledcAttach(PIN_PWM_L, PWM_FREQ, PWM_RES);
    ledcAttach(PIN_PWM_R, PWM_FREQ, PWM_RES);

    MPU6050_init();
    posr = 0; posl = 0; 
}

void loop() {
    // 1. GESTIÓN DE CLIENTE WI-FI
    if (!client.connected()) {
        client = server.accept(); 
        if (client) {
            Serial.println("Cliente conectado");
        }
    }

    if (client.connected()) {
        if (client.available()) {
            uint8_t header = client.read();
            
            if (header == 0xAA) {
                if (client.available() >= 32) {
                    float buffer[8];
                    client.readBytes((char*)buffer, 32);
                    
                    k1  = buffer[0];
                    k2  = buffer[1];
                    k3  = buffer[2];
                    k4  = buffer[3];
                    kp  = buffer[4];
                    kv  = buffer[5];
                    Ts_received = buffer[6]; 
                    vd  = buffer[7];
                    
                    running = true;
                    Serial.println("Nuevos parámetros recibidos.");
                }
            }
            else if (header == 0xAB) {
                running = false;
                stopMotors();
                Serial.println("STOP recibido.");
            }
        }
    }

    // 2. LAZO DE CONTROL
    unsigned long currentMicros = micros();
    if (currentMicros - previousMicros >= 10000) { 
        previousMicros = currentMicros;

        Sl = map(analogRead(PIN_SENSOR_L), 0, 4095, 0, 255);
        Sr = map(analogRead(PIN_SENSOR_R), 0, 4095, 0, 255);
        MPU6050_read_Ax();
        MPU6050_read_Gy();

        theta = Sr - Sl;
        if (theta > 160.0) theta = 160.0;
        if (theta < -160.0) theta = -160.0;
        theta = -0.3 * theta / 160.0;

        long current_posr = posr;
        long current_posl = posl;
        incr = current_posr - posr_prev; posr_prev = current_posr;
        incl = current_posl - posl_prev; posl_prev = current_posl;
        omegar = incr * esc * iTs;
        omegal = incl * esc * iTs;

        Xa = -Ax_raw * accel_factor;
        Yg = Gy_raw * gyro_factor * deg_2_rad;
        accelx = Xa * pi_s2;
        angulox = c1 * (angulox_1 + Yg * Ts) + c2 * accelx;
        angulox_1 = angulox;
        alpha = -angulox - calpha;
        if (alpha >= alphaM) alpha = alphaM;
        if (alpha <= -alphaM) alpha = -alphaM;

        if (running) {
            v = (omegar + omegal) * R / 2.0;
            thetap = (omegar - omegal) * R / (2.0 * b);
            thetatil = theta - thetad;
            alphap = (alpha - alpha_1) * iTs;
            alpha_1 = alpha;
            vtil = v - vd;

            if ((intvtil < v2tauM) && (intvtil > -v2tauM)) intvtil += Ts * vtil;
            else {
                if (intvtil >= v2tauM) intvtil = 0.95 * v2tauM;
                if (intvtil <= -v2tauM) intvtil = -0.95 * v2tauM;
            }

            taua = (-kv * thetap - kp * thetatil) * 2.0 * b / R;
            u = k1 * alphap + k2 * alpha + k3 * vtil + k4 * intvtil;
            if (u >= v2tauM) u = v2tauM;
            if (u <= -v2tauM) u = -v2tauM;

            taur = (taua + u) / 2.0;
            taul = (-taua + u) / 2.0;

            // --- Salida Motor Izquierdo ---
            ul = taul * Rasnkm + nkm * omegal;
            if (ul >= uNM) ul = uNM; if (ul <= -uNM) ul = -uNM;
            
            int pwm_l = (int)(escs * fabs(ul));
            if (pwm_l > 0) pwm_l += 50; // Compensación de zona muerta aumentada
            uWl = (uint8_t)(pwm_l > 255 ? 255 : pwm_l);

            if (ul >= 0) { 
                digitalWrite(PIN_DIR_L1, LOW); digitalWrite(PIN_DIR_L2, HIGH);
            } else {       
                digitalWrite(PIN_DIR_L1, HIGH); digitalWrite(PIN_DIR_L2, LOW);
            }
            ledcWrite(PIN_PWM_L, uWl);

            // --- Salida Motor Derecho ---
            ur = taur * Rasnkm + nkm * omegar;
            if (ur >= uNM) ur = uNM; if (ur <= -uNM) ur = -uNM;
            
            int pwm_r = (int)(escs * fabs(ur));
            if (pwm_r > 0) pwm_r += 50; // Compensación de zona muerta aumentada
            uWr = (uint8_t)(pwm_r > 255 ? 255 : pwm_r);

            if (ur >= 0) { 
                digitalWrite(PIN_DIR_R1, HIGH); digitalWrite(PIN_DIR_R2, LOW); // Dirección invertida
            } else {       
                digitalWrite(PIN_DIR_R1, LOW); digitalWrite(PIN_DIR_R2, HIGH);
            }
            ledcWrite(PIN_PWM_R, uWr);

        } else {
            stopMotors();
            ul = 0; ur = 0; 
        }

        // Impresión en el Monitor Serie para calibrar calpha
        Serial.print("Angulo Alpha: ");
        Serial.println(alpha);

        // 3. ENVÍO DE DATOS AL PC
        if (client.connected()) {
            datos_enviar[0] = vd;
            datos_enviar[1] = v;
            datos_enviar[2] = theta;
            datos_enviar[3] = alpha;
            datos_enviar[4] = omegal;
            datos_enviar[5] = ul;
            datos_enviar[6] = omegar;
            datos_enviar[7] = ur;

            client.write((uint8_t*)datos_enviar, sizeof(datos_enviar));
        }

        t = t + Ts;
    }
}