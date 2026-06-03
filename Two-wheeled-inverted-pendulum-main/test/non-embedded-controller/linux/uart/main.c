#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>

#define SERIAL_PORT "/dev/ttyUSB1"

// Velocidad traslacional deseada en m/s
#define vd 0.0

// Ganancias
#define k1 0.07
#define k2 1.2
#define k3 0.90
#define k4 1.6

#define kp 0.0
#define kv 0.0

// Parámetros, rangos y factores
#define tauM    0.3
#define alphaM  1.4
#define omegaM  20.0
#define uM      11.0
#define uNM     11.0

#define Ts      0.01
#define ppr     48.0
#define pi_     3.141593
#define pi_s2   1.570796
#define Ra      3.0
#define NR      34.0
#define R       0.033
#define km      0.0008
#define Cz      0.0485
#define Mp      0.4580
#define b       0.09

#define calpha 0.07

#define deg_2_rad           pi_/180 
#define rad_2_deg           180/pi_
#define accel_div_factor    16384.0
#define gyro_div_factor     131.0
#define accel_factor        1/accel_div_factor
#define gyro_factor         1/gyro_div_factor

#define c1 0.993

// Function prototypes
int8_t v_to_pwm(float voltage);
int setup_serial_port(const char *portname);

// Global variables
unsigned char flagcom = 0, flagfile = 0, signo_sal, pwm, posr, posl, ang;
int8_t uWr, uWl;
uint8_t Sr, Sl;
signed char incr, incl;
signed short int Ax, Gy;
float Xa = 0, Yg = 0, alpha = 0;
float accelx = 0.15, angulox = 0.15, angulox_1 = 0, c2 = 1 - c1;
float t = 0, iTs = 1/Ts, esc = pi_/(2*ppr*NR), escs = 127.0/uM;
float omegar = 0, omegal = 0;
float ur = 0, ul = 0;
float v = 0, alpha_1 = 0, theta = 0, thetad = 0, thetatil = 0, taua, u;
float alphap = 0, vtil = 0, thetap = 0, taur = 0, taul = 0; 
float Rasnkm = Ra/(NR*km), nkm = NR*km, intvtil = 0.0, v2tauM = 2*tauM;

int main() {
    int serial_port;
    FILE *fp;
    
    if((fp = fopen("datos.txt", "w+")) == NULL) {
        printf("No se puede abrir el archivo.\n");
        exit(1);
    }
    
    // Open serial port (replace "/dev/ttyS0" with your actual port)
    serial_port = setup_serial_port(SERIAL_PORT);
    if (serial_port < 0) {
        printf("Error opening serial port\n");
        return 1;
    }
    
    // Main loop
    while(1) {
        unsigned char recibido;
        int n;
        
        // Read data from serial port
        while((n = read(serial_port, &recibido, 1)) > 0) {
            if(flagcom != 0)
                flagcom++;
                
            if((recibido == 0xAA) && (flagcom == 0)) {
                flagcom = 1;
            }
            
            if(flagcom == 2) {
                posr = recibido;
                incr = (int8_t)posr;
            }
            
            if(flagcom == 3) {
                posl = recibido;
                incl = (int8_t)posl;
            } 
            
            if(flagcom == 4) {
                Ax = recibido;
                Ax = (int16_t)Ax << 8;
            } 
            
            if(flagcom == 5) {
                Ax = Ax + recibido;
            } 
            
            if(flagcom == 6) {
                Gy = recibido;
                Gy = (int16_t)Gy << 8;
            }     
            
            if(flagcom == 7) {
                Gy = Gy + recibido;
            }         
            
            if(flagcom == 8) {
                Sr = recibido;
            }         
            
            if(flagcom == 9) {
                Sl = recibido;
                theta = Sr - Sl;
                theta = fmax(fmin(theta, 160), -160);
                theta = -0.3 * theta / 160.0;

                omegar = incr * esc * iTs;
                omegal = incl * esc * iTs;
                Xa = -Ax * accel_factor;
                Yg = Gy * gyro_factor * deg_2_rad;
                accelx = Xa * pi_s2;
                angulox = c1 * (angulox_1 + Yg * Ts) + c2 * accelx;
                angulox_1 = angulox;
                alpha = -angulox - calpha;
                alpha = fmax(fmin(alpha, alphaM), -alphaM);

                v = (omegar + omegal) * R / 2;
                thetap = (omegar - omegal) * R / (2 * b);

                thetatil = theta - thetad;
                alphap = (alpha - alpha_1) * iTs;
                alpha_1 = alpha;
                vtil = v - vd;

                if ((intvtil < v2tauM) && (intvtil > -v2tauM)) intvtil += Ts * vtil;

                taua = (-kv * thetap - kp * thetatil) * 2 * b / R;
                u = k1 * alphap + k2 * alpha + k3 * vtil + k4 * intvtil;
                u = fmax(fmin(u, v2tauM), -v2tauM);

                taur = (taua + u) / 2.0;
                taul = (-taua + u) / 2.0;

                ul = taul * Rasnkm + nkm * omegal;
                //ul=uNM*sin(6.28*t);
                //ul=uNM*sin(0.628*t);
                //ul = 5.0;
                ul = fmax(fmin(ul, uNM), -uNM);
                uWl = v_to_pwm(ul);

                ur = taur * Rasnkm + nkm * omegar;
                //ur=uNM*sin(6.28*t);
                //ur=uNM*sin(0.628*t);
                ur = fmax(fmin(ur, uNM), -uNM);
                uWr = v_to_pwm(ur);

                int8_t pwmS[2];
                pwmS[0] = uWl;
                pwmS[1] = uWr;
                write(serial_port, &pwmS, 2);

                // Imprimiendo en pantalla
                printf("%.2f\t%.2f\t%.2f\t%i\t%i\n", t, alpha, theta, uWl, uWr);
                
                // Escribir datos en el archivo
                fprintf(fp, "%.2f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", 
                       t, alpha, theta, v, vd, omegal, omegar, ul, ur);
                
                t = t + Ts;
                flagcom = 0;
            }
        }
        
        // Handle read errors
        if (n < 0) {
            printf("Error reading from serial port\n");
            break;
        }
    }
    
    fclose(fp);
    close(serial_port);
    return 0;
}

int8_t v_to_pwm(float voltage) {
    float pwmf;
    pwmf = fmax(fmin(escs * voltage, 127.0), -128.0);
    pwm = (int8_t)pwmf;
    return pwm;
}

int setup_serial_port(const char *portname) {
    int serial_port = open(portname, O_RDWR | O_NOCTTY);
    if (serial_port < 0) {
        perror("Error opening serial port");
        return -1;
    }

    struct termios tty;
    if(tcgetattr(serial_port, &tty) != 0) {
        perror("Error getting serial port attributes");
        return -1;
    }

    // Set baud rate
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // 8N1 configuration
    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;         // Clear size bits
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    // Non-canonical mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    // Raw output
    tty.c_oflag &= ~OPOST;

    // Set read timeout (100ms)
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if(tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        perror("Error setting serial port attributes");
        return -1;
    }

    return serial_port;
}
