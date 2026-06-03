#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <math.h>

#define PACKET_SIZE 3
#define BUF_SIZE 16

// Constants and definitions
#define k1                 0.1096
#define k2                 0.9221
#define k3                 0.7556
#define k4                 1.1476

#define kp                 0.08
#define kv                 0.008

#define vd 0.0  
#define tauM 0.1654
#define alphaM 1.5
#define omegaM 13.0
#define uM 12.0
#define uNM 12.0
#define Ts 0.01
#define ppr 10.0
#define pi_ 3.141593
#define pi_s2 1.570796
#define Ra 3.0
#define NR 34.0
#define R 0.0335
#define km 0.0008
#define Cz 0.0992
#define Mp 0.3730
#define b 0.09
#define calpha 0.145
#define deg_2_rad (pi_/180)
#define rad_2_deg (180/pi_)
#define accel_div_factor 16384.0
#define gyro_div_factor 131.0
#define accel_factor (1/accel_div_factor)
#define gyro_factor (1/gyro_div_factor)
#define c1 0.993

// Prototypes
unsigned char v_to_pwm(float voltage);

// Global variables
uint8_t flagcom = 0, flagfile = 0, signo_sal, pwm, posr, posl, ang;
uint8_t uWr, uWl, Sr, Sl;
int8_t incr, incl;
int16_t Ax, Gy;
float Xa = 0, Yg = 0, alpha = 0;
float accelx = 0.15, angulox = 0.15, angulox_1 = 0, c2 = 1 - c1;
float t = 0, iTs = 1 / Ts, esc = pi_ / (2 * ppr * NR), escs = 127.0 / uM;
float omegar = 0, omegal = 0;
float ur = 0, ul = 0;
float v = 0, alpha_1 = 0, theta = 0, thetad = 0, thetatil = 0, taua, u;
float alphap = 0, vtil = 0, thetap = 0, taur = 0, taul = 0;
float Rasnkm = Ra / (NR * km), nkm = NR * km, intvtil = 0.0, v2tauM = 2 * tauM;

int main() {
    int sockfd;
    struct sockaddr_in server_addr;
    FILE *fp;
    
    if((fp = fopen("datos.txt", "w+")) == NULL) {
        printf("Cannot open the file.\n");
        exit(1);
    }

    // Initialize socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        perror("Socket creation failed");
        exit(1);
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(4545);
    server_addr.sin_addr.s_addr = inet_addr("192.168.4.1");

    int yes = 1;
    int result = setsockopt(sockfd,
                            IPPROTO_TCP,
                            TCP_NODELAY,
                            (char *) &yes, 
                            sizeof(int));

    if (connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) != 0) {
        perror("Connection with the server failed");
        close(sockfd);
        exit(1);
    }

    uint8_t recibido;
    uint8_t enviar;
    uint8_t buf[16];
    uint8_t packet[9];
    
    while (1) {
        recibido = 0;
        while (1) {
            ssize_t bytes_received = recv(sockfd, &buf, sizeof(buf), 0);
            if (bytes_received <= 0){
                continue;
            }

            for (int i = 0 ; i< bytes_received; i++)
            {
                recibido = buf[i];
                if (flagcom != 0) flagcom++;
                if ((recibido == 0xAA) && (flagcom == 0))
                {
                    packet[0] = 0xAA;
                    flagcom = 1;
                }
                if (flagcom == 2) {
                    packet[1] = recibido;
                    
                    incr = (int8_t)recibido;
                }
                if (flagcom == 3) {
                    packet[2] = recibido;
                    incl = (int8_t)recibido;
                }
                if (flagcom == 4) {
                    packet[3] = recibido;
                    Ax = ((int16_t)recibido) << 8;
                }
                if (flagcom == 5) {
                    packet[4] = recibido;
                    Ax += recibido;
                }
                if (flagcom == 6) {
                    packet[5] = recibido;
                    Gy = ((int16_t)recibido) << 8;
                }
                if (flagcom == 7) {
                    packet[6] = recibido;
                    Gy += recibido;
                }
                if (flagcom == 8) {
                    packet[7] = recibido;
                    Sr = recibido;
                }
                if (flagcom == 9) {
                    packet[8] = recibido;
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
		    //*/
                    int8_t pwmS[2];
                    pwmS[0] = uWl;
                    pwmS[1] = uWr;
                    //pwmS[2] = uWr;
		    /*/
                    int8_t pwmS[2];
                    pwmS[0] = uWl;
                    pwmS[1] = uWr;
		    //*/
                    send(sockfd, &pwmS, sizeof(pwmS), 0);

                    printf("%.2f\t%.2f\t%.2f\t", t, alpha, theta);
                    for(int j = 0 ; j < 9 ; j++){
                        printf("0x%x\t", packet[j]);
                    }
                    printf("sent:0x%x, 0x%x",uWl, uWr);
                    printf("\n");
                    fprintf(fp, "%.2f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", t, alpha, theta, v, vd, omegal, omegar, ul, ur);
                    t += Ts;
                    flagcom = 0;
                }
            }
        }
    }
    fclose(fp);
    close(sockfd);
    return 0;
}

unsigned char v_to_pwm(float voltage) {
    float pwmf = escs * voltage;
    uint8_t pwm_val = (uint8_t)fabs(pwmf);
    if (pwmf < 0) pwm_val += 128;
    return pwm_val;
}


