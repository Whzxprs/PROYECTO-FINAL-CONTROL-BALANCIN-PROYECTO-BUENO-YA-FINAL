#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>

// Parameters
#define IP_ADDRESS "192.168.4.1"
#define PORT 4545
#define pi_     3.141593
#define pi_s2   1.570796
#define Ts      0.01
#define R	    0.034
#define omegaM	20.0
#define alphaM	1.4
#define tauM	0.3	// Maximum torque on the wheel [Nm]
#define uM		10.5

uint8_t flagcom = 0, flagfile = 0;
uint8_t vdg, vg, thetag, alphag, omegalg, omegarg, ulg, urg;
// Auxiliary variables
float t = 0, omegar = 0, omegal = 0;
float ur = 0, ul = 0;
float v = 0, theta = 0;
float alpha = 0, vd;
float mv = (2 * omegaM * R) / 255.0, mtheta = (2 * 0.3) / 255.0;
float malpha = (2 * alphaM) / 255.0, momega = (2 * omegaM) / 255.0, mu = (2 * uM) / 255.0;

int main() {
    int sockfd;
    struct sockaddr_in server_addr;
    FILE *fp;

    if ((fp = fopen("datos.txt", "w+")) == NULL) {
        printf("Cannot open file.\n");
        exit(1);
    }

    // Create socket
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation failed");
        fclose(fp);
        return 1;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);

    if (inet_pton(AF_INET, IP_ADDRESS, &server_addr.sin_addr) <= 0) {
        perror("Invalid address or address not supported");
        fclose(fp);
        close(sockfd);
        return 1;
    }

    // Connect to server
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection to server failed");
        fclose(fp);
        close(sockfd);
        return 1;
    }

    uint8_t recibido;
    while (true) {
        // Reset flagcom for each data packet
        recibido = 0;
        while (true) {
            int bytes_received = recv(sockfd, &recibido, sizeof(uint8_t), 0);
            if (bytes_received <= 0) {
                perror("Connection lost or error receiving data");
                fclose(fp);
                close(sockfd);
                return 1;
            } else {
                if (flagcom != 0)
                    flagcom++;
                
                if ((recibido == 0xAA) && (flagcom == 0)) {
                    flagcom = 1;
                }

                // Process received data based on flagcom value
                if (flagcom == 2) vdg = recibido;
                if (flagcom == 3) vg = recibido;
                if (flagcom == 4) thetag = recibido;
                if (flagcom == 5) alphag = recibido;
                if (flagcom == 6) omegalg = recibido;
                if (flagcom == 7) omegarg = recibido;
                if (flagcom == 8) ulg = recibido;
                if (flagcom == 9) {
                    urg = recibido;

                    // Update calculations based on received data
                    vd = (vdg - 127) * mv;
                    v = (vg - 127) * mv;
                    theta = (thetag - 127) * mtheta;
                    alpha = (alphag - 127) * malpha;
                    omegal = (omegalg - 127) * momega;
                    omegar = (omegarg - 127) * momega;
                    ul = (ulg - 127) * mu;
                    ur = (urg - 127) * mu;

                    // Print data to screen
                    printf("%.2f\t%.2f\t%.2f\n", t, alpha, theta);

                    // Write data to file
                    fprintf(fp, "%.2f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
                            t, alpha, theta, v, vd, omegal, omegar, ul, ur);

                    // Increment time by sampling period
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


