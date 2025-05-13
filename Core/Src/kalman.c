/*
 * kalman.c
 *
 *  Created on: Apr 15, 2025
 *      Author: tobi
 */

// Generiert mit Hilfe von ChatGPT / K!mpuls



#include <stdio.h>
#include <math.h>

#define DT 1./100. // Zeitintervall in Sekunden

// Kalman-Filter Variablen
#define Q_ANGLE   0.1   // Prozessrauschen
#define Q_BIAS    0.005  // Bias-Rauschen
#define R_MEASURE  0.1     // Messrauschen

static float angle = 0;       // Geschätzter Winkel
static float bias = 0;        // Geschätzter Bias
static float rate = 0;        // Rate vom Gyroskop
static float P[2][2] = {{1, 0}, {0, 1}}; // Fehlerkovarianzmatrix

// Kalman-Filter Funktion
float angle_filter(float accel_angle, float gyro_rate)
{
    // Vorhersage
    rate = gyro_rate - bias;
    angle += rate * DT;

    // Fehlerkovarianzmatrix aktualisieren
    P[0][0] += DT * (DT * P[1][1] - P[0][1] - P[1][0] + Q_ANGLE);
    P[0][1] -= DT * P[1][1];
    P[1][0] -= DT * P[1][1];
    P[1][1] += Q_BIAS * DT;

    // Kalman-Gewichte berechnen
    float S = P[0][0] + R_MEASURE; // Innovationskovarianz
    float K[2]; // Kalman-Gewichte

    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Korrigierter Wert
    float y = accel_angle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    // Fehlerkovarianzmatrix aktualisieren
    float P00_temp = P[0][0];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P[0][1];

    return angle;
}
