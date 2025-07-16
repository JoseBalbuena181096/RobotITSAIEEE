#ifndef SENSORES_LINEA_H
#define SENSORES_LINEA_H

void setupSensoresLinea();
void leerSensores();
int calcularCentroide();
void controlPD(int centroide);

#endif // SENSORES_LINEA_H
