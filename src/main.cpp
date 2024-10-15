#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "knn_model.h"  // Archivo que contiene los datos de entrenamiento (ya normalizados)

Adafruit_MPU6050 mpu;

const int k = 5;  // Número de vecinos más cercanos
const int num_clases = 6;  // Número de clases
float current_data[3];  // Almacena los datos actuales del acelerómetro

// Pines de los LEDs
const int ledArriba = 2;
const int ledAbajo = 4;
const int ledIzquierda = 19;
const int ledDerecha = 23;
const int ledAdelante = 5;
const int ledAtras = 18;

// Función para inicializar los LEDs
void inicializarLeds() {
  pinMode(ledArriba, OUTPUT);
  pinMode(ledAbajo, OUTPUT);
  pinMode(ledIzquierda, OUTPUT);
  pinMode(ledDerecha, OUTPUT);
  pinMode(ledAdelante, OUTPUT);
  pinMode(ledAtras, OUTPUT);

  // Apagar todos los LEDs al inicio
  digitalWrite(ledArriba, LOW);
  digitalWrite(ledAbajo, LOW);
  digitalWrite(ledIzquierda, LOW);
  digitalWrite(ledDerecha, LOW);
  digitalWrite(ledAdelante, LOW);
  digitalWrite(ledAtras, LOW);
}

// Función para apagar todos los LEDs
void apagarTodosLosLeds() {
  digitalWrite(ledArriba, LOW);
  digitalWrite(ledAbajo, LOW);
  digitalWrite(ledIzquierda, LOW);
  digitalWrite(ledDerecha, LOW);
  digitalWrite(ledAdelante, LOW);
  digitalWrite(ledAtras, LOW);
}

// Función para calcular la distancia Euclidiana (los datos ya están normalizados)
float calcularDistancia(float* data1, float* data2) {
  float sum = 0.0;
  for (int i = 0; i < 3; i++) {
    sum += pow(data1[i] - data2[i], 2);  // Distancia Euclidiana simple
  }
  return sqrt(sum);
}

// Implementación del algoritmo KNN con ponderación de distancias
byte knn_predict(float* input_data, int k) {
  int total_muestras = 500 * num_clases;
  float closest_distances[k];
  byte closest_labels[k];

  // Inicializar las distancias más cercanas con valores altos
  for (int i = 0; i < k; i++) {
    closest_distances[i] = 1e30;  // Usamos un valor grande
  }

  // Calcular las distancias y encontrar los k vecinos más cercanos
  for (int i = 0; i < total_muestras; i++) {
    float distancia = calcularDistancia(input_data, training_data[i]);

    // Si la distancia actual es menor que alguna de las k más grandes, la reemplazamos
    for (int j = 0; j < k; j++) {
      if (distancia < closest_distances[j]) {
        for (int l = k - 1; l > j; l--) {
          closest_distances[l] = closest_distances[l - 1];
          closest_labels[l] = closest_labels[l - 1];
        }
        closest_distances[j] = distancia;
        closest_labels[j] = training_labels[i];
        break;
      }
    }
  }

  // Votación ponderada por distancia
  float votes[num_clases] = {0};
  for (int i = 0; i < k; i++) {
    votes[closest_labels[i]] += 1.0 / (closest_distances[i] + 1e-5);  // Inversa de la distancia
  }

  // Encontrar la clase con más votos
  float max_votes = 0;
  byte predicted_label = 0;
  for (int i = 0; i < num_clases; i++) {
    if (votes[i] > max_votes) {
      max_votes = votes[i];
      predicted_label = i;
    }
  }

  return predicted_label;
}

// Función para encender el LED según la predicción
void encenderLedSegunPrediccion(byte prediccion) {
  // Apagar todos los LEDs antes de encender el correcto
  apagarTodosLosLeds();

  switch (prediccion) {
    case 0:
      digitalWrite(ledAbajo, HIGH);
      break;
    case 1:
      digitalWrite(ledArriba, HIGH);
      break;
    case 2:
      digitalWrite(ledIzquierda, HIGH);
      break;
    case 3:
      digitalWrite(ledDerecha, HIGH);
      break;
    case 4:
      digitalWrite(ledAdelante, HIGH);
      break;
    case 5:
      digitalWrite(ledAtras, HIGH);
      break;
    default:
      // No hacer nada si la predicción es inválida
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // Inicializar el MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 no detectado.");
    while (1);
  }
  
  // Inicializar los LEDs
  inicializarLeds();
  
  Serial.println("MPU6050 detectado. Iniciando predicciones con KNN.");
}

void loop() {
  // Obtener los valores actuales del acelerómetro
  sensors_event_t accel;
  mpu.getAccelerometerSensor()->getEvent(&accel);

  // Guardar los datos actuales del acelerómetro
  current_data[0] = accel.acceleration.x;
  current_data[1] = accel.acceleration.y;
  current_data[2] = accel.acceleration.z;

  // Realizar la predicción con KNN
  byte prediccion = knn_predict(current_data, k);

  // Mostrar la predicción
  Serial.print("Predicción: ");
  Serial.println(prediccion);

  // Encender el LED correspondiente
  encenderLedSegunPrediccion(prediccion);

  delay(2000);  // Esperar 2 segundos antes de la siguiente predicción
}
