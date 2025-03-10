*
 * Código para Robot Autónomo con ESP32
 * Este programa implementa un robot que detecta y evita obstáculos automáticamente
 * utilizando sensores de distancia VL53L0X y muestra mensajes en una matriz LED.
 */

// ===== INCLUSIÓN DE LIBRERÍAS =====
#include <Wire.h>                // Librería para comunicación I2C con el sensor
#include <VL53L0X.h>             // Librería para el sensor de distancia VL53L0X
#include <MD_Parola.h>           // Librería para controlar matrices LED de forma sencilla
#include <MD_MAX72XX.h>          // Librería base para matrices LED MAX7219/MAX7221
#include <SPI.h>                 // Librería para comunicación SPI (usada por la matriz LED)
#include <esp_system.h>          // Librería de sistema para ESP32 (para reinicio)
#include <vector>                // Librería STL para usar vectores dinámicos en C++

// ===== CONFIGURACIÓN DE LA MATRIZ LED =====
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW  // Tipo de hardware (FC16)
#define MAX_DEVICES 4                      // Número de módulos MAX7219 conectados en cascada
#define DATA_PIN 23                        // Pin MOSI para datos SPI
#define CS_PIN 5                           // Pin Chip Select para SPI
#define CLK_PIN 18                         // Pin reloj para SPI

// ===== CLASE PARA GESTIÓN DE SENSORES DE DISTANCIA =====
class DistanceSensorManager {
private:
    std::vector<VL53L0X*> sensors;         // Vector que almacena punteros a los sensores
    std::vector<int> sensorAddresses;      // Vector que almacena las direcciones I2C de los sensores
    
public:
    // Constructor de la clase
    DistanceSensorManager() {
        // No hace nada especial, solo inicializa los vectores vacíos
    }
    
    // Destructor - libera la memoria de todos los sensores creados
    ~DistanceSensorManager() {
        for (auto sensor : sensors) {
            delete sensor;
        }
    }
    
    // Método para añadir un nuevo sensor al sistema
    // xshutPin: Pin para control de encendido/apagado (opcional)
    // address: Nueva dirección I2C para el sensor (opcional)
    bool addSensor(int xshutPin = -1, int address = 0) {
        // Creamos un nuevo objeto sensor
        VL53L0X* newSensor = new VL53L0X();
        
        // Si se proporcionó un pin XSHUT, lo configuramos para gestionar el sensor
        if (xshutPin >= 0) {
            pinMode(xshutPin, OUTPUT);
            digitalWrite(xshutPin, LOW);  // Apagamos el sensor temporalmente
            delay(10);
            digitalWrite(xshutPin, HIGH); // Encendemos el sensor
            delay(10);
            
            // Si se solicitó cambiar la dirección, lo hacemos
            if (address > 0) {
                newSensor->setAddress(address);
                sensorAddresses.push_back(address);
            }
        }
        
        // Inicializamos el sensor
        if (!newSensor->init()) {
            Serial.println("Error al inicializar el sensor VL53L0X");
            delete newSensor;  // Liberamos memoria si hubo error
            return false;
        }
        
        // Configuramos parámetros del sensor
        newSensor->setTimeout(500);     // Tiempo máximo de espera: 500ms
        newSensor->startContinuous();   // Modo de medición continua
        
        // Añadimos el sensor a nuestro vector
        sensors.push_back(newSensor);
        return true;
    }
    
    // Método para obtener la distancia medida por un sensor específico
    int getDistance(int sensorIndex = 0) {
        // Verificamos que el índice sea válido
        if (sensorIndex < 0 || sensorIndex >= sensors.size()) {
            return -1; // Indicamos error con valor negativo
        }
        
        // Leemos la distancia en milímetros
        int distance = sensors[sensorIndex]->readRangeContinuousMillimeters();
        
        // Verificamos si ocurrió un timeout (error de lectura)
        if (sensors[sensorIndex]->timeoutOccurred()) {
            Serial.println("Error de timeout en sensor " + String(sensorIndex));
            return -1;
        }
        
        return distance;
    }
    
    // Método para obtener mediciones de todos los sensores a la vez
    std::vector<int> getAllDistances() {
        std::vector<int> distances;
        
        for (int i = 0; i < sensors.size(); i++) {
            distances.push_back(getDistance(i));
        }
        
        return distances;
    }
    
    // Método para obtener la distancia mínima de todos los sensores
    // Útil para detectar el obstáculo más cercano
    int getMinDistance() {
        if (sensors.empty()) {
            return -1;  // No hay sensores configurados
        }
        
        int minDist = getDistance(0);
        if (minDist < 0) minDist = 999999;  // Si hay error, valor muy alto
        
        // Buscamos la medida más pequeña entre todos los sensores
        for (int i = 1; i < sensors.size(); i++) {
            int dist = getDistance(i);
            if (dist > 0 && dist < minDist) {
                minDist = dist;
            }
        }
        
        return (minDist == 999999) ? -1 : minDist;
    }
    
    // Método para obtener el número de sensores configurados
    int getSensorCount() {
        return sensors.size();
    }
};

// ===== CLASE PARA CONTROL DE MOVIMIENTO DEL ROBOT =====
class MotionController {
private:
    // Pines para control de motores
    int motor1Pin1, motor1Pin2, enablePin1;  // Motor 1: IN1, IN2, ENA
    int motor2Pin1, motor2Pin2, enablePin2;  // Motor 2: IN3, IN4, ENB
    
    // Configuración de PWM para control de velocidad
    int pwmChannel1, pwmChannel2;            // Canales PWM del ESP32
    int pwmFrequency;                        // Frecuencia PWM en Hz
    int pwmResolution;                       // Resolución PWM en bits
    
    // Velocidades actuales
    int currentSpeed1, currentSpeed2;        // Velocidades actuales de ambos motores
    
public:
    // Constructor que inicializa todos los parámetros
    MotionController(
        int m1p1, int m1p2, int en1,         // Pines para motor 1
        int m2p1, int m2p2, int en2,         // Pines para motor 2
        int pwmCh1 = 0, int pwmCh2 = 1,      // Canales PWM predeterminados
        int freq = 30000, int res = 8        // Configuración PWM predeterminada
    ) : motor1Pin1(m1p1), motor1Pin2(m1p2), enablePin1(en1),
        motor2Pin1(m2p1), motor2Pin2(m2p2), enablePin2(en2),
        pwmChannel1(pwmCh1), pwmChannel2(pwmCh2),
        pwmFrequency(freq), pwmResolution(res),
        currentSpeed1(0), currentSpeed2(0) {
        
        // Configuramos los pines como salidas
        pinMode(motor1Pin1, OUTPUT);
        pinMode(motor1Pin2, OUTPUT);
        pinMode(enablePin1, OUTPUT);
        pinMode(motor2Pin1, OUTPUT);
        pinMode(motor2Pin2, OUTPUT);
        pinMode(enablePin2, OUTPUT);
        
        // Configuramos los canales PWM
        ledcSetup(pwmChannel1, pwmFrequency, pwmResolution);
        ledcSetup(pwmChannel2, pwmFrequency, pwmResolution);
        
        // Asociamos los pines de habilitación a los canales PWM
        ledcAttachPin(enablePin1, pwmChannel1);
        ledcAttachPin(enablePin2, pwmChannel2);
        
        // Aseguramos que los motores están detenidos al iniciar
        stop();
    }
    
    // Método para mover el robot hacia adelante
    void moveForward(int speed1, int speed2) {
        // Motor 1: sentido horario
        digitalWrite(motor1Pin1, HIGH);
        digitalWrite(motor1Pin2, LOW);
        // Motor 2: sentido horario
        digitalWrite(motor2Pin1, HIGH);
        digitalWrite(motor2Pin2, LOW);
        
        // Establecemos velocidades mediante PWM
        ledcWrite(pwmChannel1, speed1);
        ledcWrite(pwmChannel2, speed2);
        
        // Guardamos las velocidades actuales
        currentSpeed1 = speed1;
        currentSpeed2 = speed2;
    }
    
    // Método para mover el robot hacia atrás
    void moveBackward(int speed1, int speed2) {
        // Motor 1: sentido antihorario
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, HIGH);
        // Motor 2: sentido antihorario
        digitalWrite(motor2Pin1, LOW);
        digitalWrite(motor2Pin2, HIGH);
        
        // Establecemos velocidades mediante PWM
        ledcWrite(pwmChannel1, speed1);
        ledcWrite(pwmChannel2, speed2);
        
        // Guardamos las velocidades actuales
        currentSpeed1 = speed1;
        currentSpeed2 = speed2;
    }
    
    // Método para detener completamente el robot
    void stop() {
        // Ambos pines en LOW para cada motor los detiene
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, LOW);
        digitalWrite(motor2Pin1, LOW);
        digitalWrite(motor2Pin2, LOW);
        
        // Ponemos PWM a 0 (innecesario pero para asegurar)
        ledcWrite(pwmChannel1, 0);
        ledcWrite(pwmChannel2, 0);
        
        // Actualizamos velocidades
        currentSpeed1 = 0;
        currentSpeed2 = 0;
    }
    
    // Método para girar a la derecha (pivote sobre rueda derecha)
    void turnRight(int speed) {
        // Motor izquierdo avanza
        digitalWrite(motor1Pin1, HIGH);
        digitalWrite(motor1Pin2, LOW);
        // Motor derecho retrocede
        digitalWrite(motor2Pin1, LOW);
        digitalWrite(motor2Pin2, HIGH);
        
        // Misma velocidad para ambos motores
        ledcWrite(pwmChannel1, speed);
        ledcWrite(pwmChannel2, speed);
        
        // Actualizamos velocidades
        currentSpeed1 = speed;
        currentSpeed2 = speed;
    }
    
    // Método para girar a la izquierda (pivote sobre rueda izquierda)
    void turnLeft(int speed) {
        // Motor izquierdo retrocede
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, HIGH);
        // Motor derecho avanza
        digitalWrite(motor2Pin1, HIGH);
        digitalWrite(motor2Pin2, LOW);
        
        // Misma velocidad para ambos motores
        ledcWrite(pwmChannel1, speed);
        ledcWrite(pwmChannel2, speed);
        
        // Actualizamos velocidades
        currentSpeed1 = speed;
        currentSpeed2 = speed;
    }
    
    // Método para obtener la velocidad actual del motor 1
    int getSpeed1() const {
        return currentSpeed1;
    }
    
    // Método para obtener la velocidad actual del motor 2
    int getSpeed2() const {
        return currentSpeed2;
    }
};

// ===== OBJETOS GLOBALES =====
// Creamos objeto para la matriz LED
MD_Parola matriz = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

// Objetos de nuestras clases para gestión de sensores y motores
DistanceSensorManager sensorManager;
MotionController motionController(
    27, 26, 14,  // Motor 1: IN1, IN2, ENA - pines del ESP32
    25, 33, 32,  // Motor 2: IN3, IN4, ENB - pines del ESP32
    0, 1,        // Canales PWM 0 y 1 del ESP32
    30000, 8     // Frecuencia PWM: 30kHz, Resolución: 8 bits (0-255)
);

// ===== CONSTANTES Y VARIABLES GLOBALES =====
// Umbral de distancia para detección de obstáculos (en milímetros)
const int DISTANCIA_MINIMA = 300;  // 30 centímetros

// Variables para el control autónomo
unsigned long ultimoTiempoDecision = 0;   // Último momento en que se tomó una decisión
int tiempoGiro = 700;                     // Tiempo en ms que el robot gira al detectar un obstáculo
bool enModoEvitarObstaculo = false;       // Indica si está en medio de una maniobra
unsigned long tiempoFinManiobra = 0;      // Tiempo en que debe finalizar la maniobra actual

// Variables para el estado del mensaje en la matriz LED
bool mensajeCambiado = false;                        // Indica si ya se cambió el mensaje
unsigned long tiempoReinicio = 0;                    // Tiempo del próximo reinicio
const unsigned long TIEMPO_ENTRE_REINICIOS = 60000;  // 60 segundos entre reinicios (1 minuto)

// Mensajes para mostrar en la matriz LED
const char* MENSAJE_NORMAL = "ROBOT AUTONOMO";       // Mensaje en operación normal
const char* MENSAJE_OBSTACULO = "EVITANDO OBSTACULO"; // Mensaje al detectar obstáculo
const char* MENSAJE_ACTUAL = MENSAJE_NORMAL;         // Mensaje que se muestra actualmente

// ===== CONFIGURACIÓN INICIAL =====
void setup() {
  // Iniciamos comunicación serial para depuración
  Serial.begin(115200);
  
  // Iniciamos comunicación I2C para el sensor
  Wire.begin();
  
  // Agregamos un sensor de distancia
  if (!sensorManager.addSensor()) {
    Serial.println("Error al inicializar el sensor principal");
    while (1) {} // Detener ejecución si hay error (bucle infinito)
  }
  
  Serial.println("Sensor de distancia inicializado correctamente");
  
  // Inicialización de la matriz LED
  matriz.begin();                      // Inicializa la matriz
  matriz.setIntensity(5);              // Brillo medio (0-15)
  matriz.displayClear();               // Limpia la pantalla
  matriz.displayText(MENSAJE_NORMAL, PA_CENTER, 100, 1000, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
  // Parámetros: texto, alineación, velocidad, pausa, efecto entrada, efecto salida
  
  // Mensaje de inicio
  Serial.println("Robot autónomo con matriz LED iniciado");
  
  // Pequeña pausa antes de comenzar
  delay(1000);
  
  // Iniciar tiempo para reinicio periódico
  tiempoReinicio = millis() + TIEMPO_ENTRE_REINICIOS;
}

// ===== BUCLE PRINCIPAL =====
void loop() {
  // Actualizar la animación de la matriz LED
  actualizarMatriz();
  
  // Leer la distancia desde el sensor
  int distancia = sensorManager.getDistance(0);
  
  // Mostrar la distancia por el puerto serie (si es válida)
  if (distancia > 0) {
    Serial.print("Distancia: ");
    Serial.print(distancia);
    Serial.println(" mm");
  }

  // ===== LÓGICA PARA NAVEGACIÓN AUTÓNOMA =====
  if (enModoEvitarObstaculo) {
    // Si estamos en medio de una maniobra para evitar obstáculo
    if (millis() < tiempoFinManiobra) {
      // Continuamos con la maniobra en curso hasta que se cumpla el tiempo
      if (!mensajeCambiado) {
        cambiarMensaje(MENSAJE_OBSTACULO);
        mensajeCambiado = true;
      }
      return; // Salimos del loop para no ejecutar el resto del código
    } else {
      // Maniobra completada, volvemos a modo normal
      enModoEvitarObstaculo = false;
      motionController.moveForward(180, 180); // Velocidad media
      cambiarMensaje(MENSAJE_NORMAL);
      mensajeCambiado = false;
      Serial.println("Continuando ruta");
    }
  }
  
  // Verificar si hay obstáculo cercano
  if (distancia > 0 && distancia < DISTANCIA_MINIMA && !enModoEvitarObstaculo) {
    // Se detectó un obstáculo a menos de la distancia mínima
    Serial.println("¡Obstáculo detectado! Evitando...");
    cambiarMensaje(MENSAJE_OBSTACULO);
    
    // Secuencia para evitar obstáculo:
    
    // 1. Detenemos primero
    motionController.stop();
    delay(200);  // Pequeña pausa para estabilizar
    
    // 2. Retrocedemos un poco
    motionController.moveBackward(200, 200);
    delay(500);  // Retrocede medio segundo
    motionController.stop();
    delay(200);  // Pequeña pausa para estabilizar
    
    // 3. Giramos para evitar el obstáculo (aleatoriamente elegimos dirección)
    if (random(2) == 0) {  // 50% probabilidad de cada dirección
      Serial.println("Girando a la derecha");
      motionController.turnRight(200);
    } else {
      Serial.println("Girando a la izquierda");
      motionController.turnLeft(200);
    }
    
    // 4. Configurar el tiempo de finalización de la maniobra
    enModoEvitarObstaculo = true;
    tiempoFinManiobra = millis() + tiempoGiro;  // Girará durante el tiempo especificado
  } 
  else if (!enModoEvitarObstaculo) {
    // Si no hay obstáculos y no estamos en maniobra, seguimos adelante
    motionController.moveForward(180, 180);  // Velocidad media
    
    // Actualizamos mensaje si es necesario
    if (MENSAJE_ACTUAL != MENSAJE_NORMAL) {
      cambiarMensaje(MENSAJE_NORMAL);
    }
  }
  
  // ===== VERIFICAR REINICIO PERIÓDICO =====
  // Verificar si es hora de reiniciar el ESP32 para evitar bloqueos
  if (millis() > tiempoReinicio) {
    Serial.println("Reiniciando ESP32 para mantenimiento...");
    delay(500);  // Pequeña pausa antes de reiniciar
    esp_restart(); // Función de la librería esp_system.h que reinicia el ESP32
  }
  
  // Pequeña pausa para no saturar el bucle y el puerto serie
  delay(50);
}

// ===== FUNCIONES AUXILIARES =====
// Función para actualizar el mensaje en la matriz
void actualizarMatriz() {
  if (matriz.displayAnimate()) {  // Actualiza la animación y devuelve true si terminó
    matriz.displayReset();        // Reinicia la animación cuando termina
  }
}

// Función para cambiar el mensaje en la matriz
void cambiarMensaje(const char* nuevoMensaje) {
  MENSAJE_ACTUAL = nuevoMensaje;
  matriz.displayClear();          // Limpia la pantalla
  matriz.displayText(nuevoMensaje, PA_CENTER, 100, 1000, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
  Serial.print("Cambiando mensaje a: ");
  Serial.println(nuevoMensaje);
}