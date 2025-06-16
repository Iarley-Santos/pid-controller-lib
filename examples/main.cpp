#include "pid_controller.h"

PidController pid(1.0f, 0.5f, 0.1f);

// Motores
const int16_t min_speed = 0;
const uint16_t max_speed = 255;
const int16_t motor_bias = 100;

// Setpoint desejado (ex: ângulo ou velocidade)
float setpoint = 50.0f;

// Simulador de sensor
float feedback = 0.0f;

unsigned long last_time = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("PID Controller Teste Iniciado");
}

void loop() 
{
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0f;  // em segundos

    // Simula leitura de sensor (você trocaria aqui por encoder, imu, etc.)
    feedback += random(-3, 4);  // valor aleatório para simular variação

    // Calcula sinal de controle
    int u = pid.pid_calculation(setpoint, feedback, delta_time);

    // Calcula velocidades dos motores
    int16_t right_speed = pid.right_motor_speed(min_speed, max_speed, motor_bias, u);
    int16_t left_speed  = pid.left_motor_speed(min_speed, max_speed, motor_bias, u);

    // Envia dados para o serial monitor
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" | Feedback: ");
    Serial.print(feedback);
    Serial.print(" | Erro: ");
    Serial.print(setpoint - feedback);
    Serial.print(" | Controle: ");
    Serial.print(u);
    Serial.print(" | R: ");
    Serial.print(right_speed);
    Serial.print(" | L: ");
    Serial.println(left_speed);

    last_time = current_time;
}
