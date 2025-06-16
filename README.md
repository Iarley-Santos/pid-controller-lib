# Controlador PID

![Status](https://img.shields.io/badge/status-stable-brightgreen)
![Language](https://img.shields.io/badge/language-C%2B%2B-orange)
![License: MIT](https://img.shields.io/badge/license-MIT-blue)

Biblioteca simples e eficiente de controle PID em C++ para robôs com dois motores (como diferencial ou tração traseira). Ideal para uso em **Arduino**, **ESP32**, e outros sistemas embarcados.

## 🚀 Funcionalidade

Essa biblioteca calcula uma saída de controle PID com base no erro entre o `setpoint` (valor desejado) e o `feedback` (valor atual medido). Essa saída é usada para ajustar a velocidade dos motores, mantendo o robô estável e no caminho correto.

## 🎮 O controle PID

O controlador PID (Proporcional, Integral, Derivativo) ajusta a resposta de um sistema utilizando três termos:

- **Proporcional (`P`)**: responde proporcionalmente ao erro atual.
- **Integral (`I`)**: corrige desvios acumulados ao longo do tempo.
- **Derivativo (`D`)**: suaviza a resposta antecipando variações bruscas.

Esse tipo de controle é amplamente usado em sistemas embarcados para controle de velocidade, posição, direção, etc.

#### Fórmula do PID:

```math
u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt
```

Onde:

- `e(t)`: erro = `setpoint - feedback`
- `Kp`, `Ki`, `Kd`: ganhos ajustáveis
- `u(t)`: saída do controlador (ajuste aplicado nos motores)

## 💻 Como usar

#### ✅ 1. Inclua o cabeçalho

```cpp
#include "pid_controller.h"
```

#### ✅ 2. Instancie o controlador

```cpp
PidController pid(1.0, 0.5, 0.1);  // Kp, Ki, Kd
```

#### ✅ 3. Use dentro do loop principal
Veja o arquivo de exemplo: [`examples/main.cpp`](examples/main.cpp)

## 🤝 Contribuição

Contribuições são bem-vindas!  
Sinta-se à vontade para abrir issues e pull requests para melhorias, correções ou novas funcionalidades.
