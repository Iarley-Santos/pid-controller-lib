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

## 📚 Funções

#### 1. `pid_controller(float kp, float ki, float kd)`

- **Descrição**: Constrói uma instância do controlador PID com os ganhos `Kp`, `Ki` e `Kd` definidos.
- **Parâmetros**:
  - `kp`: Ganho proporcional.
  - `ki`: Ganho integral.
  - `kd`: Ganho derivativo.
- **Exemplo**:
  ```cpp
  pid_controller pid(1.0, 0.5, 0.1);  // Inicializa PID com ganhos específicos

#### 2. `void set_kp(float kp)`

- **Descrição**:  
  Esta função permite ajustar o ganho proporcional (`Kp`) do controlador PID. O ganho proporcional afeta diretamente a resposta do controlador ao erro atual. Quanto maior o valor de `Kp`, maior será a reação do controlador ao erro.
- **Parâmetros**:
  - `kp`: Novo valor para o ganho proporcional.
- **Exemplo**:
  ```cpp
  pid.set_kp(2.0);  // Atualiza o ganho proporcional para 2.0

#### 3. `void set_ki(float ki)`

- **Descrição**:  
  Esta função ajusta o ganho integral (`Ki`) do controlador PID. O ganho integral é responsável por acumular o erro ao longo do tempo e corrigir desvios persistentes, ajudando a eliminar o erro residual que pode ocorrer em sistemas de controle de longo prazo.
- **Parâmetros**:
  - `ki`: Novo valor para o ganho integral.
- **Exemplo**:
  ```cpp
  pid.set_ki(1.0);  // Atualiza o ganho integral para 1.0

#### 4. `void set_kd(float kd)`

- **Descrição**:  
  Esta função ajusta o ganho derivativo (`Kd`) do controlador PID. O ganho derivativo atua sobre a taxa de variação do erro, ajudando a suavizar a resposta do sistema e a antecipar mudanças bruscas, reduzindo oscilações no controle.
- **Parâmetros**:
  - `kd`: Novo valor para o ganho derivativo.
- **Exemplo**:
  ```cpp
  pid.set_kd(0.5);  // Atualiza o ganho derivativo para 0.5

#### 5. `void set_parameters(float kp, float ki, float kd)`

- **Descrição**:  
  Esta função permite ajustar os três ganhos (`Kp`, `Ki`, `Kd`) de uma vez. Ela é útil quando você deseja atualizar rapidamente todos os parâmetros do controlador PID sem ter que chamar as funções de ajuste individualmente.
- **Parâmetros**:
  - `kp`: Novo valor para o ganho proporcional.
  - `ki`: Novo valor para o ganho integral.
  - `kd`: Novo valor para o ganho derivativo.
- **Exemplo**:
  ```cpp
  pid.set_parameters(1.2, 0.7, 0.15);  // Atualiza todos os ganhos de uma vez

#### 6. `int pid_calculation(float setpoint, float feedback, float delta_time)`

- **Descrição**:  
  Esta função realiza o cálculo do controlador PID. Ela recebe o `setpoint` (valor desejado), o `feedback` (valor atual medido) e o `delta_time` (intervalo de tempo entre os cálculos) para determinar o valor de controle necessário para minimizar o erro entre o valor atual e o desejado.
- **Parâmetros**:
  - `setpoint`: O valor desejado (por exemplo, a posição ou velocidade alvo).
  - `feedback`: O valor atual medido (por exemplo, a posição ou velocidade atual).
  - `delta_time`: O intervalo de tempo entre cada iteração do loop de controle, usado para calcular a variação do erro.
- **Retorno**:
  - Retorna o valor de controle calculado, que pode ser aplicado ao sistema (por exemplo, motores).
- **Exemplo**:
  ```cpp
  int u = pid.pid_calculation(100.0, 95.0, 0.1);  // Calcula a saída de controle baseado no erro

#### 7. `int16_t right_motor_speed(int16_t min_speed, uint16_t max_speed, int16_t motor_bias, int u)`

- **Descrição**:  
  Esta função calcula a velocidade do motor direito com base no valor de controle (`u`), nos limites de velocidade (`min_speed` e `max_speed`), e no viés do motor (`motor_bias`). O valor de controle ajusta a velocidade do motor para corrigir a diferença entre o `setpoint` e o `feedback`.
- **Parâmetros**:
  - `min_speed`: Velocidade mínima permitida para o motor.
  - `max_speed`: Velocidade máxima permitida para o motor.
  - `motor_bias`: Viés do motor, usado para ajustar a velocidade base do motor.
  - `u`: Valor de controle calculado pelo PID que ajusta a velocidade.
- **Retorno**:
  - Retorna a velocidade ajustada do motor direito, respeitando os limites de velocidade definidos.
- **Exemplo**:
  ```cpp
  int16_t speed_right = pid.right_motor_speed(0, 255, 50, u);  // Calcula a velocidade do motor direito com viés e controle

#### 8. `int16_t left_motor_speed(int16_t min_speed, uint16_t max_speed, int16_t motor_bias, int u)`

- **Descrição**:  
  Semelhante à função para o motor direito, esta função calcula a velocidade do motor esquerdo, levando em consideração o valor de controle (`u`), os limites de velocidade (`min_speed` e `max_speed`), e o viés do motor (`motor_bias`). O valor de controle ajusta a velocidade do motor esquerdo para corrigir a diferença entre o `setpoint` e o `feedback`.
- **Parâmetros**:
  - `min_speed`: Velocidade mínima permitida para o motor.
  - `max_speed`: Velocidade máxima permitida para o motor.
  - `motor_bias`: Viés do motor, usado para ajustar a velocidade base do motor.
  - `u`: Valor de controle calculado pelo PID que ajusta a velocidade.
- **Retorno**:
  - Retorna a velocidade ajustada do motor esquerdo, respeitando os limites de velocidade definidos.
- **Exemplo**:
  ```cpp
  int16_t speed_left = pid.left_motor_speed(0, 255, 50, u);  // Calcula a velocidade do motor esquerdo com viés e controle

  
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
