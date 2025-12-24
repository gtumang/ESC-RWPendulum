# ESC-RWPendulum

Este repositório contém o firmware, modelos de simulação e ferramentas de visualização para um sistema de controle de **Pêndulo com Roda de Reação** (Pêndulo Invertido). O projeto foi projetado para estabilizar o pêndulo usando uma roda de reação acionada por um motor DC, controlado por um microcontrolador ESP32.

## Estrutura do Projeto

O repositório está organizado em quatro diretórios principais:

* **`Firmware/`**: Código C++ embarcado para o ESP32, construído usando PlatformIO. Gerencia a leitura de sensores, a execução da malha de controle e a comunicação serial.
* **`Matlab/`**: Scripts MATLAB e modelos Simulink para definição de parâmetros físicos, modelagem matemática e cálculo de ganho de controle (LQR).
* **`Visualization/`**: Scripts Python para telemetria em tempo real, registro de dados e plotagem via Serial.
* **`PCB/`**: Arquivos de projeto da placa de circuito impresso (Hardware), incluindo esquemáticos, layout e arquivos de fabricação.

---

## 1. Firmware (ESP32)

O firmware está localizado no diretório `Firmware` e está configurado como um projeto **PlatformIO**.

### Configuração de Hardware
* **Microcontrolador**: ESP32 (Placa: `fm-devkit`)
* **Atuador**: Motor DC com driver PWM (controlado pelos pinos 25/26).
* **Sensores**: Dois Encoders de Quadratura (Ângulo do Pêndulo e Velocidade da Roda).

**Pinagem (`main.cpp`):**
| Componente | Pino A | Pino B | Notas |
| :--- | :--- | :--- | :--- |
| **Encoder da Roda** | GPIO 17 | GPIO 16 | Mede velocidade/posição da roda de reação |
| **Encoder do Pêndulo** | GPIO 19 | GPIO 18 | Mede o ângulo de inclinação do pêndulo |
| **Driver do Motor** | GPIO 25 | GPIO 26 | Saída PWM (LEDC) |
| **Botão Iniciar/Parar**| GPIO 4 | - | Alterna a malha de controle (ISR) |

### Malha de Controle
* **Algoritmo**: Controle em Espaço de Estados (LQR).
* **Frequência**: 50 Hz (`Ts = 0.02s`).
* **Estados**:
    1.  $x_1$: Ângulo do Pêndulo (rad)
    2.  $x_2$: Ângulo da Roda (rad)
    3.  $x_3$: Velocidade do Pêndulo (rad/s)
    4.  $x_4$: Velocidade da Roda (rad/s)
* **Comunicação**: Envia dados de estado e esforço de controle via Serial (Formato binário).

---
## 2. MATLAB & Simulação

O diretório `Matlab` contém a fundação matemática do projeto.

* **`Scripts/SalavaParams.m`**: Define as constantes físicas do sistema (massas, comprimentos, inércia, constantes do motor) e as salva em `Parametros/parametros_planta.mat`.
    * *Parâmetros incluem:* $M_h$ (Massa do cubo), $M_r$ (Massa do rotor), $L$ (Comprimento), $J_h$ (Inércia), etc.
* **`PenduloRotativoSim.slx`**: Modelo Simulink para simular a dinâmica do sistema e a resposta do controlador.
* **`Funcoes/`**: Contém funções geradas automaticamente (`function_G_Rot`, `function_M_Rot`, etc.) para a simulação não linear do sistema.
* **`Parametros/`**: Armazena arquivos `.mat` com a matriz de ganho de controle `K` calculada.

**Como usar:**
1.  Edite os parâmetros físicos em `SalavaParams.m` e execute-o.
2.  Execute o script de análise principal (ex: `main.mlx`) para calcular a matriz de ganho LQR $K$.
3.  Atualize a struct `K` em `Firmware/src/main.cpp` com os valores calculados.

---

## 3. Visualização (Python)

O diretório `Visualization` fornece ferramentas para visualizar dados do sistema em tempo real.

### Requisitos
* Python 3.x
* Bibliotecas: `pyserial`, `matplotlib`

### Scripts
* **`ler_pendulo.py`**: Um utilitário simples para ler dados brutos da porta serial e imprimir os valores decodificados em ponto flutuante no console.
* **`plot_pendulo.py`**: Uma ferramenta de plotagem em tempo real.
    * **Uso**: `python plot_pendulo.py --port COM3`
    * **Recursos**:
        * Plota múltiplas variáveis dinamicamente.
        * Suporta um modo de simulação para testar a interface sem hardware: `python plot_pendulo.py --sim`.
        * Visualiza Ângulo, Velocidade e Esforço de Controle ($u$).

---

## 4. Hardware e PCB (KiCad)

O diretório **`PCB`** contém todo o projeto eletrônico necessário para fabricar a placa de controle customizada do pêndulo, desenvolvido no software **KiCad**.

### Estrutura dos Arquivos
* **`RWPendulum/`**: Contém os arquivos fonte do projeto KiCad, incluindo o esquemático (`.kicad_sch`) e o layout da PCB (`.kicad_pcb`). O design integra o módulo ESP32 DevKit V1 e o driver de motor.
* **`OutputFiles/`**: Arquivos gerados para fabricação da placa:
    * **`Gerber/`**: Arquivos Gerber para as camadas de cobre, máscara de solda e corte.
    * **`Drill/`**: Arquivos de furação (PTH e NPTH).
* **`3D/`**: Modelos 3D (.step) da placa completa e componentes específicos (ex: Driver MD36A) para visualização mecânica e validação de encaixe.

---

## Licença
*Nenhuma licença especificada nos arquivos do repositório*.

## Autor
**Gabriel Brunoro Motta Tumang**