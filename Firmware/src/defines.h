typedef struct{
  double x1;
  double x2;
  double x3;
  double x4;
} estados_t;

// Definições de pinos (ajuste conforme sua conexão)
#define ENC_RODA_A 18
#define ENC_RODA_B 5
#define ENC_PEND_A 19
#define ENC_PEND_B 21
#define IN_A 25
#define IN_B 26
#define ONBOARD_LED 2
// #define BOTAO_SU 34
#define BOTAO_CONT 27

// Constantes do sistema
#define Ts 0.02f
#define en2rad_roda 0.006411413578755f
#define en2rad_pend 0.001570796326795f
#define pi 3.141592653589793f

// Ganhos LQR
#define LQR_K1 -5.541321904960439
#define LQR_K2 0.0 
#define LQR_K3 -0.726359517978251
#define LQR_K4 -0.098019480640735

// Comunicacao
#define DOUBLE_SIZE sizeof(double)
#define QTD_VARS_COM 5
#define TAMANHO_PAYLOAD (QTD_VARS_COM*sizeof(double))
#define TAMANHO_FRAME (TAMANHO_PAYLOAD + 2*sizeof(u_char))

// DEBUG
#define ACQSIGNAL false