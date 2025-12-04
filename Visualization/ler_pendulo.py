#!/usr/bin/env python3
# read_estados.py
# Lê frames: 0xFF (8*4 bytes doubles little-endian) 0x00

import serial
import struct
import time
from dataclasses import dataclass

# Ajuste a porta serial e baudrate conforme seu sistema
SERIAL_PORT = "COM3"   # Windows ex: "COM3"
BAUDRATE = 9600
TIMEOUT = 0.5  # segundos

START_BYTE = 0xFF
END_BYTE   = 0x00
DOUBLES_COUNT = 5
DOUBLE_SIZE = 8
PAYLOAD_SIZE = DOUBLES_COUNT * DOUBLE_SIZE  # 40

@dataclass
class Informacoes:
    d1: float
    d2: float
    d3: float
    d4: float
    d5: float

def open_serial(port=SERIAL_PORT, baud=BAUDRATE, timeout=TIMEOUT):
    return serial.Serial(port, baud, timeout=timeout)

def read_frame(ser):
    """
    Lê e retorna um objeto Estados quando um frame válido for recebido.
    Bloqueia até obter um frame válido ou retorna None se timeout acontecer.
    """
    # State machine simples:
    # 1) procurar START_BYTE
    # 2) ler payload de PAYLOAD_SIZE bytes (pode precisar de reads repetidos)
    # 3) ler END_BYTE e validar
    # Se algo falhar, tenta re-sincronizar a partir do próximo START_BYTE.
    start_found = False

    # procura start
    while True:
        b = ser.read(1)
        if not b:
            return None  # timeout no read
        if b[0] == START_BYTE:
            start_found = True
            break
        # caso contrário continua procurando

    # ler payload completo (40 bytes)
    payload = bytearray()
    remaining = PAYLOAD_SIZE
    while remaining > 0:
        chunk = ser.read(remaining)
        if not chunk:
            # timeout enquanto recebia payload -> abortar este frame
            return None
        payload.extend(chunk)
        remaining = PAYLOAD_SIZE - len(payload)

    # ler o byte final
    end = ser.read(1)
    if not end:
        return None
    if end[0] != END_BYTE:
        # tentativa de re-sincronizar: talvez esse byte faça parte do próximo pacote
        # ignoramos e retornamos None para tentar novamente
        return None

    # agora temos payload de 32 bytes; desempacotar 4 doubles little-endian
    # struct format: '<4d' => little-endian, 4 doubles
    try:
        d1, d2, d3, d4, d5 = struct.unpack('<5d', payload)
    except struct.error:
        return None

    return Informacoes(d1, d2, d3, d4, d5)

def main():
    print(f"Abrindo porta {SERIAL_PORT} @ {BAUDRATE}...")
    try:
        ser = open_serial()
    except Exception as e:
        print("Erro abrindo serial:", e)
        return

    print("Aguardando frames. Pressione Ctrl+C para sair.")
    try:
        while True:
            frame = read_frame(ser)
            if frame is None:
                # sem frame válido (timeout ou erro) — continuar esperando
                continue
            # Exemplo de uso: imprimir com maior precisão
            print(f"Recebido: {frame.d1:.10g}, {frame.d2:.10g}, {frame.d3:.10g}, {frame.d4:.10g}, {frame.d5:.10g}")
            # Se quiser gravar em arquivo ou processar, faça aqui
    except KeyboardInterrupt:
        print("\nEncerrando.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
