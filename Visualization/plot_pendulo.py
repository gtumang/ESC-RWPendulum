#!/usr/bin/env python3
"""
plot_multi_dynamic.py

Leitura de frames seriais: 0xFF (5 doubles little-endian) 0x00
Cria múltiplas figuras matplotlib dinamicamente a partir do mesmo stream.
Cada figura pode plotar qualquer combinação das variáveis d1..d5.
"""

import argparse
import struct
import threading
import time
from collections import deque
from typing import List, Tuple

import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- opcional: pyserial para leitura real ---
try:
    import serial
except Exception:
    serial = None

# --- Configurações padrão ---
SERIAL_PORT = "COM3"    # ex: "COM3" no Windows
BAUDRATE = 9600
TIMEOUT = 0.5

START_BYTE = 0xFF
END_BYTE = 0x00
DOUBLES_COUNT = 5
DOUBLE_SIZE = 8
PAYLOAD_SIZE = DOUBLES_COUNT * DOUBLE_SIZE  # 40 bytes

BUFFER_SECONDS = 30
UPDATE_INTERVAL_MS = 100
MAX_POINTS = int(BUFFER_SECONDS * 1000 / UPDATE_INTERVAL_MS)

# --- leitor serial em background (preenche deque) ---
def read_frame_from_serial(ser) -> Tuple[float, ...] or None:
    """Lê um frame e retorna tuple (d1..d5) ou None se timeout/erro."""
    # procurar start
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == START_BYTE:
            break

    payload = bytearray()
    remaining = PAYLOAD_SIZE
    while remaining > 0:
        chunk = ser.read(remaining)
        if not chunk:
            return None
        payload.extend(chunk)
        remaining = PAYLOAD_SIZE - len(payload)

    end = ser.read(1)
    if not end or end[0] != END_BYTE:
        return None

    try:
        return struct.unpack("<5d", payload)
    except struct.error:
        return None


def serial_reader_thread(data_deque: deque, lock: threading.Lock, stop_event: threading.Event,
                         port: str, baud: int, timeout: float, simulate: bool):
    """
    Popula data_deque com tuplas (timestamp, d1, d2, d3, d4, d5).
    Usar lock ao acessar data_deque para thread-safety.
    """
    if simulate:
        print("SIMULANDO dados (modo --sim).")
        import math
        t0 = time.time()
        while not stop_event.is_set():
            t = time.time()
            d1 = 2.0 * math.sin(2 * math.pi * 0.2 * (t - t0)) + 0.1 * math.sin(t * 3.3)
            d2 = 0.5 * math.sin(2 * math.pi * 0.5 * (t - t0))
            d3 = 0.2 * math.cos(2 * math.pi * 0.8 * (t - t0))
            d4 = 0.0
            d5 = 1.5 * math.cos(2 * math.pi * 0.1 * (t - t0)) + 0.05 * math.sin(t * 1.7)
            with lock:
                data_deque.append((t, d1, d2, d3, d4, d5))
                while len(data_deque) > MAX_POINTS:
                    data_deque.popleft()
            time.sleep(UPDATE_INTERVAL_MS / 1000.0)
        return

    # leitura real
    if serial is None:
        print("pyserial não disponível. Instale 'pyserial' ou use --sim.")
        return

    try:
        ser = serial.Serial(port, baud, timeout=timeout)
    except Exception as e:
        print("Erro abrindo serial:", e)
        return

    print(f"Conectado em {port} @ {baud}")
    try:
        while not stop_event.is_set():
            frame = read_frame_from_serial(ser)
            if frame is None:
                continue
            t = time.time()
            with lock:
                data_deque.append((t, *frame))
                while len(data_deque) > MAX_POINTS:
                    data_deque.popleft()
    finally:
        ser.close()


# --- função reutilizável que cria uma figura/animation ---
def create_figure(
    data_deque: deque,
    lock: threading.Lock,
    var_indices: List[int],
    *,
    separate_y: bool = False,
    buffer_seconds: float = BUFFER_SECONDS,
    update_interval_ms: int = UPDATE_INTERVAL_MS,
    fig_title: str = None,
) -> Tuple[plt.Figure, animation.FuncAnimation]:
    """
    Cria uma figura que plota as variáveis em var_indices (1-based: 1..5).
    - var_indices: lista de índices (ex: [1,5] para d1 e d5).
    - separate_y: se True cria subplots empilhados, um por variável (eixos Y independentes),
                  se False plota todas no mesmo eixo (mesmo Y).
    Retorna (fig, anim) — mantenha references no caller para evitar GC.
    """
    # valida índices
    for idx in var_indices:
        if idx < 1 or idx > DOUBLES_COUNT:
            raise ValueError("var_indices deve conter valores entre 1 e 5 (inclusive)")

    n_vars = len(var_indices)
    fig = None
    axes = []
    lines = []
    last_texts = []

    if separate_y:
        fig, axes = plt.subplots(n_vars, 1, sharex=True, figsize=(8, 3 * n_vars))
        # quando n_vars == 1, axes pode não ser uma lista
        if n_vars == 1:
            axes = [axes]
        for i, ax in enumerate(axes):
            ax.grid(True)
            ax.set_ylabel(f"d{var_indices[i]}")
            ln, = ax.plot([], [], lw=1, label=f"d{var_indices[i]}")
            ax.legend(loc="upper right")
            lines.append(ln)
            last_texts.append(ax.text(0.02, 0.9, "", transform=ax.transAxes, va="top"))
        axes[-1].set_xlabel("Tempo (s, relativo)")
        if fig_title:
            fig.suptitle(fig_title)
    else:
        fig, ax = plt.subplots(figsize=(10, 4))
        if fig_title:
            fig.suptitle(fig_title)
        ax.set_xlabel("Tempo (s, relativo)")
        ax.set_ylabel("Valor")
        ax.grid(True)
        for idx in var_indices:
            ln, = ax.plot([], [], lw=1, label=f"d{idx}")
            lines.append(ln)
        ax.legend(loc="upper right")
        axes = [ax]
        last_texts = [ax.text(0.02, 0.95, "", transform=ax.transAxes, va="top")]

    # init function
    def init():
        for ax in axes:
            ax.set_xlim(0, buffer_seconds)
            ax.set_ylim(-1.0, 1.0)
        for ln in lines:
            ln.set_data([], [])
        for t in last_texts:
            t.set_text("")
        return (*lines, *last_texts)

    # update function
    def update(_):
        with lock:
            if not data_deque:
                return (*lines, *last_texts)
            # fazer cópia leve dos dados para processamento fora do lock (lista de tuplas)
            data_copy = list(data_deque)

        times = [t for (t, *_) in data_copy]
        t0 = times[0]
        x_rel = [tt - t0 for tt in times]
        x_max = x_rel[-1]
        x_min = max(0.0, x_max - buffer_seconds)

        # extrair vetores para cada variável solicitada
        ys = []
        for var_idx in var_indices:
            # var_idx é 1-based e os dados são (t, d1, d2, d3, d4, d5)
            pos = var_idx  # because tuple index 1 corresponds to d1
            ys.append([row[pos] for row in data_copy])  # row[pos] é d{var_idx}

        if separate_y:
            for ax, y, ln, lt, idx in zip(axes, ys, lines, last_texts, var_indices):
                # set x-limits
                ax.set_xlim(x_min, x_max)
                # y-limits com margem
                y_min, y_max = min(y), max(y)
                if y_min == y_max:
                    y_min -= 0.5
                    y_max += 0.5
                margin = 0.1 * (y_max - y_min)
                ax.set_ylim(y_min - margin, y_max + margin)
                ln.set_data(x_rel, y)
                lt.set_text(f"Último d{idx} = {y[-1]:.6g}")
        else:
            ax = axes[0]
            ax.set_xlim(x_min, x_max)
            all_y = []
            for y in ys:
                all_y.extend(y)
            y_min, y_max = min(all_y), max(all_y)
            if y_min == y_max:
                y_min -= 0.5
                y_max += 0.5
            margin = 0.1 * (y_max - y_min)
            ax.set_ylim(y_min - margin, y_max + margin)
            for ln, y, idx, lt in zip(lines, ys, var_indices, last_texts):
                ln.set_data(x_rel, y)
                lt.set_text(f"Último d{idx} = {y[-1]:.6g}")

        return (*lines, *last_texts)

    anim = animation.FuncAnimation(fig, update, init_func=init,
                                   interval=update_interval_ms, blit=False)
    fig.tight_layout()
    return fig, anim


# --- EXEMPLO DE USO: criar múltiplas figuras dinamicamente ---
def main(args):
    data_deque = deque()
    lock = threading.Lock()
    stop_event = threading.Event()

    reader = threading.Thread(
        target=serial_reader_thread,
        args=(data_deque, lock, stop_event, args.port, args.baud, args.timeout, args.sim),
        daemon=True
    )
    reader.start()

    # === CONFIGURE AQUI as figuras que deseja criar ===
    # cada item é (var_indices, separate_y, titulo)
    # var_indices são 1-based (1..5) correspondendo a d1..d5
    fig_configs = [
        # exemplo: figura A com d1 (superior) e d5 (inferior) em subplots (eixos Y separados)
        ([1, 5], True, "Figura A: theta1 (rad) (top) & u (volts) (bottom) - Y separados"),
        # exemplo: figura B com d2 e d3 no mesmo eixo (Y compartilhado)
        # ([2, 3], False, "Figura B: d2 & d3 - mesmo eixo Y"),
        # você pode adicionar quantas figuras quiser, p.ex. ([1,2,3], False, "Outra")
    ]
    # === fim da configuração ===

    # criar figuras dinamicamente e manter referências em listas para não serem GC'd
    figs = []
    anims = []
    for (var_indices, separate_y, title) in fig_configs:
        fig, anim = create_figure(
            data_deque,
            lock,
            var_indices,
            separate_y=separate_y,
            buffer_seconds=BUFFER_SECONDS,
            update_interval_ms=UPDATE_INTERVAL_MS,
            fig_title=title
        )
        figs.append(fig)
        anims.append(anim)

    try:
        print("Mostrando figuras. Feche todas as janelas para encerrar.")
        plt.show()
    finally:
        stop_event.set()
        reader.join(timeout=1.0)
        print("Finalizando.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot múltiplas figuras dinâmicas a partir do mesmo stream serial.")
    parser.add_argument("--port", type=str, default=SERIAL_PORT, help="porta serial (ex: /dev/ttyUSB0 ou COM3)")
    parser.add_argument("--baud", type=int, default=BAUDRATE, help="baudrate")
    parser.add_argument("--timeout", type=float, default=TIMEOUT, help="timeout de leitura (s)")
    parser.add_argument("--sim", action="store_true", help="usar modo simulado (sem hardware)")
    args = parser.parse_args()
    main(args)
