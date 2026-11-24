#!/usr/bin/env python3
"""Análise do execucao_celula.csv: tempos de ciclo por missão.

Uso: python3 analisar_log.py execucao_celula.csv [--plot]
"""
import csv
import sys
from collections import defaultdict


def main(caminho, plot=False):
    ev = defaultdict(list)
    with open(caminho) as f:
        for r in csv.DictReader(f):
            if r["missao"] not in ("-", ""):
                ev[int(r["missao"])].append((float(r["timestamp_ros"]), r))

    def t(lst, estado, evento, ultimo=False):
        c = [ts for ts, r in lst if r["estado"] == estado and r["evento"] == evento]
        return (c[-1] if ultimo else c[0]) if c else None

    linhas = []
    for m, lst in sorted(ev.items()):
        peca = lst[0][1]["peca"]
        ini = lst[0][0]
        nav_col = _d(t(lst, "IR_COLETA", "inicio_navegacao"), t(lst, "IR_COLETA", "chegada"))
        nav_ent = _d(t(lst, "IR_ENTREGA", "inicio_navegacao"), t(lst, "IR_ENTREGA", "chegada"))
        valid = _d(t(lst, "VALIDA_POSE", "inicio"), t(lst, "VALIDA_POSE", "ok"))
        pega = _d(t(lst, "HANDSHAKE", "inicio"), t(lst, "HANDSHAKE", "pega_iniciada"))
        tm_tot = _d(t(lst, "HANDSHAKE", "inicio"), t(lst, "HANDSHAKE", "conclusao"))
        lib = t(lst, "LIBERADO", "pioneer_livre")
        ciclo_pioneer = _d(ini, lib)
        linhas.append((m, peca, nav_col, nav_ent, valid, pega, tm_tot, ciclo_pioneer))

    cab = ("missao", "peca", "nav_coleta", "nav_entrega", "validacao", "t_ate_pega",
           "tm_total", "ciclo_pioneer")
    print(",".join(cab))
    for l in linhas:
        print(",".join("" if v is None else (("%.2f" % v) if isinstance(v, float) else str(v)) for v in l))

    if plot:
        import matplotlib.pyplot as plt
        x = [f"{l[0]}:{l[1]}" for l in linhas]
        plt.bar(x, [l[7] or 0 for l in linhas], label="Ciclo Pioneer")
        plt.bar(x, [l[6] or 0 for l in linhas], width=0.4, label="Duração TM")
        plt.ylabel("s"); plt.legend(); plt.title("Tempos de ciclo por missão")
        plt.savefig("tempos_ciclo.png", dpi=150)
        print("Gráfico salvo em tempos_ciclo.png")


def _d(a, b):
    return None if a is None or b is None else b - a


if __name__ == "__main__":
    main(sys.argv[1], "--plot" in sys.argv)
