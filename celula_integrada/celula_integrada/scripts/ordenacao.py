#!/usr/bin/env python3
"""Leitura/validação do plano_producao.yaml e ordenação por prioridade.

Uso standalone (teste sem ROS):  python3 ordenacao.py ../config/plano_producao.yaml
"""
import sys
import yaml


def carregar(caminho):
    with open(caminho, "r") as f:
        cfg = yaml.safe_load(f)
    validar(cfg)
    return cfg


def validar(cfg):
    for chave in ("locais", "catalogo_pecas", "plano"):
        if chave not in cfg:
            raise ValueError("YAML sem a chave obrigatória '%s'" % chave)
    if "C4" not in cfg["locais"]:
        raise ValueError("Local C4 (entrega) não definido")
    for nome, loc in cfg["locais"].items():
        if len(loc) != 3:
            raise ValueError("Local %s deve ser [x, y, th]" % nome)
    for peca, info in cfg["catalogo_pecas"].items():
        for k in ("max_vel", "destino_tm", "prioridade"):
            if k not in info:
                raise ValueError("Peça %s sem o campo '%s'" % (peca, k))
        if info["destino_tm"] not in ("esquerda", "direita"):
            raise ValueError("Peça %s: destino_tm inválido" % peca)
    for i, m in enumerate(cfg["plano"]):
        if m["peca"] not in cfg["catalogo_pecas"]:
            raise ValueError("Missão %d: peça '%s' não está no catálogo" % (i, m["peca"]))
        if m["origem"] not in cfg["locais"] or m["origem"] == "C4":
            raise ValueError("Missão %d: origem '%s' inválida" % (i, m["origem"]))


def ordenar(cfg):
    """Menor número de prioridade = mais urgente. sorted() é estável:
    empates preservam a ordem original do arquivo."""
    cat = cfg["catalogo_pecas"]
    return sorted(cfg["plano"], key=lambda m: cat[m["peca"]]["prioridade"])


if __name__ == "__main__":
    c = carregar(sys.argv[1])
    print("Ordem original:", [(m["peca"], m["origem"]) for m in c["plano"]])
    print("Ordem final   :", [(m["peca"], m["origem"]) for m in ordenar(c)])
