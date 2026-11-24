# celula_integrada — Pioneer 3DX + Omron TM (ROS 1 Noetic)

Célula de manufatura flexível coordenada pelo nó **Maestro**.

## Arquitetura de comunicação

| Nó | Máquina | Interface |
|---|---|---|
| `RosAria` | PC do Pioneer | sub `/cmd_vel`, pub `/odom` |
| laser | PC do Pioneer | pub `/scan` |
| `amcl`, `map_server`, `move_base` | PC do Pioneer (ou notebook A) | pub `/amcl_pose`; action `move_base` |
| `maestro` | Notebook A (master) | client `move_base` e `transfer_piece`; sub `/amcl_pose`, `/odom`; pub `/cmd_vel` (correção fina); dynamic_reconfigure no DWA |
| `tm_driver` + `move_group` | Notebook B | driver do TM e MoveIt! |
| `tm_node` | Notebook B | **Action Server** `transfer_piece` (`TransferPiece.action`) |

Fluxo: `plano_producao.yaml → ordenacao.py → maestro` → (IR_COLETA → CARREGANDO → IR_ENTREGA → VALIDA_POSE → HANDSHAKE → LIBERADO).
O feedback `PICK_STARTED` do TM é o gatilho que libera o Pioneer enquanto o braço ainda deposita (concorrência).
Exclusão mútua: o Maestro não envia nova missão ao TM nem (por padrão) entra em C4 enquanto o TM está ocupado.

## Instalação
```bash
cd ~/celula_ws/src
git clone https://github.com/TechmanRobotInc/tmr_ros1.git
git clone https://github.com/amor-ros-pkg/rosaria.git
# copie esta pasta (celula_integrada) para cá
cd .. && rosdep install --from-paths src --ignore-src -y && catkin_make
```

## Ordem de execução
1. Notebook A: `source env/env_notebook_A.sh && roscore`
2. Pioneer: `source env/env_pioneer.sh && roslaunch celula_integrada pioneer.launch`
3. (uma vez) mapeamento: `roslaunch celula_integrada mapeamento.launch` e salvar o mapa em `maps/arena`
4. Pioneer/A: `roslaunch celula_integrada navegacao.launch x0:=.. y0:=.. a0:=..`
5. Notebook B: TMflow com nó **Listen** rodando, depois `roslaunch celula_integrada tm.launch robot_ip:=<IP>`
   (teste antes com `sim:=true simular_gripper:=true`)
6. Notebook A: `roslaunch celula_integrada maestro.launch`

## Calibração obrigatória
- `config/plano_producao.yaml`: coordenadas reais de C1–C4 (`rostopic echo -n1 /amcl_pose`).
- `config/poses_tm.yaml`: poses articulares **placeholder** — reensinar todas.
- `launch/pioneer.launch`: posição do laser; `config/nav/costmap_common_params.yaml`: footprint.
- `tm.launch`: nome do pacote MoveIt! do seu modelo de TM e nome do grupo (`tmr_arm`).
- Confirme campos do serviço `tm_msgs/SetIO` na sua versão do `tmr_ros1`.

## Testes
```bash
python3 scripts/ordenacao.py config/plano_producao.yaml   # testa a ordenação sem ROS
rosrun actionlib axclient.py /transfer_piece              # testa o TM isolado
python3 scripts/analisar_log.py ~/execucao_celula.csv --plot
```

## Log (`execucao_celula.csv`)
Colunas: `timestamp_ros, timestamp_iso, missao, peca, origem, estado, evento, detalhe`.
Sincronize os relógios dos PCs (chrony) para os timestamps serem coerentes.

## Segurança
Valide as trajetórias do TM em simulação, mantenha `vel_scaling` baixo e o botão de emergência acessível.
