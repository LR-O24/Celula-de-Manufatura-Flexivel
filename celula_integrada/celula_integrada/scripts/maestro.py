#!/usr/bin/env python3
"""Nó Maestro: coordena Pioneer 3DX (move_base) e braço TM (action transfer_piece)."""
import csv
import math
import os
import statistics
import threading
from datetime import datetime

import actionlib
import rospy
import tf.transformations as tft
from actionlib_msgs.msg import GoalStatus
from dynamic_reconfigure.client import Client
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

from celula_integrada.msg import TransferPieceAction, TransferPieceGoal

import ordenacao


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clip(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_de(q):
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class Logger:
    """CSV estruturado; flush a cada linha para sobreviver a Ctrl+C."""
    CAMPOS = ["timestamp_ros", "timestamp_iso", "missao", "peca", "origem",
              "estado", "evento", "detalhe"]

    def __init__(self, caminho):
        os.makedirs(os.path.dirname(os.path.abspath(caminho)), exist_ok=True)
        self.f = open(caminho, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(self.CAMPOS)
        self.f.flush()
        self.lock = threading.Lock()

    def __call__(self, missao, peca, origem, estado, evento, detalhe=""):
        with self.lock:
            self.w.writerow(["%.3f" % rospy.Time.now().to_sec(),
                             datetime.now().isoformat(timespec="milliseconds"),
                             missao, peca, origem, estado, evento, detalhe])
            self.f.flush()
        rospy.loginfo("[%s] %s/%s %s %s", missao, peca, estado, evento, detalhe)

    def close(self):
        with self.lock:
            if not self.f.closed:
                self.f.close()


class Maestro:
    def __init__(self):
        # ---------- parâmetros ----------
        g = rospy.get_param
        self.plano_path = g("~plano")
        self.vel_padrao = g("~vel_padrao", 0.5)
        self.tempo_carga = g("~tempo_carga", 3.0)
        self.nav_timeout = g("~nav_timeout", 120.0)
        self.nav_tentativas = g("~nav_tentativas", 3)
        self.tol_pos = g("~tol_pos", 0.05)                       # m
        self.tol_ang = math.radians(g("~tol_ang_deg", 3.0))      # rad
        self.max_iter_corr = g("~max_iter_correcao", 8)
        self.n_amostras = g("~n_amostras_amcl", 5)
        self.esperar_tm_antes_c4 = g("~esperar_tm_antes_c4", True)
        self.pick_timeout = g("~pick_timeout", 90.0)
        self.cmd_vel_topic = g("~cmd_vel_topic", "cmd_vel")
        self.odom_topic = g("~odom_topic", "odom")
        self.dwa_ns = g("~dwa_ns", "/move_base/DWAPlannerROS")

        # ---------- plano ----------
        self.cfg = ordenacao.carregar(self.plano_path)
        self.locais = self.cfg["locais"]
        self.catalogo = self.cfg["catalogo_pecas"]
        self.fila = ordenacao.ordenar(self.cfg)
        self.log = Logger(g("~log", os.path.expanduser("~/execucao_celula.csv")))
        rospy.on_shutdown(self.log.close)
        rospy.loginfo("Fila ordenada por prioridade: %s",
                      [(m["peca"], m["origem"]) for m in self.fila])
        self.log("-", "-", "-", "INIT", "fila_ordenada",
                 str([(m["peca"], m["origem"]) for m in self.fila]))

        # ---------- ROS ----------
        self.odom = None
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        self.mb = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Aguardando move_base...")
        self.mb.wait_for_server()
        self.tm = actionlib.SimpleActionClient("transfer_piece", TransferPieceAction)
        rospy.loginfo("Aguardando transfer_piece (tm_node)...")
        self.tm.wait_for_server()

        try:
            self.dwa = Client(self.dwa_ns, timeout=15)
        except Exception as e:
            rospy.logwarn("dynamic_reconfigure do DWA indisponível: %s", e)
            self.dwa = None

        try:
            self.amcl_update = rospy.ServiceProxy("request_nomotion_update", Empty)
        except Exception:
            self.amcl_update = None

        # estado da concorrência com o TM
        self.tm_idle = threading.Event()
        self.tm_idle.set()
        self.pick_started = threading.Event()
        self.tm_falhou = False

    # ------------------------------------------------------------------ util
    def _odom_cb(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_de(p.orientation))

    def set_vel(self, v):
        if self.dwa is None:
            return
        try:
            self.dwa.update_configuration({"max_vel_x": v, "max_vel_trans": v})
        except Exception:
            try:
                self.dwa.update_configuration({"max_vel_x": v})
            except Exception as e:
                rospy.logwarn("Falha ao reconfigurar velocidade: %s", e)
        rospy.loginfo("max_vel reconfigurada para %.2f m/s", v)

    def parar(self):
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------ navegação
    def navegar(self, ctx, nome, estado):
        x, y, th = self.locais[nome]
        for tent in range(1, self.nav_tentativas + 1):
            if rospy.is_shutdown():
                return False
            g = MoveBaseGoal()
            g.target_pose.header.frame_id = "map"
            g.target_pose.header.stamp = rospy.Time.now()
            g.target_pose.pose.position.x = x
            g.target_pose.pose.position.y = y
            q = tft.quaternion_from_euler(0, 0, th)
            (g.target_pose.pose.orientation.x, g.target_pose.pose.orientation.y,
             g.target_pose.pose.orientation.z, g.target_pose.pose.orientation.w) = q
            self.log(*ctx, estado, "inicio_navegacao", "destino=%s tentativa=%d" % (nome, tent))
            self.mb.send_goal(g)
            if not self.mb.wait_for_result(rospy.Duration(self.nav_timeout)):
                self.mb.cancel_goal()
                self.log(*ctx, estado, "timeout_navegacao", nome)
                continue
            if self.mb.get_state() == GoalStatus.SUCCEEDED:
                self.log(*ctx, estado, "chegada", nome)
                return True
            self.log(*ctx, estado, "navegacao_falhou",
                     "%s status=%s" % (nome, GoalStatus.to_string(self.mb.get_state())
                                       if hasattr(GoalStatus, "to_string")
                                       else self.mb.get_state()))
            rospy.sleep(1.0)
        return False

    # ----------------------------------------------- validação de pose (C4)
    def medir_pose(self):
        """Mediana de N amostras do /amcl_pose (após forçar atualização do AMCL)."""
        if self.amcl_update:
            try:
                self.amcl_update()
            except Exception:
                pass
        rospy.sleep(0.3)
        xs, ys, yaws, cov = [], [], [], 0.0
        for _ in range(self.n_amostras):
            m = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=3.0)
            p = m.pose.pose
            xs.append(p.position.x)
            ys.append(p.position.y)
            yaws.append(yaw_de(p.orientation))
            cov = max(m.pose.covariance[0], m.pose.covariance[7])
        # média circular do yaw
        yaw = math.atan2(statistics.mean(math.sin(a) for a in yaws),
                         statistics.mean(math.cos(a) for a in yaws))
        return statistics.median(xs), statistics.median(ys), yaw, cov

    def erro(self, alvo, pose):
        dx, dy = alvo[0] - pose[0], alvo[1] - pose[1]
        return math.hypot(dx, dy), wrap(alvo[2] - pose[2]), dx, dy

    def girar(self, delta):
        """Gira 'delta' rad em malha fechada usando a odometria."""
        if self.odom is None or abs(delta) < math.radians(0.3):
            return
        alvo = wrap(self.odom[2] + delta)
        r, t0 = rospy.Rate(20), rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 15.0:
            err = wrap(alvo - self.odom[2])
            if abs(err) < math.radians(0.4):
                break
            w = clip(1.0 * err, -0.35, 0.35)
            if abs(w) < 0.06:
                w = math.copysign(0.06, err)
            t = Twist(); t.angular.z = w
            self.cmd_pub.publish(t); r.sleep()
        self.parar()

    def avancar(self, dist):
        """Avança/recua 'dist' m (com sinal) em malha fechada pela odometria."""
        if self.odom is None or abs(dist) < 0.005:
            return
        x0, y0 = self.odom[0], self.odom[1]
        sinal = 1.0 if dist > 0 else -1.0
        r, t0 = rospy.Rate(20), rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 15.0:
            falta = abs(dist) - math.hypot(self.odom[0] - x0, self.odom[1] - y0)
            if falta < 0.004:
                break
            t = Twist(); t.linear.x = sinal * clip(0.8 * falta, 0.03, 0.08)
            self.cmd_pub.publish(t); r.sleep()
        self.parar()

    def corrigir(self, alvo, pose):
        """Pioneer é diferencial: gira -> anda -> gira para o yaw final."""
        e, dth, dx, dy = self.erro(alvo, pose)
        yaw = pose[2]
        if e > self.tol_pos * 0.6:
            rumo = math.atan2(dy, dx)
            frente = abs(wrap(rumo - yaw)) <= math.pi / 2
            if not frente:                       # alvo atrás: recua em vez de girar 180°
                rumo = wrap(rumo + math.pi)
            self.girar(wrap(rumo - yaw))
            self.avancar(e if frente else -e)
            yaw = rumo
        self.girar(wrap(alvo[2] - yaw))

    def validar_e_corrigir(self, ctx, nome):
        alvo = self.locais[nome]
        for it in range(self.max_iter_corr + 1):
            try:
                pose = self.medir_pose()
            except rospy.ROSException:
                self.log(*ctx, "VALIDA_POSE", "sem_amcl_pose", "timeout")
                return False
            e, dth, _, _ = self.erro(alvo, pose)
            det = "iter=%d erro_pos=%.4fm erro_ang=%.2fdeg cov=%.5f" % (
                it, e, math.degrees(dth), pose[3])
            if e < self.tol_pos and abs(dth) < self.tol_ang:
                self.log(*ctx, "VALIDA_POSE", "ok", det)
                return True
            self.log(*ctx, "VALIDA_POSE", "fora_tolerancia", det)
            if it == self.max_iter_corr:
                break
            self.corrigir(alvo, pose)
            rospy.sleep(0.5)
        return False

    # --------------------------------------------------------- TM (action)
    def enviar_tm(self, ctx, destino):
        self.tm_idle.clear()
        self.pick_started.clear()
        self.tm_falhou = False
        goal = TransferPieceGoal(peca=ctx[1], destino=destino)
        self.tm.send_goal(goal,
                          done_cb=lambda s, r: self._tm_done(ctx, s, r),
                          feedback_cb=lambda fb: self._tm_fb(ctx, fb))

    def _tm_fb(self, ctx, fb):
        if fb.etapa == "PICK_STARTED":
            self.log(*ctx, "HANDSHAKE", "pega_iniciada")
            self.pick_started.set()
        else:
            self.log(*ctx, "TM", "feedback", fb.etapa)

    def _tm_done(self, ctx, status, result):
        ok = status == GoalStatus.SUCCEEDED and result is not None and result.success
        self.tm_falhou = not ok
        self.log(*ctx, "HANDSHAKE", "conclusao" if ok else "falha_tm",
                 result.message if result else "sem_resultado")
        self.pick_started.set()      # destrava quem espera, mesmo em falha
        self.tm_idle.set()

    def esperar_tm_livre(self, ctx, motivo):
        if self.tm_idle.is_set():
            return
        self.log(*ctx, "ESPERA_TM", "inicio", motivo)
        while not rospy.is_shutdown() and not self.tm_idle.wait(0.2):
            pass
        self.log(*ctx, "ESPERA_TM", "fim", motivo)

    # --------------------------------------------------------------- missão
    def executar_missao(self, i, m):
        peca, org = m["peca"], m["origem"]
        info = self.catalogo[peca]
        ctx = (i, peca, org)

        # 1) Coleta (velocidade padrão)
        self.set_vel(self.vel_padrao)
        if not self.navegar(ctx, org, "IR_COLETA"):
            return False

        # 2) Carga
        self.log(*ctx, "CARREGANDO", "inicio", "%.1fs" % self.tempo_carga)
        rospy.sleep(self.tempo_carga)
        self.log(*ctx, "CARREGANDO", "fim")

        # 3) Entrega: velocidade da peça
        if self.esperar_tm_antes_c4:
            self.esperar_tm_livre(ctx, "antes_de_ir_para_C4")
        self.set_vel(info["max_vel"])
        self.log(*ctx, "IR_ENTREGA", "velocidade_max", "%.2f" % info["max_vel"])
        if not self.navegar(ctx, "C4", "IR_ENTREGA"):
            return False

        # 4) Validação de pose
        self.log(*ctx, "VALIDA_POSE", "inicio")
        if not self.validar_e_corrigir(ctx, "C4"):
            self.log(*ctx, "VALIDA_POSE", "FALHA", "pose fora da tolerância; missão abortada")
            return False

        # 5) Handshake
        self.esperar_tm_livre(ctx, "antes_do_handshake")
        self.log(*ctx, "HANDSHAKE", "inicio", "destino=%s" % info["destino_tm"])
        self.enviar_tm(ctx, info["destino_tm"])
        if not self.pick_started.wait(self.pick_timeout):
            self.log(*ctx, "HANDSHAKE", "timeout_pega")
            self.tm.cancel_goal()
            return False
        if self.tm_falhou:
            return False
        # Pioneer liberado: o laço segue para a próxima coleta enquanto o TM deposita
        self.log(*ctx, "LIBERADO", "pioneer_livre", "TM segue em paralelo")
        return True

    def run(self):
        if self.odom is None:
            rospy.loginfo("Aguardando odometria...")
            while self.odom is None and not rospy.is_shutdown():
                rospy.sleep(0.2)
        self.log("-", "-", "-", "BATERIA", "inicio")
        resumo = {"ok": 0, "falha": 0}
        for i, m in enumerate(self.fila):
            if rospy.is_shutdown():
                break
            try:
                ok = self.executar_missao(i, m)
            except Exception as e:
                rospy.logerr("Exceção na missão %d: %s", i, e)
                ok = False
            resumo["ok" if ok else "falha"] += 1
            if not ok:
                self.log(i, m["peca"], m["origem"], "MISSAO", "FALHA")
        self.esperar_tm_livre(("-", "-", "-"), "fim_da_bateria")
        self.set_vel(self.vel_padrao)
        self.log("-", "-", "-", "BATERIA", "fim", str(resumo))
        self.log.close()
        rospy.loginfo("Bateria concluída: %s", resumo)


if __name__ == "__main__":
    rospy.init_node("maestro")
    Maestro().run()
