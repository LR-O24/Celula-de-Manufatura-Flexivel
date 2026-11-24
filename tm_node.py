#!/usr/bin/env python3
"""Nó do TM: Action Server 'transfer_piece' usando MoveIt! + gripper via SetIO."""
import sys

import actionlib
import moveit_commander
import rospy
import yaml

from celula_integrada.msg import (TransferPieceAction, TransferPieceFeedback,
                                  TransferPieceResult)

try:
    from tm_msgs.srv import SetIO, SetIORequest
    TEM_TM_MSGS = True
except ImportError:
    TEM_TM_MSGS = False


class TMNode:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        with open(rospy.get_param("~poses")) as f:
            self.cfg = yaml.safe_load(f)
        self.poses = self.cfg["poses"]
        self.simular_gripper = rospy.get_param("~simular_gripper", False)

        self.group = moveit_commander.MoveGroupCommander(
            rospy.get_param("~grupo", self.cfg.get("grupo_moveit", "tmr_arm")))
        self.group.set_max_velocity_scaling_factor(self.cfg.get("vel_scaling", 0.2))
        self.group.set_max_acceleration_scaling_factor(self.cfg.get("acc_scaling", 0.2))

        self.io = None
        g = self.cfg["gripper"]
        if not self.simular_gripper and TEM_TM_MSGS:
            rospy.wait_for_service(g["servico"], timeout=15)
            self.io = rospy.ServiceProxy(g["servico"], SetIO)
        else:
            rospy.logwarn("Gripper em modo SIMULADO (apenas log).")

        self.server = actionlib.SimpleActionServer(
            "transfer_piece", TransferPieceAction, self.execute, auto_start=False)
        self.server.start()
        rospy.loginfo("tm_node pronto (action: transfer_piece).")

    # ----------------------------------------------------------- primitivas
    def ir_para(self, nome):
        self.group.set_joint_value_target(self.poses[nome])
        ok = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        return ok

    def _set_io(self, estado):
        g = self.cfg["gripper"]
        if self.io is None:
            rospy.loginfo("[gripper simulado] estado=%s", estado)
        else:
            req = SetIORequest()
            req.module = SetIORequest.MODULE_ENDEFFECTOR
            req.type = SetIORequest.TYPE_DIGITAL_OUT
            req.pin = g["pin"]
            req.state = estado
            self.io(req)
        rospy.sleep(g.get("tempo_acionamento", 0.8))

    def fechar_gripper(self):
        self._set_io(self.cfg["gripper"]["fechar_estado"])

    def abrir_gripper(self):
        self._set_io(self.cfg["gripper"]["abrir_estado"])

    # --------------------------------------------------------------- action
    def execute(self, goal):
        fb = TransferPieceFeedback()
        res = TransferPieceResult()
        chave_place = "place_" + goal.destino

        def etapa(nome):
            fb.etapa = nome
            self.server.publish_feedback(fb)

        def abortar(msg):
            res.success, res.message = False, msg
            rospy.logerr("TM abortado: %s", msg)
            self.server.set_aborted(res)

        if chave_place not in self.poses:
            return abortar("destino inválido: %s" % goal.destino)

        try:
            etapa("IR_PRE_PICK")
            if not self.ir_para("pre_pick"):
                return abortar("falha ao ir para pre_pick")
            if self.server.is_preempt_requested():
                self.server.set_preempted(); return
            etapa("PICK")
            self.abrir_gripper()
            if not self.ir_para("pick"):
                return abortar("falha ao ir para pick")
            self.fechar_gripper()

            # Peça segura: a partir daqui o Pioneer pode partir (gatilho da concorrência)
            etapa("PICK_STARTED")
            if not self.ir_para("pre_pick"):
                return abortar("falha ao recuar para pre_pick")

            etapa("IR_PLACE")
            if not self.ir_para(chave_place):
                return abortar("falha ao ir para " + chave_place)
            etapa("PLACING")
            self.abrir_gripper()

            etapa("HOME")
            if not self.ir_para("home"):
                return abortar("falha ao voltar para home")
        except Exception as e:
            return abortar("exceção: %s" % e)

        res.success, res.message = True, "peça %s depositada (%s)" % (goal.peca, goal.destino)
        self.server.set_succeeded(res)


if __name__ == "__main__":
    rospy.init_node("tm_node")
    TMNode()
    rospy.spin()
