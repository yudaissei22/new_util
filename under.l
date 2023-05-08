(load "package://msl_hand_controller/euslisp/jaxon_red-interface.l")
(load "package://auto_stabilizer/euslisp/auto-stabilizer-interface.l")
(jaxon_red-init)
(setq *robot* *jaxon_red*)

;;IRTviewerの表示
(objects *robot*)


;;まず、姿勢を作るところ
;;投げる前の姿勢
(send *robot* :rarm-elbow-p :joint-angle 40)
(send *robot* :rarm-wrist :joint-angle -90)
(setq *before-swing* (send *robot* :angle-vector)) ;;グローバル変数にbefore-swingに設定できた。

(send *ri* :angle-vector (send *robot* :angle-vector))


;;投げたあとの姿勢



;;P,Dゲインを一括で弱める関数...後々に各関節ごとにしやすくするかも
(defun set-pdgain-rarm-roll (percentage)
  (send *ri* :robothardwareservice_setservopgainpercentagewithtime :name :RARM_JOINT1 :percentage percentage :time 5)
  (send *ri* :robothardwareservice_setservodgainpercentagewithtime :name :RARM_JOINT1 :percentage percentage :time 5)

  (send *ri* :robothardwareservice_setservopgainpercentagewithtime :name :RARM_JOINT4 :percentage percentage :time 5)
  (send *ri* :robothardwareservice_setservodgainpercentagewithtime :name :RARM_JOINT4 :percentage percentage :time 5)

  (send *ri* :robothardwareservice_setservopgainpercentagewithtime :name :RARM_JOINT6 :percentage percentage :time 5)
  (send *ri* :robothardwareservice_setservodgainpercentagewithtime :name :RARM_JOINT6 :percentage percentage :time 5)
)

;;投げる関数
(defun throw (time)
  send *ri* :angle-vector (send *robot* :angle-vector) time)
)
