;;必要なもののロード
(load "package://msl_hand_controller/euslisp/jaxon_red-interface.l")
(load "package://auto_stabilizer/euslisp/auto-stabilizer-interface.l")
(jaxon_red-init)
(setq *robot* *jaxon_red*)

;;IRTviewerの表示
(objects *robot*)


;;まず、姿勢を作るところ
;;投げる前の姿勢を、生成し*ri*に記録する。これも関数化するもの？
(send *robot* :rarm-elbow-p :joint-angle 40)
(send *robot* :rarm-wrist-p :joint-angle 10)
(send *robot* :rarm-wrist-y :joint-angle -90)

(setq *before-swing* (send *robot* :angle-vector)) ;;グローバル変数にbefore-swingに設定できた。
(send *ri* :angle-vector (send *robot* :angle-vector)) ;;riへ送る


;;P,Dゲインを一括で弱める関数...後々に各関節ごとにしやすくするかも
(defun set-pdgain-rarm-roll (percentage)
  (send *ri* :robothardwareservice_setservopgainpercentagewithtime :name :RARM_JOINT1 :percentage percentage :time 5)
  (send *ri* :robothardwareservice_setservodgainpercentagewithtime :name :RARM_JOINT1 :percentage percentage :time 5)

  (send *ri* :robothardwareservice_setservopgainpercentagewithtime :name :RARM_JOINT4 :percentage percentage :time 5)
  (send *ri* :robothardwareservice_setservodgainpercentagewithtime :name :RARM_JOINT4 :percentage percentage :time 5)

  (send *ri* :robothardwareservice_setservopgainpercentagewithtime :name :RARM_JOINT6 :percentage percentage :time 5)
  (send *ri* :robothardwareservice_setservodgainpercentagewithtime :name :RARM_JOINT6 :percentage percentage :time 5)
)



;;投げる関数timeで、投擲時間を決めれる...原理：beforeの状態からafterにすることで投げる
(defun throw (time)

;;投げたあとの姿勢...ここを目標にして投げる。
  (send *robot* :rarm-elbow-p :joint-angle -40)
  (send *robot* :rarm-shoulder-p :joint-angle -10)

  (setq *after-swing* (send *robot* :angle-vector))  ;;after-swingに設定できた
  (send *ri* :angle-vector (send *robot* :angle-vector) time)
)
