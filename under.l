;;必要なもののロード
(load "package://msl_hand_controller/euslisp/jaxon_red-interface.l")
(load "package://auto_stabilizer/euslisp/auto-stabilizer-interface.l")
(jaxon_red-init)
(setq *robot* *jaxon_red*)
(send *ri* :start-auto-balancer)
(send *ri* :start-st)

;;IRTviewerの表示
(objects *robot*)

;;まず、姿勢を作るところ
;;投げる前の姿勢を、生成し*ri*に記録する。これも関数化するもの？
;; (send *robot* :rarm-elbow-p :joint-angle 40)
;; (send *robot* :rarm-wrist-p :joint-angle 10)
;; (send *robot* :rarm-wrist-y :joint-angle -90)
;; (setq *before-swing* (send *robot* :angle-vector)) ;;グローバル変数にbefore-swingに設定できた。
;; (send *ri* :angle-vector (send *robot* :angle-vector)) ;;riへ送る

;;P,Dゲインを一括で弱める関数...後々に各関節ごとにしやすくするかも
(defun pdgain-rarm (percentage)
  (send *ri* :robothardwareservice_setservopgainpercentagewithtime :name :RARM_JOINT1 :percentage percentage :time 5)
  (send *ri* :robothardwareservice_setservodgainpercentagewithtime :name :RARM_JOINT1 :percentage percentage :time 5)

  (send *ri* :robothardwareservice_setservopgainpercentagewithtime :name :RARM_JOINT4 :percentage percentage :time 5)
  (send *ri* :robothardwareservice_setservodgainpercentagewithtime :name :RARM_JOINT4 :percentage percentage :time 5)

  (send *ri* :robothardwareservice_setservopgainpercentagewithtime :name :RARM_JOINT6 :percentage percentage :time 5)
  (send *ri* :robothardwareservice_setservodgainpercentagewithtime :name :RARM_JOINT6 :percentage percentage :time 5)
  )

;;個別のjointのゲイン調整用
(defun joint1-pdgain-rarm (p, d)
  (send *ri* :robothardwareservice_setservopgainpercentagewithtime :name :RARM_JOINT1 :percentage p :time 5)
  (send *ri* :robothardwareservice_setservodgainpercentagewithtime :name :RARM_JOINT1 :percentage d :time 5)
  )

(defun joint4-pdgain-rarm (p, d)
  (send *ri* :robothardwareservice_setservopgainpercentagewithtime :name :RARM_JOINT4 :percentage p :time 5)
  (send *ri* :robothardwareservice_setservodgainpercentagewithtime :name :RARM_JOINT4 :percentage d :time 5)
  )

(defun joint6-pdgain-rarm (p, d)
  (send *ri* :robothardwareservice_setservopgainpercentagewithtime :name :RARM_JOINT6 :percentage p :time 5)
  (send *ri* :robothardwareservice_setservodgainpercentagewithtime :name :RARM_JOINT6 :percentage d :time 5)
  )

;;投げる準備。投げる前を*ri*に入れてから、投げたあとを*robot*へ入れる。
(defun set-throw ()
  (send *robot* :rarm-elbow-p :joint-angle 40)
  (send *robot* :rarm-wrist-p :joint-angle 10)
  (send *robot* :rarm-wrist-y :joint-angle -90)
  (setq *before-throw* (send *robot* :angle-vector)) ;;グローバル変数にbefore-swingに設定できた。
  (send *ri* :angle-vector (send *robot* :angle-vector)) ;;riへ送る
   
  (send *robot* :rarm-elbow-p :joint-angle -40)
  (send *robot* :rarm-shoulder-p :joint-angle -10)
  (setq *after-throw* (send *robot* :angle-vector))  ;;after-swingに設定できた
  )

;;*ri*を時間指定して送る。
(defun throw (time)
  (send *ri* :angle-vector (send *robot* :angle-vector) time)
  )
