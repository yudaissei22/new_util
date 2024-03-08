;;必要なもののロード
(load "package://msl_hand_controller/euslisp/jaxon_red-interface.l")
(load "package://auto_stabilizer/euslisp/auto-stabilizer-interface.l")
(jaxon_red-init)
(setq *robot* *jaxon_red*)

;;これらは手で打ち込んだほうが良かった

(defun ab ()
  (send *ri* :start-auto-balancer)
  )

(defun st()
  (send *ri* :start-st)
  )

(defun walk()
  (send *ri* :start-log)
  (send *ri* :go-velocity 0.5 0 0)
  ;; (send *ri* :save-log "/tmp/for_si_latest")
  )
(defun stop()
  ;; 歩き終わるまで待つプログラムにするやつ。なんだっけ？シーケンシャルなんとかみたいな
  ;; 歩く速度をwalkの引数にして、動かせるようにすると便利かも(advance)
  (send *ri* :save-log "/tmp/for_si_latest")  
  )

