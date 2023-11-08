;; load here
(load "package://msl_hand_controller/euslisp/jaxon_red-interface.l")
(load "package://auto_stabilizer/euslisp/auto-stabilizer-interface.l")
(jaxon_red-init)
(setq *robot* *jaxon_red*)

(defun ab ()
  (send *ri* :start-auto-balancer)
  )

(defun st()
  (send *ri* :start-st)
  )

;; load avlist
(with-open-file (s "A" :direction :input)
		(let (buff))
		(while (setq buff (read-line s nil))
		  (print buff)))

;; send avlist 500Hz

