;; load here
(defun setup()
  (load "package://msl_hand_controller/euslisp/jaxon_red-interface.l")
  (load "package://auto_stabilizer/euslisp/auto-stabilizer-interface.l")
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )

(defun ab ()
  (send *ri* :start-auto-balancer)
  )

(defun st()
  (send *ri* :start-st)
  )

;; load avlist
(with-open-file (s "Aug--7-17-21-37-2023-motion.avlist" :direction :input)
		(while (setq a (read-line s nil))
		(print a)
		(setq l (length a))
		(print "length list is")
		(print l)
		  ))


;; I cant save avlist as global veriavlesc
