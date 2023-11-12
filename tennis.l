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
(defun wo()
  (with-open-file (s "Aug--7-17-21-37-2023-motion.avlist" :direction :input)
		(while (coerce (setq a (read-line s nil)) float-vector)
		(print a)
		;; (setq l (length a))
		;; (print "length list is")
		;; (print l) ;;275
		(print "aref is")
		(print (aref a 0)))))
;; arefは。aref matrix 行 列で、その要素が取れる。

(setq a (open "Aug--7-17-21-37-2023-motion.avlist" ))
(print a)
;; aがfile-streamとして読み込まれてしまい、それ以上何もできない。どうやって、行列の要素にアクセスすればいいのかわからない。

;; I cant save avlist as global veriavles
;; 
