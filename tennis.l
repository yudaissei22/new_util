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
;; (with-open-file (s "test.txt" :direction :input)
		(loop
		 (let
		     ((buff (read-line s nil)))
		   (if (not buff)
		       (return))
		   ;;buffは一列ずつとってくるが、毎回上書きされてしまう
		   (setq *avlist*
			 (mnake-array '(241 33) :element-type :float)
			 )
		   (setf (aref *avlist* ) buff) ;;行列の要素にどうアクセスするのかよくわからない。一個一個の要素に代入できるの？

		  
		   ;; (setq *avlist* buff) 

		   
		   ;; (print *avlist*)		
		   ;;   ((buff (read-line s nil)))
		   ;; (if (not buff)
		   ;;     (return))
		   ;; (print buff)
		   )))






;; (print (aref *avlist* 0)) ;;arefでとっても、謎の48,50という値が得られる。
;; (print (elt *avlist* 0)) ;;eltでも同じだった
	       

    ;; (while (coerce (setq a (read-line s nil)) float-vector)
		;; (print a)
		;; (setq l (length a))
		;; (print "length list is")
		;; (print l) ;;275
    ;; (print s)

;; arefは。aref matrix 行 列で、その要素が取れる。

;; (setq a (open "Aug--7-17-21-37-2023-motion.avlist" ))
;; (print a)
;; aがfile-streamとして読み込まれてしまい、それ以上何もできない。どうやって、行列の要素にアクセスすればいいのかわからない。

;; I cant save avlist as global veriavles
;; 
