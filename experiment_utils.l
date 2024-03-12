;; angle-vectorを送ればよいので、それをchoreonoidに送るためのプログラムを書く。

(load "package://msl_hand_controller/euslisp/jaxon_red-interface.l")
(load "package://auto_stabilizer/euslisp/auto-stabilizer-interface.l")

(defun experiment-init ()
  (setq *robot* *jaxon_red*)  
  (send *ri* :set-st-param :emergency-check-mode 0)
  (send *ri* :set-interpolation-mode :linear)
  )


(defun send-angle-vector-sequence-full (jpos tm rc zmp optional &optional (initial-time 10000))
  (send *ri* :angle-vector (send *robot* :angle-vector (car jpos)) initial-time) ;; initial-timeまでに最初の姿勢にする
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector-sequence-full jpos tm :root-coords rc :zmp zmp :optional optional)
  )


(defun ast-make-sequence-in-advance
  (jpos-deg tm-ms rc zmp-wc optional   
            &key
            (step 0.1)
            (x-max *x-max*)
            )
  (if p
      (setq *p* p)
    (setq *p* (copy-seq *p-orig*))
    )
  
  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
   (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                              :joint-class 6dof-joint)
   )

  (replace jpos-deg (copy-object *bspline-av-list*))
  (replace tm-ms (copy-object *bspline-tm-list*))
  (replace rc (copy-object *bspline-rc-list*))
  (replace zmp-ws (copy-object *bspline-zmp-wc-list*))
  (replace optional (copy-object *bspline-optional-list*))

  (format t "replace is done~%")
  
  nil
  )


(defun make-sequence-in-advance
  (jpos-deg tm-ms rc zmp-wc optional
   jpos-rad tm-s pos rpy zmp-rc robot-wc cog-wc
            &key
            (step 0.1)
            (x-max *x-max*)
            (choreonoid nil)
            (modified-p-path nil)
            (lfoot-ik nil)
            (rfoot-ik nil)
            (zmp-offset #f(0 0 0))
            (p nil)
            )
  (if p
      (setq *p* p)
    (setq *p* (copy-seq *p-orig*))
    )
  (setq jops-deg (concatenate float-vector #f(#f(-50 0 70 0 100 100 0 0 0 0 0 0)))
		   )

  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
   (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                              :joint-class 6dof-joint)
   )

  (replace j)
  nil
  )

(defun add-msl ()
  (setq *bspline-av-list*
	(mapcar
	 #'(lambda (av)
	     (setq av (concatenate float-vector av #f(-50 0 70 0 100 100 0 0 0 0 0 0)))
	     )
	 *bspline-av-list*)
	 )
  )



(let* ((t-max *x-max-of-p-orig*) (step 0.05) (seq-num (round (+ 1 (/ t-max step)))))
  (setq *exp-jpos-deg1* (make-list seq-num))
  (setq *exp-tm-ms1* (make-list seq-num))
  (setq *exp-rc1* (make-list seq-num)) 
  (setq *exp-zmp-wc1* (make-list seq-num))
  (setq *exp-optional1* (make-list seq-num))
  (setq *exp-jpos-rad1* (make-list seq-num))
  (setq *exp-tm-s1* (make-list seq-num))
  (setq *exp-pos1* (make-list seq-num))
  (setq *exp-rpy1* (make-list seq-num))
  (setq *exp-zmp-rc1* (make-list seq-num))
  (setq *exp-wc1* (make-list seq-num))
  (setq *exp-cog-wc1* (make-list seq-num))
  (make-sequence-in-advance
   *exp-jpos-deg1* *exp-tm-ms1* *exp-rc1* *exp-zmp-wc1* *exp-optional1*
   *exp-jpos-rad1* *exp-tm-s1* *exp-pos1* *exp-rpy1* *exp-zmp-rc1* *exp-wc1* *exp-cog-wc1*
   :step step :x-max t-max :choreonoid t :p *p* :lfoot-ik t :rfoot-ik t)
  )
;; mapcar func &rest arg-list
;; arg-listのそれぞれの要素にfuncをmapし、その全ての結果のリストを作る。
;; 例えば、
;; (mapcar #'(lambda (x) (* x x)) '(1 2 3))のように書く。
;; mapcarを使う前に、dolistを試すのがよいらしい。

;; lispは関数型言語であり、データに関数を適応できるというもの。
;; 関数は他のデータと同様に扱えるらしくて、関数に関数を代入できたりするそう。

