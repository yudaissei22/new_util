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

(defun make-angle-vector-sequence ()
  
  )

(defun make-sequence-in-advance
  (jpos-deg tm-ms rc zmp-wc optional
   jpos-rad tm-s pos rpy zmp-rc robot-wc cog-wc
            &key
            (step 0.1)
            (x-max *x-max*)
            (choreonoid nil)
            (modified-p-path nil);;"package://baseball/config/modified-p/modified-p_-70-0--50.l")
            (lfoot-ik nil)
            (rfoot-ik nil)
            (zmp-offset #f(0 0 0))
            (p nil)
            )
  (if p
      (setq *p* p)
    (setq *p* (copy-seq *p-orig*))
    )
  ;; (when modified-p-path
  ;;   (load modified-p-path)
  ;;   (insert-modified-p *p* *modified-p*)
  ;;   )

  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
   (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                              :joint-class 6dof-joint)
   ;; (show-optimized-motion *p* :x-step step :x-max x-max :real-time nil :make-sequence t :choreonoid choreonoid :lfoot-ik lfoot-ik :rfoot-ik rfoot-ik :zmp-offset zmp-offset)
   )
  ;; 恐らくここのshow-optimized-motionでセグフォが起きている。
  ;; epochとかディレクトリを作るのは、defun show-optyimized-motionでなくて、それを読んでいるところでやるか。
  ;;   
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

;; mapcar func &rest arg-list
;; arg-listのそれぞれの要素にfuncをmapし、その全ての結果のリストを作る。
;; 例えば、
;; (mapcar #'(lambda (x) (* x x)) '(1 2 3))のように書く。
;; mapcarを使う前に、dolistを試すのがよいらしい。

;; lispは関数型言語であり、データに関数を適応できるというもの。
;; 関数は他のデータと同様に扱えるらしくて、関数に関数を代入できたりするそう。

