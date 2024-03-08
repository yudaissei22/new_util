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
  ;; (if p
  ;;     (setq *p* p)
  ;;   (setq *p* (copy-seq *p-orig*))
  ;;   )
  (when modified-p-path
    (load modified-p-path)
    (insert-modified-p *p* *modified-p*)
    )

  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
   (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                              :joint-class 6dof-joint)
   ;; (show-optimized-motion *p* :x-step step :x-max x-max :real-time nil :make-sequence t :choreonoid choreonoid :lfoot-ik lfoot-ik :rfoot-ik rfoot-ik :zmp-offset zmp-offset)
   )
  ;; 恐らくここのshow-optimized-motionでセグフォが起きている。
  ;; epochとかディレクトリを作るのは、defun show-optyimized-motionでなくて、それを読んでいるところでやるか。
  ;; 
  
  ;; (when (= (length tm-ms) (length *bspline-avsf-tm-ms*))
  ;;   (replace jpos-deg (copy-object *bspline-avsf-jpos-deg*))
  ;;   (replace tm-ms (copy-object *bspline-avsf-tm-ms*))
  ;;   (replace rc (copy-object *bspline-avsf-rc*))
  ;;   (replace zmp-wc (copy-object *bspline-avsf-zmp-wc*))
  ;;   (replace optional (copy-object *bspline-avsf-optional*))
  ;;   (replace jpos-rad (copy-object *bspline-avsf-jpos-rad*))
  ;;   (replace tm-s (copy-object *bspline-avsf-tm-s*))
  ;;   (replace pos (copy-object *bspline-avsf-pos*))
  ;;   (replace rpy (copy-object *bspline-avsf-rpy*))
  ;;   (replace zmp-rc (copy-object *bspline-avsf-zmp-rc*))
  ;;   (replace robot-wc (copy-object *bspline-avsf-wc*))
  ;;   (replace cog-wc (copy-object *bspline-avsf-cog-wc*))
  ;;   )
  nil
  )
