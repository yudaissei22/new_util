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

(defun send-angle-vector-sequence-full-without-zmp (jpos tm rc optional &optional (initial-time 10000))
  (send *ri* :angle-vector (send *robot* :angle-vector (car jpos)) initial-time) ;; initial-timeまでに最初の姿勢にする
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector-sequence-full jpos tm :root-coords rc :optional optional)
  )

(defun ast-make-sequence-in-advance
  (jpos-deg tm-ms rc zmp-wc optional   
            &key
            (step 0.1)
            (x-max *x-max*)
            )
  
  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
   (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                              :joint-class 6dof-joint))

  (replace jpos-deg (copy-object *bspline-av-list*))
  (replace tm-ms (copy-object *bspline-tm-list*))
  (replace rc (copy-object *bspline-rc-list*))
  (replace zmp-wc (copy-object *bspline-zmp-wc-list*))
  (replace optional (copy-object *bspline-optional-list*))

  (format t "replace is done~%")
  
  nil
  )

(defun ast-make-sequence-in-advance-without-zmp
  (jpos-deg tm-ms rc optional   
            &key
            (step 0.1)
            (x-max *x-max*)
            )
  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
   (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                              :joint-class 6dof-joint))

  (replace jpos-deg (copy-object *bspline-av-list*))
  (replace tm-ms (copy-object *bspline-tm-list*))
  (replace rc (copy-object *bspline-rc-list*))
  (replace optional (copy-object *bspline-optional-list*))
  (format t "replace is done~%")
  
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



