(defun experiment-init ()
  (setq *robot* *jaxon*)  
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


(defun experiment-angle-vector-sequence-full
  (jpos tm rc zmp optional
   &key (initial-time 10000) (final-time 5000) (log-fname "/tmp/angle-vector-sequence-full"))

  ;;(send *ri* :set-interpolation-mode :linear)
  (send *ri* :start-log)
  (send-angle-vector-sequence-full jpos tm rc zmp optional initial-time)
  (send *ri* :wait-interpolation-seq)
  (unix:usleep (* final-time 1000))
  (send *ri* :save-log log-fname)
  ;;(send *ri* :set-interpolation-mode :hoffarbib)
  )
(defun experiment-angle-vector-sequence-full-without-zmp
  (jpos tm rc optional
   &key (initial-time 10000) (final-time 5000) (log-fname "/tmp/angle-vector-sequence-full"))

  (send *ri* :start-log)
  (send-angle-vector-sequence-full jpos tm rc optional initial-time)
  (send *ri* :wait-interpolation-seq)
  (unix:usleep (* final-time 1000))
  (send *ri* :save-log log-fname)
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

  (setq tmp (copy-object *bspline-av-list*))
  ;; mslの12自由度を後から追加
  (setq *new-bspline-av-list*
	(mapcar
	 #'(lambda (av)
	     (setq av (concatenate float-vector av #f(-50 0 70 0 100 100 0 0 0 0 0 0)))
	     )
	 tmp)
	 )
  (replace jpos-deg (copy-object *new-bspline-av-list*))
  (replace tm-ms (copy-object *bspline-tm-list*))
  (replace rc (copy-object *bspline-rc-list*))
  (replace optional (copy-object *bspline-optional-list*))
  (format t "replace is done~%")
  
  nil
  )
