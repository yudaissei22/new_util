(defun red-init ()
  (load "package://msl_hand_controller/euslisp/jaxon_red_mslhand-interface.l")
  (load "package://auto_stabilizer/euslisp/auto-stabilizer-interface.l")
  (jaxon_red-init)
  (setq *robot* (jaxon_red))
  (objects (list *robot*))
  )

(defun experiment-init ()
  ;; (send *ri* :set-st-param :emergency-check-mode 0)
  ;;一個づつ読んだほうがよい。
  (send *ri* :set-auto-stabilizer-param :is-emergency-step-mode nil)
  ;; 寺澤さんはstabilizer使っていた。
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

(defun send-angle-vector-sequence-full-without-zmp-rc (jpos tm optional &optional (initial-time 10000))
  (send *ri* :angle-vector (send *robot* :angle-vector (car jpos)) initial-time) ;; initial-timeまでに最初の姿勢にする
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector-sequence-full jpos tm :optional optional)
  )


(defun experiment-angle-vector-sequence-full
  (jpos tm rc zmp optional
   &key (initial-time 10000) (final-time 5000) (log-fname "/tmp/angle-vector-sequence-full"))

  ;;(send *ri* :set-interpolation-mode :linear)
  (send *ri* :start-log)
  (send-angle-vector-sequence-full jpos tm rc zmp optional initial-time)
  (send *ri* :wait-interpolation-seq)
  ;; (unix:usleep (* final-time 1000))
  (unix:usleep (* final-time 1000))
  (unix:system "say one; sleep 1; say two; sleep 1; say three"send-angle-vector-sequence-full-without-zmp)
  (send *ri* :save-log log-fname)
  ;;(send *ri* :set-interpolation-mode :hoffarbib)
  )

(defun experiment-angle-vector-sequence-full-without-zmp
  (jpos tm rc optional
   &key (initial-time 10000) (final-time 5000) (log-fname "/tmp/angle-vector-sequence-full"))

  (send *ri* :start-log)
  (send-angle-vector-sequence-full-without-zmp jpos tm rc optional initial-time)
  (send *ri* :wait-interpolation-seq)
  (unix:usleep (* final-time 1000))
  (send *ri* :save-log log-fname)
  )

(defun experiment-angle-vector-sequence-full-without-zmp-rc
  (jpos tm optional
   &key (initial-time 10000) (final-time 5000) (log-fname "/tmp/angle-vector-sequence-full"))

  (send *ri* :start-log)
  (send-angle-vector-sequence-full-without-zmp-rc jpos tm optional initial-time)
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

;; (defun ast-make-sequence-in-advance-without-zmp-rc
;;   (jpos-deg tm-ms optional   
;;             &key
;;             (step 0.1)
;;             (x-max *x-max*)
;;             )
  
;;   (send *robot* :move-to (make-coords) :world)
;;   (with-append-root-joint
;;    (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
;;                               :joint-class 6dof-joint))

;;   (replace jpos-deg (copy-object *bspline-av-list*))
;;   (replace tm-ms (copy-object *bspline-tm-list*))
;;   (replace optional (copy-object *bspline-optional-list*))
;;   nil

;;   )

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
  (format t "length of tmp is ~s~%" (length (car tmp)))
  ;; mslの12自由度を後から追加
  (setq *new-bspline-av-list*
	(mapcar
	 #'(lambda (av)
	     (setq av (concatenate float-vector av #f(-50 0 70 0 100 100 0 0 0 0 0 0)))
	     )
	 tmp)
	)
  (format t "length of *new-bspline-av-list* is ~s~%" (length (car *new-bspline-av-list*)))  
  (replace jpos-deg (copy-object *new-bspline-av-list*))
  (format t "lenght of jpos-deg is ~s~%" (length (car jpos-deg)))
  (replace tm-ms (copy-object *bspline-tm-list*))
  (replace rc (copy-object *bspline-rc-list*))
  (replace optional (copy-object *bspline-optional-list*))
  (format t "replace is done~%")
  
  nil
  )

(defun ast-make-sequence-in-advance-without-zmp-rc
  (jpos-deg tm-ms optional   
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
  (replace optional (copy-object *bspline-optional-list*))
  
  nil
  )
