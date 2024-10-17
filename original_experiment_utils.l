(load "online_trajectory_modification.l")
(comp::compile-file-if-src-newer "online_trajectory_modification.l" nil)
(load "online_trajectory_modification.so")
(require "package://eus_qp/euslisp/eiquadprog.l")

(defun experiment-init ()
  (jaxon_red-init)
  ;; (send *ri* :start-grasp)
  ;; thkのhandでないため、コメントアウト、angle-vectorにmsl_hand用のangle-vectorのlistを書いた。
  (send *ri* :set-st-param :emergency-check-mode 0)
  (send *ri* :set-interpolation-mode :linear)
  (when (not (boundp '*append-list*))
    (setq *append-list* nil)
    )
  (send *ri* :stop-st)
  (format t "stop st~%")
  (send *ri* :stop-auto-balancer) ;; 実機ではコメントアウトしたほうがいかも？
  (format t "stop auto-balancer ~%")
  )

(defun send-pose (&optional (tm 10000))
  (send *ri* :angle-vector (send *robot* :angle-vector) tm)
  )



(defun experiment-loadpattern
  (sequence-fname &key (log-fname "/tmp/loadpattern") (reverse-fname nil #|"/tmp/reverse"|#))
  "Sequence files consist of ***.pos, ***.zmp, ***.waist, ***.optionaldata.
   sequence-fname (***) should be full path.
   revese is the sequence file for returning to the initial pose."
  (send *ri* :start-log)
  (send *ri* :load-pattern sequence-fname 1)
  (send *ri* :wait-interpolation-seq)
  (when reverse-fname
    (send *ri* :load-pattern reverse-fname 3)
    (send *ri* :wait-interpolation-seq)
    )
  (send *ri* :save-log log-fname)
  )





(defun send-angle-vector-sequence-full (jpos tm rc zmp optional &optional (initial-time 10000))
  (send *ri* :angle-vector (send *robot* :angle-vector (car jpos)) initial-time) ;; initial-timeまでに最初の姿勢にする
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector-sequence-full jpos tm :root-coords rc :zmp zmp :optional optional)
  )


(defun send-angle-vector-sequence-full-no-wait (jpos tm rc zmp optional)
  (send *ri* :angle-vector-sequence-full jpos tm :root-coords rc :zmp zmp :optional optional)
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


(defun experiment-angle-vector-sequence-full-switch
  (jpos1 tm1 rc1 zmp1 optional1
   jpos2 tm2 rc2 zmp2 optional2
   &key (switch-time 0.0) (initial-time 10000) (log-fname "/tmp/angle-vector-sequence-full-switch"))

  (let* ((step (* 1e-3 (car tm1))) ;; [s]
         (nthcdr-num (round (+ (/ switch-time step) 2 #|1|#))))
    (setq jpos2 (nthcdr nthcdr-num jpos2))
    (setq tm2 (nthcdr nthcdr-num tm2))
    (setq rc2 (nthcdr nthcdr-num rc2))
    (setq optional2 (nthcdr nthcdr-num optional2))
    (send *ri* :start-log)
    (let* ((timer (instance mtimer :init)))
      (send-angle-vector-sequence-full jpos1 tm1 rc1 zmp1 optional1 initial-time)
      (while (< (send timer :stop) (+ (* 1e-3 initial-time) switch-time))
        )
      (send-angle-vector-sequence-full-no-wait jpos2 tm2 rc2 zmp2 optional2)
      (send *ri* :wait-interpolation-seq)
      (send *ri* :save-log log-fname)
      )
    )
  )



(defun experiment-online-trajectory-modification
  (jpos tm pos rpy zmp optional t-max x-hit qp-time
   &key (initial-time 10000) (final-time 5000)
        (log-fname "/tmp/online-trajectory-modification")
        (test-target #f(0 0 -0.0026 0 0 0))
        (real t)
        (debug? t)
        (append nil)
        (lfoot-ik nil)
        (rfoot-ik nil)
        )
  (if (eps= (apply #'+ (cdr tm)) t-max)
      (progn
        (setq *p* (copy-seq *p-orig*))
        (setq *qp-ss-list* nil)
        (send *robot* :angle-vector (map float-vector #'rad2deg (car jpos)))
        (if real
            (progn
              (send *ri* :start-log)
              (send *ri* :angle-vector (send *robot* :angle-vector) initial-time) ;; initial-timeまでに最初の姿勢にする
              (send *ri* :wait-interpolation)
              )
          (progn
            (send *robot* :angle-vector)
            (send *robot* :fix-leg-to-coords *lfoot* :lleg)
            (send *irtviewer* :draw-objects)
            )
          )
        (let* ((timer (instance mtimer :init))
               (timer2 (instance mtimer :init))
               (step (car tm))
               (load-flag t)
               (send-flag t)
               (bspline
                (progn
                  (if (send (car (send *robot* :links)) :joint) ;; with-append-root-jointしてあればt
                      (progn
                        (mapcar #'(lambda (k)
                                    (instance basic-spline :init
                                              :id-max *id-max* :recursive-order *recursive-order*
                                              :x-min *x-min* :x-max t-max))
                                (flatten *jlist* (make-list 6)))) ;; root-jointの6個分付け加える
                    (progn
                      (mapcar #'(lambda (k)
                                  (instance basic-spline :init
                                            :id-max *id-max* :recursive-order *recursive-order*
                                            :x-min *x-min* :x-max t-max))
                              *jlist*))
                    )
                  ))

               (send-num 3)

               ;; ;; before switching
               ;; (switch-id (round (+ (/ switch-time step) 1))) ;; switch-id => send-num
               (exp-jpos-orig (subseq jpos 0 send-num))
               (exp-tm-orig (subseq tm 0 send-num))
               (exp-pos-orig (subseq pos 0 send-num))
               (exp-rpy-orig (subseq rpy 0 send-num))
               (exp-zmp-orig (subseq zmp 0 send-num))
               (exp-optional-orig (subseq optional 0 send-num))
               (exp-vel-orig (make-list send-num :initial-element (instantiate float-vector (length (car exp-jpos-orig)))))
               (exp-torque-orig (make-list send-num :initial-element (instantiate float-vector (length (car exp-jpos-orig)))))
               (exp-acc-orig (make-list send-num :initial-element (instantiate float-vector 3)))
               (exp-wrench-orig (make-list send-num :initial-element (instantiate float-vector (length (car exp-jpos-orig)))))

               ;; ;; old ;; switch-id => send-num
               (exp-jpos-old (make-list send-num :initial-element (car jpos)))
               ;; ;; new
               (exp-jpos-new-deg nil)
               (exp-jpos-new-rad nil)

               (jvss (instance std_msgs::float64multiarray :init))
               (vels (instance std_msgs::float64multiarray :init))
               (torques (instance std_msgs::float64multiarray :init))
               (poss (instance std_msgs::float64multiarray :init))
               (rpys (instance std_msgs::float64multiarray :init))
               (accs (instance std_msgs::float64multiarray :init))
               (zmps (instance std_msgs::float64multiarray :init))
               (wrenchs (instance std_msgs::float64multiarray :init))
               (optionals (instance std_msgs::float64multiarray :init))
               array-jvs array-tm array-pos array-rpy array-zmp array-optional
               array-vel array-torque array-acc array-wrench

               exp-jpos exp-pos exp-rpy exp-zmp exp-optional exp-tm
               (exp-vel (make-list send-num :initial-element (instantiate float-vector (length (car exp-jpos-orig)))))
               (exp-torque (make-list send-num :initial-element (instantiate float-vector (length (car exp-jpos-orig)))))
               (exp-acc (make-list send-num :initial-element (instantiate float-vector 3)))
               (exp-wrench (make-list send-num :initial-element (instantiate float-vector (length (car exp-jpos-orig)))))
               (tms (coerce exp-tm-orig float-vector))

               (choreonoid (if (= (length (car exp-jpos-orig)) (length (send *robot* :angle-vector))) nil t))
               j
               )
          ;; jvss
          (setq array-jvs (apply #'matrix exp-jpos-orig))
          (send jvss :layout :dim (list
                                   (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-jvs)) :stride (* (nth 0 (array-dimensions array-jvs)) (nth 1 (array-dimensions array-jvs))))
                                   (instance std_msgs::multiArraydimension :init :label "jvs length" :size (nth 1 (array-dimensions array-jvs)) :stride (nth 1 (array-dimensions array-jvs)))))
          (send jvss :data (array-entity array-jvs))
          ;; vels
          (setq array-vel (apply #'matrix exp-vel-orig))
          (send vels :layout :dim (list
                                   (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-vel)) :stride (* (nth 0 (array-dimensions array-vel)) (nth 1 (array-dimensions array-vel))))
                                   (instance std_msgs::multiArraydimension :init :label "vel length" :size (nth 1 (array-dimensions array-vel)) :stride (nth 1 (array-dimensions array-vel)))))
          (send vels :data (array-entity array-vel))
          ;; torques
          (setq array-torque (apply #'matrix exp-torque-orig))
          (send torques :layout :dim (list
                                      (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-torque)) :stride (* (nth 0 (array-dimensions array-torque)) (nth 1 (array-dimensions array-torque))))
                                      (instance std_msgs::multiArraydimension :init :label "torque length" :size (nth 1 (array-dimensions array-torque)) :stride (nth 1 (array-dimensions array-torque)))))
          (send torques :data (array-entity array-torque))
          ;; poss
          (setq array-pos (apply #'matrix exp-pos-orig))
          (send poss :layout :dim (list
                                   (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-pos)) :stride (* (nth 0 (array-dimensions array-pos)) (nth 1 (array-dimensions array-pos))))
                                   (instance std_msgs::multiArraydimension :init :label "pos length" :size (nth 1 (array-dimensions array-pos)) :stride (nth 1 (array-dimensions array-pos)))))
          (send poss :data (array-entity array-pos))
          ;; rpys
          (setq array-rpy (apply #'matrix exp-rpy-orig))
          (send rpys :layout :dim (list
                                   (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-rpy)) :stride (* (nth 0 (array-dimensions array-rpy)) (nth 1 (array-dimensions array-rpy))))
                                   (instance std_msgs::multiArraydimension :init :label "rpy length" :size (nth 1 (array-dimensions array-rpy)) :stride (nth 1 (array-dimensions array-rpy)))))
          (send rpys :data (array-entity array-rpy))
          ;; accs
          (setq array-acc (apply #'matrix exp-acc-orig))
          (send accs :layout :dim (list
                                   (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-acc)) :stride (* (nth 0 (array-dimensions array-acc)) (nth 1 (array-dimensions array-acc))))
                                   (instance std_msgs::multiArraydimension :init :label "acc length" :size (nth 1 (array-dimensions array-acc)) :stride (nth 1 (array-dimensions array-acc)))))
          (send accs :data (array-entity array-acc))
          ;; zmps
          (setq array-zmp (apply #'matrix exp-zmp-orig))
          (send zmps :layout :dim (list
                                   (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-zmp)) :stride (* (nth 0 (array-dimensions array-zmp)) (nth 1 (array-dimensions array-zmp))))
                                   (instance std_msgs::multiArraydimension :init :label "zmp length" :size (nth 1 (array-dimensions array-zmp)) :stride (nth 1 (array-dimensions array-zmp)))))
          (send zmps :data (array-entity array-zmp))
          ;; wrenchs
          (setq array-wrench (apply #'matrix exp-wrench-orig))
          (send wrenchs :layout :dim (list
                                      (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-wrench)) :stride (* (nth 0 (array-dimensions array-wrench)) (nth 1 (array-dimensions array-wrench))))
                                      (instance std_msgs::multiArraydimension :init :label "wrench length" :size (nth 1 (array-dimensions array-wrench)) :stride (nth 1 (array-dimensions array-wrench)))))
          (send wrenchs :data (array-entity array-wrench))
          ;; optionals
          (setq array-optional (apply #'matrix exp-optional-orig))
          (send optionals :layout :dim (list
                                        (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-optional)) :stride (* (nth 0 (array-dimensions array-optional)) (nth 1 (array-dimensions array-optional))))
                                        (instance std_msgs::multiArraydimension :init :label "optional length" :size (nth 1 (array-dimensions array-optional)) :stride (nth 1 (array-dimensions array-optional)))))
          (send optionals :data (array-entity array-optional))
          ;; tms
          (setq tms (coerce exp-tm-orig float-vector))

          (send timer :start)
          (when real
            (send *ri* :sequenceplayerservice_setjointanglessequencefull :jvss jvss :vels vels :torques torques :poss poss :rpys rpys :accs accs :zmps zmps :wrenchs wrenchs :optionals optionals :tms tms)
            )

          (setq *dp-list* nil)
          (dolist (i (my-range (- (round (/ t-max step)) (- send-num 1))))
            (setq j (+ i 1)) ;; j = i + 1
            ;; (format t "timer-start = ~a~%" (send timer :stop))
            (let*
                (dp
                 (target test-target) ;;後でiに応じて変更する
                 )
              (send timer2 :start)
              (setq dp
                    (if (< (send timer :stop) qp-time
                           )
                        (progn
                          (calc-dp-from-dr target (* step j) x-hit :bspline bspline :debug? debug?)
                          )
                      (instantiate float-vector (length *p*))
                      )
                    )
              (send timer2 :stop)
              (format t "timer2 = ~a~%" (send timer2 :stop))
              (push dp *dp-list*)
              (print dp)
              (setq *p* (v+ *p* dp))
              )

            (let* (id (cnt 0))
              (setq exp-jpos nil)

              (while (and (< cnt send-num) (<= (* step (+ j cnt)) (+ t-max 1e-5)))
                (setq id 0)
                (map float-vector
                     #'(lambda (jl bs)
                         (send jl :joint-angle (send bs :calc (* step (+ j cnt)) (subseq *p* id (setq id (+ id *id-max*)))))
                         )
                     *jlist* bspline)

                (send *robot* :move-to (nth j *exp-wc1*) :world) ;; *exp-wc1*を使っているがやむなし?
                (when lfoot-ik
                  (let* ((new-lfoot (copy-object *lfoot*)))
                    (when (vectorp lfoot-ik)
                      (send new-lfoot :translate  (subseq lfoot-ik 0 3) :world)
                      (when (= (length lfoot-ik) 4)
                        (send new-lfoot :rotate (deg2rad (aref lfoot-ik 3)) :z)
                        )
                      )
                    (send *robot* :lleg :inverse-kinematics new-lfoot :rotation-axis t)
                    )
                  )
                (when rfoot-ik
                  (let* ((new-rfoot (copy-object *rfoot*)))
                    (when (vectorp rfoot-ik)
                      (send new-rfoot :translate  (subseq rfoot-ik 0 3) :world)
                      (when (= (length rfoot-ik) 4)
                        (send new-rfoot :rotate (deg2rad (aref rfoot-ik 3)) :z)
                        )
                      )
                    (send *robot* :rleg :inverse-kinematics new-rfoot :rotation-axis t)
                    )
                  )
                (setq exp-jpos-new-deg (concatenate float-vector (send *robot* :angle-vector) (if choreonoid #f(-80 80 -80 80) nil)))
                (setq exp-jpos-new-rad (map float-vector #'deg2rad exp-jpos-new-deg))

                (when (= cnt 0)
                  (setq exp-jpos-old (append (cdr exp-jpos-old) (list exp-jpos-new-rad)))
                  ;;(send *robot* :angle-vector exp-jpos-new-deg)
                  (send *irtviewer* :draw-objects)
                  )

                (setq exp-jpos (append exp-jpos (list exp-jpos-new-rad)))
                (incf cnt)
                )
              )
            (push (send *hit-point* :copy-worldcoords) *qp-ss-list*)
            (setq exp-pos (subseq *exp-pos1* j (+ j send-num)))
            (setq exp-rpy (subseq *exp-rpy1* j (+ j send-num)))
            (setq exp-zmp (subseq *exp-zmp-rc1* j (+ j send-num)))
            (setq exp-optional (subseq *exp-optional1* j (+ j send-num)))
            (setq exp-tm (subseq *exp-tm-s1* j (+ j send-num)))

            (format t "exp-jpos length = ~a, exp-pos length = ~a, exp-tm length = ~a~%" (length exp-jpos) (length exp-pos) (length exp-tm))

            ;; jvss
            (setq array-jvs (apply #'matrix exp-jpos))
            (send jvss :layout :dim (list
                                     (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-jvs)) :stride (* (nth 0 (array-dimensions array-jvs)) (nth 1 (array-dimensions array-jvs))))
                                     (instance std_msgs::multiArraydimension :init :label "jvs length" :size (nth 1 (array-dimensions array-jvs)) :stride (nth 1 (array-dimensions array-jvs)))))
            (send jvss :data (array-entity array-jvs))
            ;; vels
            (setq array-vel (apply #'matrix exp-vel))
            (send vels :layout :dim (list
                                     (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-vel)) :stride (* (nth 0 (array-dimensions array-vel)) (nth 1 (array-dimensions array-vel))))
                                     (instance std_msgs::multiArraydimension :init :label "vel length" :size (nth 1 (array-dimensions array-vel)) :stride (nth 1 (array-dimensions array-vel)))))
            (send vels :data (array-entity array-vel))
            ;; torques
            (setq array-torque (apply #'matrix exp-torque))
            (send torques :layout :dim (list
                                        (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-torque)) :stride (* (nth 0 (array-dimensions array-torque)) (nth 1 (array-dimensions array-torque))))
                                        (instance std_msgs::multiArraydimension :init :label "torque length" :size (nth 1 (array-dimensions array-torque)) :stride (nth 1 (array-dimensions array-torque)))))
            (send torques :data (array-entity array-torque))
            ;; poss
            (setq array-pos (apply #'matrix exp-pos))
            (send poss :layout :dim (list
                                     (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-pos)) :stride (* (nth 0 (array-dimensions array-pos)) (nth 1 (array-dimensions array-pos))))
                                     (instance std_msgs::multiArraydimension :init :label "pos length" :size (nth 1 (array-dimensions array-pos)) :stride (nth 1 (array-dimensions array-pos)))))
            (send poss :data (array-entity array-pos))
            ;; rpys
            (setq array-rpy (apply #'matrix exp-rpy))
            (send rpys :layout :dim (list
                                     (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-rpy)) :stride (* (nth 0 (array-dimensions array-rpy)) (nth 1 (array-dimensions array-rpy))))
                                     (instance std_msgs::multiArraydimension :init :label "rpy length" :size (nth 1 (array-dimensions array-rpy)) :stride (nth 1 (array-dimensions array-rpy)))))
            (send rpys :data (array-entity array-rpy))
            ;; accs
            (setq array-acc (apply #'matrix exp-acc))
            (send accs :layout :dim (list
                                     (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-acc)) :stride (* (nth 0 (array-dimensions array-acc)) (nth 1 (array-dimensions array-acc))))
                                     (instance std_msgs::multiArraydimension :init :label "acc length" :size (nth 1 (array-dimensions array-acc)) :stride (nth 1 (array-dimensions array-acc)))))
            (send accs :data (array-entity array-acc))
            ;; zmps
            (setq array-zmp (apply #'matrix exp-zmp))
            (send zmps :layout :dim (list
                                     (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-zmp)) :stride (* (nth 0 (array-dimensions array-zmp)) (nth 1 (array-dimensions array-zmp))))
                                     (instance std_msgs::multiArraydimension :init :label "zmp length" :size (nth 1 (array-dimensions array-zmp)) :stride (nth 1 (array-dimensions array-zmp)))))
            (send zmps :data (array-entity array-zmp))
            ;; wrenchs
            (setq array-wrench (apply #'matrix exp-wrench))
            (send wrenchs :layout :dim (list
                                        (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-wrench)) :stride (* (nth 0 (array-dimensions array-wrench)) (nth 1 (array-dimensions array-wrench))))
                                        (instance std_msgs::multiArraydimension :init :label "wrench length" :size (nth 1 (array-dimensions array-wrench)) :stride (nth 1 (array-dimensions array-wrench)))))
            (send wrenchs :data (array-entity array-wrench))
            ;; optionals
            (setq array-optional (apply #'matrix exp-optional))
            (send optionals :layout :dim (list
                                          (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-optional)) :stride (* (nth 0 (array-dimensions array-optional)) (nth 1 (array-dimensions array-optional))))
                                          (instance std_msgs::multiArraydimension :init :label "optional length" :size (nth 1 (array-dimensions array-optional)) :stride (nth 1 (array-dimensions array-optional)))))
            (send optionals :data (array-entity array-optional))
            ;; tms
            (setq tms (coerce exp-tm float-vector))
            (format t "timer = ~a,  step*(i+1) = ~a~%" (send timer :stop) (* step j))

            (while (<= (send timer :stop) (* step j))
              (unix:usleep 10)
              )
            (if real
                (send *ri* :sequenceplayerservice_setjointanglessequencefull :jvss jvss :vels vels :torques torques :poss poss :rpys rpys :accs accs :zmps zmps :wrenchs wrenchs :optionals optionals :tms tms)
              (progn
                (format t "send ~a setJointAnglesSequenceFull timer = ~a~%" (* step j) (send timer :stop))
                ;; (send *robot* :angle-vector (map float-vector #'rad2deg (car exp-jpos)))
                ;; (send *robot* :fix-leg-to-coords *lfoot* :lleg)
                ;; (send *irtviewer* :draw-objects)
                )
              )
            (format t "timer-end = ~a~%~%" (send timer :stop))
            ) ;; dolist

          (while (<= (send timer :stop) t-max)
            (when (and send-flag (consp append))
              (dolist (als append) (append-last-sequence (nth 0 als) :tm (nth 1 als) :append t :choreonoid choreonoid))
              (setq exp-jpos (nthcdr (- (length *exp-jpos-rad1*) (length append)) *exp-jpos-rad1*))
              (setq exp-pos (nthcdr (- (length *exp-pos1*) (length append)) *exp-pos1*))
              (setq exp-rpy (nthcdr (- (length *exp-rpy1*) (length append)) *exp-rpy1*))
              (setq exp-zmp (nthcdr (- (length *exp-zmp-rc1*) (length append)) *exp-zmp-rc1*))
              (setq exp-optional (nthcdr (- (length *exp-optional1*) (length append)) *exp-optional1*))
              (setq exp-tm (nthcdr (- (length *exp-tm-s1*) (length append)) *exp-tm-s1*))
              (setq exp-vel (make-list (length append) :initial-element (instantiate float-vector (length (car exp-jpos-orig)))))
              (setq exp-torque (make-list (length append) :initial-element (instantiate float-vector (length (car exp-jpos-orig)))))
              (setq exp-acc (make-list (length append) :initial-element (instantiate float-vector 3)))
              (setq exp-wrench (make-list (length append) :initial-element (instantiate float-vector (length (car exp-jpos-orig)))))
              (format t "exp-jpos length = ~a, exp-pos length = ~a, exp-tm length = ~a~%" (length exp-jpos) (length exp-pos) (length exp-tm))

              ;; jvss
              (setq array-jvs (apply #'matrix exp-jpos))
              (send jvss :layout :dim (list
                                       (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-jvs)) :stride (* (nth 0 (array-dimensions array-jvs)) (nth 1 (array-dimensions array-jvs))))
                                       (instance std_msgs::multiArraydimension :init :label "jvs length" :size (nth 1 (array-dimensions array-jvs)) :stride (nth 1 (array-dimensions array-jvs)))))
              (send jvss :data (array-entity array-jvs))
              ;; vels
              (setq array-vel (apply #'matrix exp-vel))
              (send vels :layout :dim (list
                                       (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-vel)) :stride (* (nth 0 (array-dimensions array-vel)) (nth 1 (array-dimensions array-vel))))
                                       (instance std_msgs::multiArraydimension :init :label "vel length" :size (nth 1 (array-dimensions array-vel)) :stride (nth 1 (array-dimensions array-vel)))))
              (send vels :data (array-entity array-vel))
              ;; torques
              (setq array-torque (apply #'matrix exp-torque))
              (send torques :layout :dim (list
                                          (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-torque)) :stride (* (nth 0 (array-dimensions array-torque)) (nth 1 (array-dimensions array-torque))))
                                          (instance std_msgs::multiArraydimension :init :label "torque length" :size (nth 1 (array-dimensions array-torque)) :stride (nth 1 (array-dimensions array-torque)))))
              (send torques :data (array-entity array-torque))
              ;; poss
              (setq array-pos (apply #'matrix exp-pos))
              (send poss :layout :dim (list
                                       (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-pos)) :stride (* (nth 0 (array-dimensions array-pos)) (nth 1 (array-dimensions array-pos))))
                                       (instance std_msgs::multiArraydimension :init :label "pos length" :size (nth 1 (array-dimensions array-pos)) :stride (nth 1 (array-dimensions array-pos)))))
              (send poss :data (array-entity array-pos))
              ;; rpys
              (setq array-rpy (apply #'matrix exp-rpy))
              (send rpys :layout :dim (list
                                       (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-rpy)) :stride (* (nth 0 (array-dimensions array-rpy)) (nth 1 (array-dimensions array-rpy))))
                                       (instance std_msgs::multiArraydimension :init :label "rpy length" :size (nth 1 (array-dimensions array-rpy)) :stride (nth 1 (array-dimensions array-rpy)))))
              (send rpys :data (array-entity array-rpy))
              ;; accs
              (setq array-acc (apply #'matrix exp-acc))
              (send accs :layout :dim (list
                                       (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-acc)) :stride (* (nth 0 (array-dimensions array-acc)) (nth 1 (array-dimensions array-acc))))
                                       (instance std_msgs::multiArraydimension :init :label "acc length" :size (nth 1 (array-dimensions array-acc)) :stride (nth 1 (array-dimensions array-acc)))))
              (send accs :data (array-entity array-acc))
              ;; zmps
              (setq array-zmp (apply #'matrix exp-zmp))
              (send zmps :layout :dim (list
                                       (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-zmp)) :stride (* (nth 0 (array-dimensions array-zmp)) (nth 1 (array-dimensions array-zmp))))
                                       (instance std_msgs::multiArraydimension :init :label "zmp length" :size (nth 1 (array-dimensions array-zmp)) :stride (nth 1 (array-dimensions array-zmp)))))
              (send zmps :data (array-entity array-zmp))
              ;; wrenchs
              (setq array-wrench (apply #'matrix exp-wrench))
              (send wrenchs :layout :dim (list
                                          (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-wrench)) :stride (* (nth 0 (array-dimensions array-wrench)) (nth 1 (array-dimensions array-wrench))))
                                          (instance std_msgs::multiArraydimension :init :label "wrench length" :size (nth 1 (array-dimensions array-wrench)) :stride (nth 1 (array-dimensions array-wrench)))))
              (send wrenchs :data (array-entity array-wrench))
              ;; optionals
              (setq array-optional (apply #'matrix exp-optional))
              (send optionals :layout :dim (list
                                            (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-optional)) :stride (* (nth 0 (array-dimensions array-optional)) (nth 1 (array-dimensions array-optional))))
                                            (instance std_msgs::multiArraydimension :init :label "optional length" :size (nth 1 (array-dimensions array-optional)) :stride (nth 1 (array-dimensions array-optional)))))
              (send optionals :data (array-entity array-optional))
              ;; tms
              (setq tms (coerce exp-tm float-vector))

              (setq *exp-jpos-deg1* (subseq *exp-jpos-deg1* 0 (- (length *exp-jpos-deg1*) (length append))))
              (setq *exp-tm-ms1* (subseq *exp-tm-ms1* 0 (- (length *exp-tm-ms1*) (length append))))
              (setq *exp-rc1* (subseq *exp-rc1* 0 (- (length *exp-rc1*) (length append))))
              (setq *exp-zmp-wc1* (subseq *exp-zmp-wc1* 0 (- (length *exp-zmp-wc1*) (length append))))
              (setq *exp-optional1* (subseq *exp-optional1* 0 (- (length *exp-optional1*) (length append))))
              (setq *exp-jpos-rad1* (subseq *exp-jpos-rad1* 0 (- (length *exp-jpos-rad1*) (length append))))
              (setq *exp-tm-s1* (subseq *exp-tm-s1* 0 (- (length *exp-tm-s1*) (length append))))
              (setq *exp-pos1* (subseq *exp-pos1* 0 (- (length *exp-pos1*) (length append))))
              (setq *exp-rpy1* (subseq *exp-rpy1* 0 (- (length *exp-rpy1*) (length append))))
              (setq *exp-zmp-rc1* (subseq *exp-zmp-rc1* 0 (- (length *exp-zmp-rc1*) (length append))))
              (setq *exp-wc1* (subseq *exp-wc1* 0 (- (length *exp-wc1*) (length append))))
              (setq send-flag nil)
              )
            (unix:usleep 10)
            )

          (if real
              (send *ri* :sequenceplayerservice_setjointanglessequencefull :jvss jvss :vels vels :torques torques :poss poss :rpys rpys :accs accs :zmps zmps :wrenchs wrenchs :optionals optionals :tms tms)
            (progn
              (format t "send ~a setJointAnglesSequenceFull timer = ~a~%" t-max (send timer :stop))
              ;; (send *robot* :move-to (car (last *exp-wc1*)) :world)
              ;; (send *robot* :angle-vector (car (last *exp-jpos-deg1*)))
              ;; (send *irtviewer* :draw-objects)
              )
            )
          );; end let

        (when real
          (send *ri* :wait-interpolation-seq)
          (unix:usleep (* final-time 1000))
          (send *ri* :save-log log-fname)
          )
        (setq *dp-list* (reverse *dp-list*))
        (setq *qp-ss-list* (reverse *qp-ss-list*))
        )
    (format t "Time-list and t-max are not consistent. You should make sure the sequence and time scale!~%")
    )

  nil
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
  (when modified-p-path
    (load modified-p-path)
    (insert-modified-p *p* *modified-p*)
    )

  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
   (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                              :joint-class 6dof-joint)
   (show-optimized-motion *p* :x-step step :x-max x-max :real-time nil :make-sequence t :choreonoid choreonoid :lfoot-ik lfoot-ik :rfoot-ik rfoot-ik :zmp-offset zmp-offset)
   )
  ;; 恐らくここのshow-optimized-motionでセグフォが起きている。
  ;; epochとかディレクトリを作るのは、defun show-optyimized-motionでなくて、それを読んでいるところでやるか。

  ;; ここで使用するリストは、choreonoidがtの時用のものだから、これは実行されていないものだと思ってよいのでは？
  ;; しかも、これらの値って、*bspline-av-list*とか元のがあれば作れるくないかな？
  (when (= (length tm-ms) (length *bspline-avsf-tm-ms*))
    (format t "replace is done")
    (replace jpos-deg (copy-object *bspline-avsf-jpos-deg*))
    (replace tm-ms (copy-object *bspline-avsf-tm-ms*))
    (replace rc (copy-object *bspline-avsf-rc*))
    (replace zmp-wc (copy-object *bspline-avsf-zmp-wc*))
    (replace optional (copy-object *bspline-avsf-optional*))
    (replace jpos-rad (copy-object *bspline-avsf-jpos-rad*))
    (replace tm-s (copy-object *bspline-avsf-tm-s*))
    (replace pos (copy-object *bspline-avsf-pos*))
    (replace rpy (copy-object *bspline-avsf-rpy*))
    (replace zmp-rc (copy-object *bspline-avsf-zmp-rc*))
    (replace robot-wc (copy-object *bspline-avsf-wc*))
    (replace cog-wc (copy-object *bspline-avsf-cog-wc*))
    )
  nil
  )




;; 特定の関節を一定の関節角度にする
(defun modify-joint-constant (jlist av &key (p *p*))
  (let* (link-id
         (modified-av-list
          (mapcar #'(lambda (x)
                      (coerce (make-list *id-max* :initial-element x) float-vector)
                      )
                  (coerce av cons))
          )
         )
    (print modified-av-list)
    (mapc
     #'(lambda (jl av-list)
         (setq link-id (position jl *jlist* :test #'equal))
         (replace *p* av-list
                  :start1 (* link-id *id-max*) :end1 (* link-id (+ *id-max* 1))
                  :start2 0 :end2 *id-max*
                  )
         )
     jlist modified-av-list)
    )
  (format t ";; (setq *p-orig* (copy-seq *p*))")
  nil
  )



(defun append-last-sequence
  (centroid-pos &key (append nil) (tm 1000) (optional #f(1 1 0 0 1 1 1 1)) (choreonoid t))
  (let* ((target-coords
          (list
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll *robot* '(:lleg :rleg))
     (send *robot*
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(1.0 1.0 1.0 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos centroid-pos
           :centroid-thre 5
           :move-target mt
           :link-list ll
           :avoid-collision-distance 60
           :avoid-collision-null-gain 5.0
           :avoid-collision-joint-gain 0.8
           :additional-weight-list (list (list (send *robot* :head-neck-p :child-link) 0)
                                         (list (send *robot* :head-neck-y :child-link) 0))
           :rotation-axis (list t t)
           :debug-view nil) ;;:no-message) ;;t)
     ))
  (send *irtviewer* :draw-objects)

  (when append
    (let* ((rc (send (car (send *robot* :links)) :copy-worldcoords))
           (av (send *robot* :angle-vector)))
      (setq *exp-jpos-deg1* (append *exp-jpos-deg1* (list (concatenate float-vector av (if choreonoid #f(-80 80 -80 80) nil))) ))
      (setq *exp-tm-ms1* (append *exp-tm-ms1* (list tm) ))
      (setq *exp-rc1* (append *exp-rc1* (list rc) ))
      (setq *exp-zmp-wc1* (append *exp-zmp-wc1* (list centroid-pos) ))
      (setq *exp-optional1* (append *exp-optional1* (list optional)))
      (setq *exp-jpos-rad1* (append *exp-jpos-rad1* (list (map float-vector #'deg2rad (concatenate float-vector av (if choreonoid #f(-80 80 -80 80) nil)))) ))
      (setq *exp-tm-s1*  (append *exp-tm-s1* (list (* 1e-3 tm))))
      (setq *exp-pos1* (append *exp-pos1* (list (scale 1e-3 (send rc :worldpos))) ))
      (setq *exp-rpy1* (append *exp-rpy1* (list (reverse (nth 0 (rpy-angle (send rc :worldrot))))) ))
      (setq *exp-zmp-rc1* (append *exp-zmp-rc1* (list (scale 1e-3 (send rc :inverse-transform-vector centroid-pos))) ))
      (setq *exp-wc1* (append *exp-wc1* (list (send *robot* :copy-worldcoords)) ))
      (setq *exp-cog-wc1* (append *exp-cog-wc1* (list (send *robot* :centroid)) ))
      )
    )

  nil
  )

#|
;; (append-last-sequence (midpoint 0.35 (send *lfoot* :worldpos) (send *rfoot* :worldpos)) :tm 1000 :append nil)
(setq *append-list*
      (list
       (list (float-vector (aref (send *robot* :centroid) 0) (aref (send *robot* :centroid) 1) 0) 1000)
       ;; (list (car (last *exp-zmp-wc1*)) 1000)
       (list (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos)) 1000)
       )
      )
(dolist (als *append-list*) (append-last-sequence (nth 0 als) :tm (nth 1 als) :append nil))

;; (send *ri* :angle-vector-sequence-full *bspline-avsf-jpos-deg* *bspline-avsf-tm-ms* :root-coords *bspline-avsf-rc* :zmp *bspline-avsf-zmp-wc* :optional *bspline-avsf-optional*)

(experiment-angle-vector-sequence-full-switch *exp-jpos-deg2* *exp-tm-ms2* *exp-rc2* *exp-zmp-wc2* *exp-optional2* *exp-jpos-deg1* *exp-tm-ms1* *exp-rc1* *exp-zmp-wc1* *exp-optional1* :initial-time 10000 :log-fname "/tmp/one-switch" :switch-time 6.1)

(modify-joint-constant (send-all (send *robot* :larm :links) :joint) (send *robot* :larm :angle-vector))



(let* ((time-scale 1.0))
  (setq *p* (copy-seq *p-orig*))
  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
   (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                              :joint-class 6dof-joint)
   (experiment-online-trajectory-modification *exp-jpos-rad1* *exp-tm-s1* *exp-pos1* *exp-rpy1* *exp-zmp-rc1* *exp-optional1* (* *x-max-of-p-orig* time-scale) (* (eval *x-hit*) time-scale) (* (- (eval *x-hit*) (* 3.0 (car *exp-tm-s1*))) time-scale) :initial-time 2000 :final-time 5000 :test-target #f(0 0 -0.02 0 0 0) :real nil :debug? t :append *append-list* :lfoot-ik t :rfoot-ik t)
   (setq *p* (copy-seq *p-orig*))
   nil))

(experiment-angle-vector-sequence-full *exp-jpos-deg1* *exp-tm-ms1* *exp-rc1* *exp-zmp-wc1* *exp-optional1* :initial-time 10000 :final-time 5000 :log-fname "/tmp/angle-vector-sequence-full")

(experiment-angle-vector-sequence-full (list (car *exp-jpos-deg1*)) (list (car *exp-tm-ms1*)) (list (car *exp-rc1*)) (list (car *exp-zmp-wc1*)) (list (car *exp-optional1*)) :initial-time 2000 :final-time 0 :log-fname "/tmp/init")

(let* ((t-max *x-max-of-p-orig*) (step (/ *x-max-of-p-orig* 40.0)) (seq-num (round (+ 1 (/ t-max step)))))
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
|#

