(require "package://baseball/euslisp/motion-generation-utils.l")

(objects (list *robot* *racket-grip* *racket-model* *sweet-spot*))

;; make coords of robot foots
(setq *lfoot-step* (make-coords :pos #f(-300 0 0) :rpy (list (deg2rad 30) 0 0)))
(setq *lfoot* (make-coords :pos #f(-100 380 0) :rpy (list (deg2rad 45) 0 0)))
(setq *rfoot* (make-coords :pos #f(0 -380 0)))


;; forehand swing http://www.ikeyon.com/jtc/image/stforhandonecrt.gif
(defun pose1 (&key (robot *robot*) (base-time 5000) (real nil))
  (send robot :angle-vector
        #f(
           5.35899 -18.7187 -43.3007 82.5595 -37.7075 17.3467 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
           48.7875 28.4699 -45.369 58.7039 -11.0443 -28.4245  ;; lleg_joint0~5
           3.0813 13.7412 -16.5634 ;; chest_joint0~2 (<torso> waist-r,p,y)
           22.4 0.0 ;; head_joint0~1 (<head> neck-y,p)
           -0.328496 -59.2828 -50.1455 -0.49446 -90 -69.5632 -19.1267 46.2037 ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           0 0 60 0 -60 45 0 0 ;; larm_joint0~6
           ;; 8 0 80 0 -40 45 0 0 ;; before Humanoids2016
           ))

        ;; #f(
        ;;    3.20807 -20.7196 -24.2303 55.1402 -33.1479 20.7332 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
        ;;    4.14109 22.8213 -22.1177 52.6859 -30.9337 -22.8213  ;; lleg_joint0~5
        ;;    0 20 0 ;; chest_joint0~2 (<torso> waist-r,p,y)
        ;;    45 0 ;; head_joint0~1 (<head> neck-y,p)
        ;;    0 -61.2996 -22.099 3.48124 -136.099 -66.0525 -1.02826 78.5083 ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
        ;;    8 0 80 0 -40 45 0 0;; larm_joint0~6
        ;;    ))

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad -6) :z :world)
  (send robot :locate (float-vector 0 -10 -130) :world)

  (send *racket-coords* :orient (deg2rad -80) :z :world)
  (send *racket-coords* :locate (float-vector 0 0 0) :world)
  (send *racket-coords* :rotate (deg2rad -5) :x :world)
  (send *racket-coords* :rotate (deg2rad -10) :y :world)
  (send *racket-coords* :locate (float-vector 300 -650 1500) :world)

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *racket-grip*
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.6 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 20
           :move-target mt
           :link-list ll
           :avoid-collision-distance 60
           :avoid-collision-null-gain 5.0
           :avoid-collision-joint-gain 0.8
           :collision-avoidance-link-pair
           (list
            (list (cadr (send robot :torso :links))
                  (send robot :larm_link2_lk))
            (list (cadr (send robot :torso :links))
                  (send robot :rarm_link2_lk))
            )
           :additional-weight-list (list (list (send robot :head-neck-p :child-link) 0)
                                         (list (send robot :head-neck-y :child-link) 0))
           :rotation-axis (list t t t)
           :debug-view nil) ;;:no-message) ;;t)
     ))

  (send *irtviewer* :draw-objects)

  (if real
      (progn
        (send *ri* :angle-vector (send robot :angle-vector) base-time)
        (send *ri* :wait-interpolation)
        )
    )

  (list :angle-vector (send robot :angle-vector)
        :root-coords (send (car (send robot :links)) :copy-worldcoords) ;; (send robot :copy-worldcoords)
        :worldcoords (send robot :copy-worldcoords)
        :time base-time)
  )




(defun pose2 (&key (robot *robot*) (base-time 5000) (real nil))
  (send robot :angle-vector
        ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
        ;; lleg_joint0~5
        ;; chest_joint0~2 (<torso> waist-r,p,y) head_joint0~1 (<head> neck-y,p)
        ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
        ;; larm_joint0~6
        ;; angle-vector of pose1
        #f(5.96116 -17.3268 -41.4895 76.9899 -34.6405 16.8142 50.2142 27.6913 -43.417 53.2275 -8.74364 -27.4784 4.80211 14.6122 -10.6716 22.4 0.0 -3.70715 -48.6773 -48.9095 2.77695 -118.559 -30.0848 -31.8118 1.9412 8.0 0.0 80.0 0.0 -40.0 45.0 0.0 0.0))

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad 2) :z :world)
  (send robot :locate (float-vector 0 -10 -260) :world)

  (send *racket-coords* :orient (deg2rad -80) :z :world)
  (send *racket-coords* :locate (float-vector 0 0 0) :world)
  (send *racket-coords* :rotate (deg2rad 100) :x :world)
  (send *racket-coords* :rotate (deg2rad -10) :z :world)
  (send *racket-coords* :locate (float-vector 500 -700 1000) :world)

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *racket-grip*
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.55 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 20
           :move-target mt
           :link-list ll
           :avoid-collision-distance 60
           :avoid-collision-null-gain 5.0
           :avoid-collision-joint-gain 0.8
           :collision-avoidance-link-pair
           (list
            (list (cadr (send robot :torso :links))
                  (send robot :larm_link2_lk))
            (list (cadr (send robot :torso :links))
                  (send robot :rarm_link2_lk))
            )
           :additional-weight-list (list (list (send robot :head-neck-p :child-link) 0)
                                         (list (send robot :head-neck-y :child-link) 0))
           :rotation-axis (list t t t)
           :debug-view nil) ;;:no-message) ;;t)
     ))

  (send *irtviewer* :draw-objects)

  (if real
      (progn
        (send *ri* :angle-vector (send robot :angle-vector) base-time)
        (send *ri* :wait-interpolation)
        )
    )

  (list :angle-vector (send robot :angle-vector)
        :root-coords (send (car (send robot :links)) :copy-worldcoords) ;; (send robot :copy-worldcoords)
        :worldcoords (send robot :copy-worldcoords)
        :time base-time)
  )



(defun pose3 (&key (robot *robot*) (base-time 5000) (real nil))
  (send robot :angle-vector
        ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
        ;; lleg_joint0~5
        ;; chest_joint0~2 (<torso> waist-r,p,y) head_joint0~1 (<head> neck-y,p)
        ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
        ;; larm_joint0~6
        ;; angle-vector of pose2
        #f(-0.851399 -25.3628 -43.6886 81.0988 -37.0053 23.9976 43.4412 23.4412 -52.1147 73.8623 -20.4123 -24.1527
           4.69939 17.9085 19.7456 22.4 0.0
           13.9379 -1.29912 -50.4969 -82.3214 -80 -71.956 -85.2512 16.7357
           5.0 20.0 50.0 0.0 -100.0 45.0 0.0 0.0))
        ;; #f(-0.851399 -25.3628 -43.6886 81.0988 -37.0053 23.9976 43.4412 23.4412 -52.1147 73.8623 -20.4123 -24.1527 4.69939 17.9085 19.7456 22.4 0.0 13.9379 -1.29912 -50.4969 -82.3214 -108.234 -71.956 -85.2512 16.7357 8.0 0.0 80.0 0.0 -40.0 45.0 0.0 0.0)) ;; before Humanoids2016

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad 10) :z :world)
  (send robot :locate (float-vector 0 -10 -240) :world)

  (send *racket-coords* :orient (deg2rad 0) :z :world)
  (send *racket-coords* :locate (float-vector 0 0 0) :world)
  (send *racket-coords* :rotate (deg2rad 110) :y :world)
  (send *racket-coords* :rotate (deg2rad 15) :x :world)
  (send *racket-coords* :locate (float-vector 650 300 780) :world)
  ;; (send *racket-coords* :locate (float-vector 550 300 805) :world) ;; before Humanoids2016

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *racket-grip*
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 20
           :move-target mt
           :link-list ll
           :avoid-collision-distance 60
           :avoid-collision-null-gain 5.0
           :avoid-collision-joint-gain 0.8
           :collision-avoidance-link-pair
           (list
            (list (cadr (send robot :torso :links))
                  (send robot :larm_link2_lk))
            (list (cadr (send robot :torso :links))
                  (send robot :rarm_link2_lk))
            )
           :additional-weight-list (list (list (send robot :head-neck-p :child-link) 0)
                                         (list (send robot :head-neck-y :child-link) 0))
           :rotation-axis (list t t t)
           :debug-view nil) ;;:no-message) ;;t)
     ))

  (send *irtviewer* :draw-objects)

  (if real
      (progn
        (send *ri* :angle-vector (send robot :angle-vector) base-time)
        (send *ri* :wait-interpolation)
        )
    )

  (list :angle-vector (send robot :angle-vector)
        :root-coords (send (car (send robot :links)) :copy-worldcoords) ;; (send robot :copy-worldcoords)
        :worldcoords (send robot :copy-worldcoords)
        :time base-time)
  )




(defun pose4 (&key (robot *robot*) (base-time 5000) (real nil))
  (send robot :angle-vector
        ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
        ;; lleg_joint0~5
        ;; chest_joint0~2 (<torso> waist-r,p,y) head_joint0~1 (<head> neck-y,p)
        ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
        ;; larm_joint0~6
        ;; angle-vector of pose3
        #f(-9.13176 -24.5612 -39.3477 78.5497 -37.9731 24.8068 35.0836 24.0268 -53.3272 75.0331 -21.027 -23.0677 4.6512 14.1216 23.3492 22.4 0.0 17.0316 -30.8319 -17.8012 17.7639 -78.7638 -98.9432 -43.7087 -4.6911 5.0 20.0 50.0 0.0 -100.0 45.0 0.0 0.0))

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad 18) :z :world)
  (send robot :locate (float-vector 0 -10 -100) :world)

  (send *racket-coords* :orient (deg2rad 70) :z :world)
  (send *racket-coords* :locate (float-vector 0 0 0) :world)
  (send *racket-coords* :rotate (deg2rad -50) :x :world)
  (send *racket-coords* :rotate (deg2rad 60) :z :world)
  (send *racket-coords* :locate (float-vector -200 600 1350) :world)
  ;; (send *racket-coords* :rotate (deg2rad 30) :z :world) ;; before Humanoids2016
  ;; (send *racket-coords* :locate (float-vector 40 760 1200) :world) ;; before Humanoids2016

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *racket-grip*
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.45 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 20
           :move-target mt
           :link-list ll
           :avoid-collision-distance 60
           :avoid-collision-null-gain 5.0
           :avoid-collision-joint-gain 0.8
           :collision-avoidance-link-pair
           (list
            (list (cadr (send robot :torso :links))
                  (send robot :larm_link2_lk))
            (list (cadr (send robot :torso :links))
                  (send robot :rarm_link2_lk))
            )
           :additional-weight-list (list (list (send robot :head-neck-p :child-link) 0)
                                         (list (send robot :head-neck-y :child-link) 0))
           :rotation-axis (list t t t)
           :debug-view nil) ;;:no-message) ;;t)
     ))

  (send *irtviewer* :draw-objects)

  (if real
      (progn
        (send *ri* :angle-vector (send robot :angle-vector) base-time)
        (send *ri* :wait-interpolation)
        )
    )

  (list :angle-vector (send robot :angle-vector)
        :root-coords (send (car (send robot :links)) :copy-worldcoords) ;; (send robot :copy-worldcoords)
        :worldcoords (send robot :copy-worldcoords)
        :time base-time)
  )


(defun pose5 (&key (robot *robot*) (base-time 5000) (real nil))
  (send robot :angle-vector
        ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
        ;; lleg_joint0~5
        ;; chest_joint0~2 (<torso> waist-r,p,y) head_joint0~1 (<head> neck-y,p)
        ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
        ;; larm_joint0~6
        #f(0.0 0.0 -20.0 60.0 -20.0 0.0
           0.0 0.0 -20.0 60.0 -20.0 0.0
           0 15 0 20 0
           30.0 -30.0 -48.4355 5.0 -80.0 -30.0 0.0 0.0 -30.0 -30.0 40.0 -5.0 -80.0 0.0 0.0 0.0)
        )

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad 40) :z :world)
  (send robot :locate (float-vector -200 -200 -50) :world)

  ;; (send *racket-coords* :orient (deg2rad 70) :z :world)
  ;; (send *racket-coords* :locate (float-vector 0 0 0) :world)
  ;; (send *racket-coords* :rotate (deg2rad -50) :x :world)
  ;; (send *racket-coords* :rotate (deg2rad 60) :z :world)
  ;; (send *racket-coords* :locate (float-vector -200 600 1350) :world)
  ;; ;; (send *racket-coords* :rotate (deg2rad 30) :z :world) ;; before Humanoids2016
  ;; ;; (send *racket-coords* :locate (float-vector 40 760 1200) :world) ;; before Humanoids2016

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           #|*racket-grip*|#
           *lfoot-step*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(#|:rarm|# :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.5 (send *lfoot-step* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 20
           :move-target mt
           :link-list ll
           :avoid-collision-distance 60
           :avoid-collision-null-gain 5.0
           :avoid-collision-joint-gain 0.8
           ;; :collision-avoidance-link-pair
           ;; (list
           ;;  (list (cadr (send robot :torso :links))
           ;;        (send robot :larm_link2_lk))
           ;;  (list (cadr (send robot :torso :links))
           ;;        (send robot :rarm_link2_lk))
           ;;  )
           :additional-weight-list (list (list (send robot :head-neck-p :child-link) 0)
                                         (list (send robot :head-neck-y :child-link) 0))
           :rotation-axis (list #|t|# t t)
           :debug-view nil) ;;:no-message) ;;t)
     ))

  (send *racket-coords* :dissoc *racket-grip*)
  (send *racket-grip* :assoc *racket-coords*)
  (send *racket-grip* :move-to (send robot :rarm :end-coords :copy-worldcoords) :world)
  (send *racket-grip* :dissoc *racket-coords*)
  (send *racket-coords* :assoc *racket-grip*)

  (send *irtviewer* :draw-objects)

  (if real
      (progn
        (send *ri* :angle-vector (send robot :angle-vector) base-time)
        (send *ri* :wait-interpolation)
        )
    )

  (list :angle-vector (send robot :angle-vector)
        :root-coords (send (car (send robot :links)) :copy-worldcoords) ;; (send robot :copy-worldcoords)
        :worldcoords (send robot :copy-worldcoords)
        :time base-time)
  )



(defun load-forehand-pose (&key (pose 0))
  (send (send *robot* :rarm :end-coords) :dissoc *racket-coords*)
  (let* ((robot-worldcoords nil))
    (cond ((eq pose 1) (pose1) (push (send *robot* :copy-worldcoords) robot-worldcoords))
          ((eq pose 2) (pose2) (push (send *robot* :copy-worldcoords) robot-worldcoords))
          ((eq pose 3) (pose3) (push (send *robot* :copy-worldcoords) robot-worldcoords))
          ((eq pose 4) (pose4) (push (send *robot* :copy-worldcoords) robot-worldcoords))
          ((eq pose 5) (pose5) (push (send *robot* :copy-worldcoords) robot-worldcoords))
          (t
           (setq *lp1* (pose1)) (push (send *robot* :copy-worldcoords) robot-worldcoords)
           (setq *lp2* (pose2)) (push (send *robot* :copy-worldcoords) robot-worldcoords)
           (setq *lp3* (pose3)) (push (send *robot* :copy-worldcoords) robot-worldcoords)
           (setq *lp4* (pose4)) (push (send *robot* :copy-worldcoords) robot-worldcoords)
           (setq *lp5* (pose5)) (push (send *robot* :copy-worldcoords) robot-worldcoords)
           (setq *lp-list* (list *lp1* *lp2* *lp3* *lp4* *lp5*))
           (setq robot-worldcoords (reverse robot-worldcoords))
           )
          )
    (send (send *robot* :rarm :end-coords) :assoc *racket-coords*)
    (send *robot* :copy-worldcoords)
    robot-worldcoords
    )
  )
