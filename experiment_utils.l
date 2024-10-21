(defun red-init ()
  (load "package://msl_hand_controller/euslisp/jaxon_red_mslhand-interface.l")
  (load "package://auto_stabilizer/euslisp/auto-stabilizer-interface.l")
  (jaxon_red-init)
  (setq *robot* (jaxon_red))
  ;; (objects (list *robot*))
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
	     (setq av (concatenate float-vector av #f(-50 0 70 0 100 14.3239 0 0 0 0 0 0)))	     
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
	     ;; (setq av (concatenate float-vector av #f(-50 0 70 0 100 100 0 0 0 0 0 0)))
	     (setq av (concatenate float-vector av #f(-50 0 70 0 100 14.3239 0 0 0 0 0 0)))
	     )
	 tmp)
	)
  (replace jpos-deg (copy-object *new-bspline-av-list*))
  (replace tm-ms (copy-object *bspline-tm-list*))
  (replace optional (copy-object *bspline-optional-list*))
  
  nil
  )


;; 寺澤さんversion
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
  (when (= (length tm-ms) (length *bspline-avsf-tm-ms*))
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

(defun make-sequence-in-advance-without-zmp-rc
    (jpos-deg tm-ms optional
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
  ;; jpos-degに、mslの12自由度を後から追加
  (setq *bspline-avsf-jpos-deg*
	(mapcar
	 #'(lambda (av)
	     (setq av (concatenate float-vector av #f(-50 0 70 0 100 14.3239 0 0 0 0 0 0)))
	     )
	 tmp)
	)

  (when (= (length tm-ms) (length *bspline-avsf-tm-ms*))
    (replace jpos-deg (copy-object *bspline-avsf-jpos-deg*))
    (replace tm-ms (copy-object *bspline-avsf-tm-ms*))
    (replace optional (copy-object *bspline-avsf-optional*))
    ) 
  nil
  )


(defun show-optimized-motion
  (p-ret
   &key
   (robot *robot*)
   (cnt 30)
   (x-max *x-max*)
   (make-loadpattern nil) ;; "/tmp/motion"
   (make-sequence nil)
   (choreonoid nil)
   (specific-time nil)
   (x-step (if (or make-loadpattern specific-time) 0.002 (/ (- x-max *x-min*) (* 1.0 cnt))))
   (real-time t)
   (warning t)
   (show-pose t)
   ;; (modify-zmp nil) ;; (send (midcoords 0.5 *rfoot* *lfoot*) :worldpos)
   (lfoot-ik nil)
   (rfoot-ik nil)
   (zmp-offset #f(0 0 0))
   (fix-leg-coords (list :lleg *lfoot*))
   )

  (when (not specific-time)
    (setq *bspline-av-list* '()) ;; for load-pattern .pos, angle-vector-sequence-full jpos and showing jvel or jvel-ratio
    (setq *bspline-tm-list* '()) ;; for load-pattern time-stamp, angle-vector-sequence-full tm and showing jvel or jvel-ratio
    (setq *bspline-zmp-rc-list* '()) ;; for load-pattern .zmp
    (setq *bspline-zmp-wc-list* '()) ;; for angle-vector-sequence-full :zmp
    (setq *bspline-cog-wc-list* '()) ;; for angle-vector-sequence-full :zmp
    (setq *bspline-rc-list* '()) ;; for load-pattern .waist and angle-vector-sequence-full :root-coords
    (setq *bspline-wc-list* '())
    (setq *bspline-optional-list* '()) ;; for load-pattern .optionaldata and angle-vector-sequence-full :optional
    (setq *bspline-ec-list* '())
    (setq *bspline-tau-list* '())
    (setq *bspline-hp-list* '())
    )

  (let* ((x-buf (if specific-time specific-time *x-min*))
         (retl (list (cons :gain p-ret)))  ;; (cons :gain p-ret) ==> (:gain . p-ret)
         p dp ddp id tau
         (show-loop 0)
         (bspline
          (if specific-time
              *bspline* ;; 特定の時刻を表示するときは今の*bspline*を用いる（現状はnloptの計算速度を考慮してこうする）
            (progn ;; ひと通り表示するときはx-maxを変えて表示ができるようにここで新たにbsplineのinstanceを生成
              (if (send (car (send robot :links)) :joint) ;; with-append-root-jointしてあればt
                  (progn
                    (mapcar #'(lambda (k)
                                (instance basic-spline :init
                                          :id-max *id-max* :recursive-order *recursive-order*
                                          :x-min *x-min* :x-max x-max))
                            (flatten *jlist* (make-list 6)))) ;; root-jointの6個分付け加える
                (progn
                  (mapcar #'(lambda (k)
                              (instance basic-spline :init
                                        :id-max *id-max* :recursive-order *recursive-order*
                                        :x-min *x-min* :x-max x-max))
                          *jlist*))
                )
              )
            )
          )
         (contact-tm-cnt 0)
         (switching-tm-list (list x-max))
         (tm (instance mtimer :init)) ;; 実際の時間で表示するためのタイマー
         )
    (dotimes (i (1- (length *contact-flag*)))
      (push (* (aref p-ret (- (length p-ret) 1 i)) (/ x-max *x-max*)) switching-tm-list)
      )

    ;; (send *irtviewer* :logging :clear)
    ;; (send *irtviewer* :logging :start)
    
    (while (<= x-buf x-max)
      (setq id 0)
      (mapc
       #'(lambda (jl bs)
           (list
            (send jl :joint-angle
                  (send bs :calc x-buf (subseq p-ret id (+ id *id-max*))))
            (send jl :put :p (send bs :calc x-buf (subseq p-ret id (+ id *id-max*))))
            (send jl :put :dp (send bs :calc-delta x-buf (subseq p-ret id (+ id *id-max*)) :n 1))
            (send jl :put :ddp (send bs :calc-delta x-buf (subseq p-ret id (setq id (+ id *id-max*))) :n 2))
            ))
       *jlist* bspline)

      (push (send-all *jlist* :get :ddp) ddp)
      (push (send-all *jlist* :get :dp) dp)
      (push (send-all *jlist* :get :p) p)

      (send robot :calc-torque-from-vel-acc
            :jvv (map float-vector
                      #'(lambda (j) (deg2rad (or (send j :get :dp) 0)))
                      (cdr (send robot :links)))
            :jav (map float-vector
                      #'(lambda (j) (deg2rad (or (send j :get :ddp) 0)))
                      (cdr (send robot :links))))
      (push (coerce (send-all *jlist* :joint-torque) float-vector) *bspline-tau-list*)
      (push (send-all *jlist* :joint-torque) tau)

      (let* (each-6dof current-zmp-tf current-zmp-with-offset)
        (if (send (car (send robot :links)) :joint)
            (progn
              (send (send (car (send robot :links)) :joint) :joint-angle
                    (coerce
                     (mapcar
                      #'(lambda (bs)
                          (setq each-6dof (send bs :calc x-buf (subseq p-ret id (+ id *id-max*))))
                          ;; (send bs :calc-delta x-buf (subseq p-ret id (+ id *id-max*)) :n 1)
                          ;; (send bs :calc-delta x-buf (subseq p-ret id (+ id *id-max*)) :n 2)
                          (setq id (+ id *id-max*))
                          each-6dof)
                      (subseq bspline (length *jlist*) (+ (length *jlist*) 6)))
                     float-vector)
                    )
              )
          (progn
            (send robot :fix-leg-to-coords (nth 1 fix-leg-coords) (nth 0 fix-leg-coords))
            )
          )


        ;; 支持多角形の表示
        (calc-specific-time-force-zmp p-ret x-buf :bspline bspline :zmp-at-specific-time *current-zmp* :force-at-specific-time *current-force* :zmp-moment-at-specific-time *current-zmp-moment* :contact-flag (nth contact-tm-cnt *contact-flag*) :lfoot-ik lfoot-ik :rfoot-ik rfoot-ik :step-time x-step)
        (send *support-face* :draw-on :flush t :color #f(0.5 0.5 0) :width 2.5)

        (setq current-zmp-with-offset (v+ *current-zmp* zmp-offset))
        (send *arrow* :locate current-zmp-with-offset :world) ;; zmpを表示

        (when (and (eq (send *support-face* :insidep current-zmp-with-offset) :outside) warning)
          (format t "ZMP is on the outside of support polygon at ~a!!~%" x-buf)
          )

	;; (dump-list)
        (when specific-time
          (when show-pose
            (send *irtviewer* :draw-objects)
            (x::window-main-one)
            )
	  
          (return-from nil 1)
          )
        (push current-zmp-with-offset *bspline-zmp-wc-list*)
        (setq current-zmp-tf (send (send (car (send robot :links)) :copy-worldcoords) :inverse-transform-vector current-zmp-with-offset)) ;; root-coords相対の座標変換
        (push current-zmp-tf  *bspline-zmp-rc-list*)
        )

      ;; (dump-list)

      (when (and lfoot-ik (nth 0 (nth contact-tm-cnt *contact-flag*)))
        (let* ((new-lfoot (copy-object (nth 0 (nth contact-tm-cnt *contact-flag*)))))
          (when (vectorp lfoot-ik)
            (send new-lfoot :translate  (subseq lfoot-ik 0 3) :world)
            (when (= (length lfoot-ik) 4)
              (send new-lfoot :rotate (deg2rad (aref lfoot-ik 3)) :z)
              )
            )
          (send robot :lleg :inverse-kinematics new-lfoot :rotation-axis t)
          )
        )
      (when (and rfoot-ik (nth 1 (nth contact-tm-cnt *contact-flag*)))
        (let* ((new-rfoot (copy-object (nth 1 (nth contact-tm-cnt *contact-flag*)))))
          (when (vectorp rfoot-ik)
            (send new-rfoot :translate  (subseq rfoot-ik 0 3) :world)
            (when (= (length rfoot-ik) 4)
              (send new-rfoot :rotate (deg2rad (aref rfoot-ik 3)) :z)
              )
            )
          (send robot :rleg :inverse-kinematics new-rfoot :rotation-axis t)
          )
        )

      (push (send robot :centroid) *bspline-cog-wc-list*)

      ;;(send *robot* :fix-leg-to-coords *lfoot* :lleg)
      (push (send robot :angle-vector) *bspline-av-list*)
      (push x-buf *bspline-tm-list*)
      (push (send (car (send robot :links)) :copy-worldcoords) *bspline-rc-list*)
      (push (send robot :copy-worldcoords) *bspline-wc-list*)
      (let ((right-remain-tm (- x-max x-buf))
            (left-remain-tm (- x-max x-buf))
            (max-contact-tm-cnt (- (length *contact-flag*) 1))
            )
        (dotimes (i (- max-contact-tm-cnt contact-tm-cnt))
          (when (or (and (not (nth 1 (nth contact-tm-cnt *contact-flag*)))
                         (nth 1 (nth (min (+ contact-tm-cnt i) max-contact-tm-cnt) *contact-flag*)))
                    (and (nth 1 (nth contact-tm-cnt *contact-flag*))
                         (not (nth 1 (nth (min (+ contact-tm-cnt i) max-contact-tm-cnt) *contact-flag*))))
                    ) ;; xor (exclusive-or)
            (setq right-remain-tm (- (nth (+ contact-tm-cnt i -1) switching-tm-list) x-buf)) ;; のこぎり (iは必ず1以上)
            (return nil)
            )
          )
        (dotimes (i (- max-contact-tm-cnt contact-tm-cnt))
          (when (or (and (not (nth 0 (nth contact-tm-cnt *contact-flag*)))
                         (nth 0 (nth (min (+ contact-tm-cnt i) max-contact-tm-cnt) *contact-flag*)))
                    (and (nth 0 (nth contact-tm-cnt *contact-flag*))
                         (not (nth 0 (nth (min (+ contact-tm-cnt i) max-contact-tm-cnt) *contact-flag*))))
                    ) ;; xor (exclusive-or)
            (setq left-remain-tm (- (nth (+ contact-tm-cnt i -1) switching-tm-list) x-buf)) ;; のこぎり (iは必ず1以上)
            (return nil)
            )
          )
        (push (float-vector
               (if (nth 1 (nth contact-tm-cnt *contact-flag*)) 1 0)
               (if (nth 0 (nth contact-tm-cnt *contact-flag*)) 1 0)
               0 0
               right-remain-tm
               left-remain-tm
               1 1
               )
              *bspline-optional-list*)
        )
      (push (send (send robot *limb* :end-coords) :copy-worldcoords) *bspline-ec-list*)
      (push (send *hit-point* :copy-worldcoords) *bspline-hp-list*)
      (send *irtviewer* :draw-objects)
      (x::window-main-one)

      (if real-time
          (while (< (send tm :stop) (* x-step show-loop)))
        (unix:usleep 1)
        )
      (setq x-buf (+ x-buf x-step))

      (when (> x-buf (nth contact-tm-cnt switching-tm-list))
        (incf contact-tm-cnt)
        )
      (incf show-loop)
      ) ;; end while loop
    
    ;; (send *irtviewer* :logging :stop)

    
    ;; ()
    ;; (dump-loadable-structure "tst.l" *bspline-av-list*
    ;;  )
    ;; 保存したbspline-av-listがlenght25でよくわからんな。


    (when (not specific-time)
      (push (cons :p (reverse p)) retl)
      (push (cons :dp (reverse dp)) retl)
      (push (cons :ddp (reverse ddp)) retl)
      (push (cons :tau (reverse tau)) retl)
      (setq *bspline-av-list* (reverse *bspline-av-list*))
      (setq *bspline-tm-list* (reverse *bspline-tm-list*))
      (setq *bspline-zmp-rc-list* (reverse *bspline-zmp-rc-list*))
      (setq *bspline-zmp-wc-list* (reverse *bspline-zmp-wc-list*))
      (setq *bspline-cog-wc-list* (reverse *bspline-cog-wc-list*))
      (setq *bspline-rc-list* (reverse *bspline-rc-list*))
      (setq *bspline-wc-list* (reverse *bspline-wc-list*))
      (setq *bspline-tau-list* (reverse *bspline-tau-list*))
      (progn
        (setf (aref (car *bspline-optional-list*) 4) 1)
        (setf (aref (car *bspline-optional-list*) 5) 1) ;; 最後のoptionaldataの時間だけベタ書きで1にしておく
        (setq *bspline-optional-list* (reverse *bspline-optional-list*))
        )
      (setq *bspline-ec-list* (reverse *bspline-ec-list*))
      (setq *bspline-hp-list* (reverse *bspline-hp-list*))

      (when make-loadpattern
        (file-init (concatenate string make-loadpattern ".pos"))
        (file-init (concatenate string make-loadpattern ".zmp"))
        (file-init (concatenate string make-loadpattern ".waist"))
        (file-init (concatenate string make-loadpattern ".optionaldata"))

        ;; write load-pattern file
        (let* ((i 0) bspline-pos bspline-zmp tmp-rc bspline-waist bspline-optional)
          (dolist (tm *bspline-tm-list*)
            (setq bspline-pos (map float-vector #'deg2rad (nth i *bspline-av-list*)))
            (setq bspline-zmp (scale 1e-3 (nth i *bspline-zmp-rc-list*)))
            (setq tmp-rc (nth i *bspline-rc-list*))
            (setq bspline-waist (concatenate float-vector
                                             (scale 1e-3 (send tmp-rc :worldpos)) ;; x y z [m]
                                             (nth 0 (rpy-angle (send tmp-rc :worldrot)))) ;; yaw pitch roll [rad]
                  )
            (setq bspline-optional (nth i *bspline-optional-list*))
            (with-open-file
             (f (concatenate string make-loadpattern ".pos") :direction :output :if-exists :append)
             (format f "~a" tm)
             (dotimes (j (length bspline-pos))
               (format f " ~a" (aref bspline-pos j))
               )
             (if choreonoid
                 (format f " -1.39626 1.39626 -1.39626 1.39626~%")
               (format f "~%")
               )
             )
            (with-open-file
             (f (concatenate string make-loadpattern ".zmp") :direction :output :if-exists :append)
             (format f "~a ~a ~a ~a~%" tm (aref bspline-zmp 0) (aref bspline-zmp 1) (aref bspline-zmp 2)) ;; ２個目のzmpの位置は上で補正している
             )
            (with-open-file
             (f (concatenate string make-loadpattern ".waist") :direction :output :if-exists :append)
             (format f "~a ~a ~a ~a ~a ~a ~a~%" tm (aref bspline-waist 0) (aref bspline-waist 1) (aref bspline-waist 2) (aref bspline-waist 5) (aref bspline-waist 4) (aref bspline-waist 3)) ;; x y z roll pitch yaw
             )
            (with-open-file
             (f (concatenate string make-loadpattern ".optionaldata") :direction :output :if-exists :append)
             (format f "~a ~a ~a ~a ~a ~a ~a ~a ~a~%" tm (aref bspline-optional 0) (aref bspline-optional 1) (aref bspline-optional 2) (aref bspline-optional 3) (aref bspline-optional 4) (aref bspline-optional 5) (aref bspline-optional 6) (aref bspline-optional 7))
             )
            (incf i)
            ) ;; end dolist
          )
        )

      (when make-sequence
        ;; for :angle-vector-sequence-full
        ;; jpos
        (if choreonoid
            (setq *bspline-avsf-jpos-deg*
                  (mapcar
                   #'(lambda (av)
                       (setq av (concatenate float-vector av #f(-80 80 -80 80))) ;; add head and hand for choreonoid
                       )
                   *bspline-av-list*))
          (setq *bspline-avsf-jpos-deg* *bspline-av-list*)
          ) ;; [deg]
        (setq *bspline-avsf-jpos-rad*
              (mapcar
               #'(lambda (av)
                   (map float-vector #'deg2rad av)
                   )
               *bspline-avsf-jpos-deg*)) ;; [rad]

        ;; tm
        (setq *bspline-avsf-tm-ms* (make-list (length *bspline-tm-list*) :initial-element (* 1e3 x-step))) ;; [ms]
        (setq *bspline-avsf-tm-s* (make-list (length *bspline-tm-list*) :initial-element x-step)) ;; [s]

        ;; pos
        (setq *bspline-avsf-pos*
              (mapcar
               #'(lambda (rc)
                   (scale 1e-3 (send rc :worldpos)) ;; x y z [m]
                   )
               *bspline-rc-list*))
        ;; rpy
        (setq *bspline-avsf-rpy*
              (mapcar
               #'(lambda (rc)
                   (reverse (nth 0 (rpy-angle (send rc :worldrot)))) ;; roll pitch yaw [rad]
                   )
               *bspline-rc-list*))
        ;; root coords
        (setq *bspline-avsf-rc* *bspline-rc-list*)

        ;; robot world coords
        (setq *bspline-avsf-wc* *bspline-wc-list*)

        ;; zmp
        (setq *bspline-avsf-zmp-rc*
              (mapcar
               #'(lambda (zmp)
                   (scale 1e-3 zmp) ;; x y z [m]
                   )
               *bspline-zmp-rc-list*)) ;; [m] relative to root coords
        (setq *bspline-avsf-zmp-wc* *bspline-zmp-wc-list*) ;; [mm] relative to world coords

        ;; optional
        (setq *bspline-avsf-optional* *bspline-optional-list*)

        ;; centroid position
        (setq *bspline-avsf-cog-wc* *bspline-cog-wc-list*)
        )
      )

    nil
    )
  )
