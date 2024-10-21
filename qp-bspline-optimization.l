(require "package://baseball/euslisp/motion-generation-utils.l")
(require "package://baseball/euslisp/bspline.so")
(require "package://eus_qpoases/euslisp/eus-qpoases.l")

(defun qp-init (&key (robot *robot*) (motion-choice *motion-choice*))
  ;; set pose parameter according to the designated motion
  (cond
   ((equal motion-choice "forehand")
    (load "package://baseball/euslisp/jaxon-key-pose/jaxon-forehand-motion.l")
    (setq *limb* :rarm)
    (setq *load-pose-func* 'load-forehand-pose)
    (setq *key-pose-id* (list 1 3 4))
    )
   ((equal motion-choice "punch")
    (load "package://baseball/euslisp/jaxon-key-pose/jaxon-punch-motion.l")
    (setq *limb* :rarm)
    (setq *load-pose-func* 'load-punch-pose)
    (setq *key-pose-id* (list 1 3 2))
    )
   ((equal motion-choice "kick")
    (load "package://baseball/euslisp/jaxon-key-pose/jaxon-kick-motion.l")
    (setq *limb* :rleg)
    (setq *load-pose-func* 'load-kick-pose)
    (setq *key-pose-id* (list 2 3 4))
    )
   ((equal motion-choice "smash")
    (load "package://baseball/euslisp/jaxon-key-pose/jaxon-smash-motion.l")
    (setq *limb* :rarm)
    (setq *key-pose-id* (list 1 3 4))
    (setq *load-pose-func* 'load-smash-pose)
    )
   ((equal motion-choice "batting")
    (load "package://baseball/euslisp/jaxon-key-pose/jaxon-batting-motion.l")
    (setq *limb* :larm)
    (setq *load-pose-func* 'load-batting-pose)
    (setq *key-pose-id* (list 1 3 4))
    )
   ((equal motion-choice "forehand-step")
    (load "package://baseball/euslisp/jaxon-key-pose/jaxon-forehand-motion.l")
    (setq *limb* :rarm)
    (setq *load-pose-func* 'load-forehand-pose)
    (setq *key-pose-id* (list 5 3 4))
    )
   ((equal motion-choice "kick-step")
    (load "package://baseball/euslisp/jaxon-key-pose/jaxon-kick-motion.l")
    (setq *limb* :rleg)
    (setq *load-pose-func* 'load-kick-pose)
    (setq *key-pose-id* (list 1 3 5))
    )
   ((equal motion-choice "batting-step")
    (load "package://baseball/euslisp/jaxon-key-pose/jaxon-batting-motion.l")
    (setq *limb* :larm)
    (setq *load-pose-func* 'load-batting-pose)
    (setq *key-pose-id* (list 5 3 4))
    )
   )

  (funcall *load-pose-func*)
  (setq *limb-av-pose-list*
        (mapcar #'(lambda (lp)
                    (send robot :angle-vector (cadr (memq :angle-vector lp)))
                    (send robot *limb* :angle-vector)
                    )
                *lp-list*)
        )
  (push nil *limb-av-pose-list*)
  (send robot :reset-pose)
  (send robot :move-to (make-coords) :world)
  (setq *limb-end-coords-default* (send (send robot *limb* :end-coords) :copy-worldcoords))
  (funcall *load-pose-func* :pose (nth 0 *key-pose-id*)) ;; init-pose
  (send *irtviewer* :draw-objects)
  ;;(format t ";;(qp-motion-optimize :x-max 2.6 :x-hit 1.7 :maxvel-weight 1e3 :minjerk-weight 1e0 :x-step 0.02)~%")
  )

;; optimization by qp
;; if use-all-joint t --> optimize motion using all joints
;; if use-all-joint nil --> optimize motion using rarm joints
(defun qp-motion-optimize
  (&key
   (use-all-joint nil)
   (use-margin nil)
   (robot
    (cond
     ((and (boundp '*robot*) *robot*) *robot*) ;; *robot*があってnilではなかったら*robot*を使う
     (t
      (require "package://jskeus/irteus/demo/sample-robot-model.l") ;; そうでなかったらsample-robotを使う
      (setq *robot* (instance sample-robot :init))
      (objects (list *robot*))
      *robot*)))

   (init (progn
           (require "package://eus_qp/euslisp/eiquadprog.l")
           (qp-init :motion-choice *motion-choice*)
           (send robot :reset-pose)
           (if use-all-joint
               (funcall *load-pose-func* :pose (nth 0 *key-pose-id*))
             (send robot *limb* :angle-vector (nth (nth 0 *key-pose-id*) *limb-av-pose-list*))
             )
           )) ;; ここでstart-avの姿勢を作る

   (jlist (if use-all-joint
              (send robot :joint-list)
            (send robot *limb* :joint-list))
            )

   (start-av (send-all jlist :joint-angle)) ;; start-av = (coerce *limb-av-pose1* cons)

   (hit-av (if use-all-joint
               (progn (funcall *load-pose-func* :pose (nth 1 *key-pose-id*)) (coerce (send robot :angle-vector) cons))
             (coerce (nth (nth 1 *key-pose-id*) *limb-av-pose-list*) cons))
           )

   (end-av
    (if use-all-joint
        (progn (funcall *load-pose-func* :pose (nth 2 *key-pose-id*)) (coerce (send robot :angle-vector) cons))
      (coerce (nth (nth 2 *key-pose-id*) *limb-av-pose-list*) cons)
      )
    )

   (id-max 14) ;; タスク変数（位置姿勢6）より大きくする
   (recursive-order 5)

   (x-min 0.0)
   (x-max 2.6)
   (x-hit (* x-max 0.65))

   (bspline
    (mapcar #'(lambda (k)
                (instance basic-spline :init
                          :id-max id-max :recursive-order recursive-order
                          :x-min x-min :x-max x-max))
            jlist)) ;; 各関節（*jaxon_red*なら8個）にそれぞれid-max(=9)個のbsplineを作る、bsplineには(#<basic-spline #X8f25708> #<basic-spline #Xe85b650> #<basic-spline #Xfbd6448> #<basic-spline #X11555750> #<basic-spline #X12afff10> #<basic-spline #X218ad928> #<basic-spline #X250322c8> #<basic-spline #Xf8f82d8>)のように関節数分のbsplineのインスタンスのリストが入る

   (initial-state
    (instantiate float-vector (* id-max (length bspline))))
   ;; (length bspline) = (length jlist)
   ;; *jaxon_red*の場合(length initial-state)=9*8=72

   (state-min-vector
    (apply #'concatenate
           (cons float-vector
                 (mapcar
                  #'(lambda (j) (make-list id-max
                                           :initial-element (if use-margin (+ (send j :min-angle) use-margin) (send j :min-angle))
                                           ))
                  jlist))))
   ;; (#<vectorclass #X1ff49c8 float-vector> (-17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812) (-180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0) (-180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0) (-180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0) (-125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481) (-180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0) (-90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002) (-81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0))
   ;;       ↓
   ;; #f(-17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0)

   (state-max-vector
    (apply #'concatenate
           (cons float-vector
                 (mapcar
                  #'(lambda (j) (make-list id-max
                                           :initial-element (if use-margin (- (send j :max-angle) use-margin) (send j :max-angle))
                                           ))
                  jlist))))

   (equality-matrix-for-start/end-pos
    (matrix-append ;; s-nodaさんが作った関数で、'(1 0) で行（下）方向に、'(0 1)で列（横）方向に、'(1 1)で斜め方向につなげる ex)(matrix-append (list (matrix (list 1 2 3) (list 4 5 6)) (matrix (list 7 8) (list 9 10))) '(1 1)) = #2f((1.0 2.0 3.0 0.0 0.0) (4.0 5.0 6.0 0.0 0.0) (0.0 0.0 0.0 7.0 8.0) (0.0 0.0 0.0 9.0 10.0))
     (map cons
          #'(lambda (bs st ed)
              (send bs :calc-gain-vector-coeff-matrix-from-via-x-list (list 0.0 x-max x-hit)))
          ;; ex) (setq via-x-list (list 0.5)) (send (nth 0 bspline) :calc-gain-vector-coeff-matrix-from-via-x-list via-x-list) = #2f((0.0 0.0 0.002604 0.197917 0.598958 0.197917 0.002604 0.0 0.0))
          ;; (length via-x-list)行id-max(=9)列
          bspline start-av end-av)
     '(1 1))) ;; 24(start-av+end-av+hit-av)*72(jlist*id-max)行列

   (equality-coeff-for-start/end-pos
    (concatenate float-vector
                 (flatten (map cons #'list start-av end-av hit-av)))) ;; #f(start-av end-av hit-av)

   (equality-matrix-for-start/end-vel ;; 初期・終端条件（速度ゼロ）
    (matrix-append
     (map cons
          #'(lambda (bs st ed)
              (make-matrix
               2 id-max ;;2 行id-max列
               (mapcar
                #'(lambda (x) (send bs :calc-delta-coeff-vector x :n 1))
                (list 0.0 x-max))))
          bspline start-av end-av)
     '(1 1)))

   (equality-coeff-for-start/end-vel
    (scale 0 (concatenate float-vector
                          (flatten (map cons #'list start-av end-av)))))

   (equality-matrix-for-start/end-acc ;; 初期・終端条件（加速度ゼロ）
    (matrix-append
     (map cons
          #'(lambda (bs st ed)
              (make-matrix
               2 id-max
               (mapcar
                #'(lambda (x) (send bs :calc-delta-coeff-vector x :n 2))
                (list 0.0 x-max))))
          bspline start-av end-av)
     '(1 1)))

   (equality-coeff-for-start/end-acc
    (scale 0 (concatenate float-vector
                          (flatten (map cons #'list start-av end-av)))))

   (minjerk-weight 1e-6)
   (eval-weight-matrix ;; 72*72
    ;; minimize integreted jerk squared
    (let* ((mat
            (matrix-append
             (mapcar
              #'(lambda (rate)
                  (matrix-append
                   (mapcar
                    #'(lambda (bs)
                        (make-matrix ;; 1行id-max(=9)列の行列を作る
                         1 id-max
                         (list
                          (send bs :calc-delta-coeff-vector ;; :calc-delta-coeff-vector (x &key (n 1)) bsplineのn階微分したときのxにおける値
                                (+ x-min (* rate (- x-max x-min))) ;; x-minからx-maxの間で離散化
                                :n 3)
                          )))
                    bspline) ;; 1行id-max(=9)列の行列の関節数(=8)個のlist
                   '(1 1)))
              '(0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0)) ;; 8行72列の行列の離散化した分(=11)のリスト
             '(1 0)))) ;; 88行72列の行列（上のリストを縦につなげる）
      (m* (transpose mat) mat)) ;; M^T*M jerkの二乗
    )

   ;; (eigen-weight 1) ;; for compile
   (maxvel-weight 1)
   (eval-weight-matrix2 (progn
    ;; maxmize joint velocity at a time point x-hit
    (let* ((mat
            (matrix-append
             (mapcar
              #'(lambda (bs)
                  (make-matrix ;; 1行id-max(=9)列の行列を作る
                   1 id-max
                   (list
                    (send bs :calc-delta-coeff-vector x-hit :n 1) ;; x-hitにおけるbsplineの1階微分（＝速度）の値
                    )))
              bspline) ;; 1行id-max(=9)列の行列の関節数(=8)個のlist
             '(1 1))))
      (scale-matrix -1 (m* (transpose mat) mat))) ;; 速度を二乗後、maximizeのため符号反転
    ))

   (cnt 30)
   (x-step ;; x-stepごとに結果を出力
    ;;0.002
    (/ (- x-max x-min) (* 1.0 cnt))
    )

   (x-buf x-min)
   (ret (solve-eiquadprog
         ;; solve-qpoases-qp
         ;; :debug? t
         :initial-state initial-state
         :eval-weight-matrix (setq *mat* (m+ (scale-matrix minjerk-weight eval-weight-matrix)
                                             ;; (scale-matrix eigen-weight
                                             (unit-matrix (send eval-weight-matrix :get-val 'dim0))
                                             ;; )
                                             (scale-matrix maxvel-weight eval-weight-matrix2)))
         :state-min-vector state-min-vector
         :state-max-vector state-max-vector
         :equality-vector
         (concatenate float-vector
                      equality-coeff-for-start/end-pos
                      equality-coeff-for-start/end-vel
                      equality-coeff-for-start/end-acc)
         :equality-matrix
         (matrix-append
          (list equality-matrix-for-start/end-pos
                equality-matrix-for-start/end-vel
                equality-matrix-for-start/end-acc)
          '(1 0))
         ))
   )
  (format t "(qp-motion-optimize :x-max ~a :x-hit ~a :maxvel-weight ~a :minjerk-weight ~a :x-step ~a :use-all-joint ~a :use-margin ~a)~%" x-max x-hit maxvel-weight minjerk-weight x-step use-all-joint use-margin)
  (setq *ret* (copy-object (if ret ret initial-state)))
  ;; (setq *bspline* bspline)

  (setq *bspline-av-list* '())
  (setq *bspline-tm-list* '())
  (setq *bspline-ec-list* '())

  (if (null ret) (setq ret initial-state))
  (format t "   --- ~A x ~A = ~A variables~%" id-max (length start-av) (length initial-state))
  (let* ((retl (list (cons :gain ret))) p dp ddp (id) tau) ;; (cons :gain ret) ==> (:gain . ret)
    (setq x-buf x-min)
    (while (<= x-buf x-max)
      (setq id 0)
      (mapcar
       #'(lambda (bs js)
           (list
            (send js :joint-angle
                  (send bs :calc x-buf (subseq ret id (+ id id-max))))
            (send js :put :p (send bs :calc x-buf (subseq ret id (+ id id-max))))
            (send js :put :dp (send bs :calc-delta x-buf (subseq ret id (+ id id-max)) :n 1))
            (send js :put :ddp (send bs :calc-delta x-buf (subseq ret id (setq id (+ id id-max))) :n 2))
            ))
       bspline jlist)
      (push (send-all jlist :get :ddp) ddp)
      (push (send-all jlist :get :dp) dp)
      (push (send-all jlist :get :p) p)
      (push (send robot :angle-vector) *bspline-av-list*)
      (push x-buf *bspline-tm-list*)
      (push (send (send robot *limb* :end-coords) :copy-worldcoords) *bspline-ec-list*)
      (send robot :calc-torque-from-vel-acc
            :jvv (map float-vector
                      #'(lambda (j) (deg2rad (or (send j :get :dp) 0)))
                      (cdr (send robot :links)))
            :jav (map float-vector
                      #'(lambda (j) (deg2rad (or (send j :get :ddp) 0)))
                      (cdr (send robot :links))))
      (push (send-all jlist :joint-torque) tau)
      (send robot :fix-leg-to-coords *lfoot* :lleg)
      (setq x-buf (+ x-buf x-step))
      (send *irtviewer* :draw-objects)
      (x::window-main-one)
      (unix:usleep (round (* 0.01 1000 1000))))
    (push (cons :p (reverse p)) retl)
    (push (cons :dp (reverse dp)) retl)
    (push (cons :ddp (reverse ddp)) retl)
    (push (cons :tau (reverse tau)) retl)
    (setq *bspline-av-list* (reverse *bspline-av-list*))
    (setq *bspline-tm-list* (reverse *bspline-tm-list*))
    (setq *bspline-ec-list* (reverse *bspline-ec-list*))
    ;; (format t "  [dif] |~A| = ~A~%"
    ;;         (map float-vector #'- end-av (send-all jlist :joint-angle))
    ;;         (norm (map float-vector #'- end-av (send-all jlist :joint-angle))))
    ;; retl
    nil
    )
  )


;; (dolist (arm (send *robot* *limb* :links)) (dolist (b (append (send *robot* :head :links) (send *robot* :torso :links))) (print (pqp-collision-distance arm b))))
