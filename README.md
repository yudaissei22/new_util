# new_util

* angle-vectorを、thk hand ⇨ msl hand用にするもの


* まずはじめに、いろんなグローバル変数を集める。
```
roseus ~~
```

0. 前処理
```
(load "package://msl_hand_controller/euslisp/jaxon_red-interface.l")
(load "package://auto_stabilizer/euslisp/auto-stabilizer-interface.l")
(jaxon_red-init)
(setq *robot* *jaxon_red*)
```

* 空のlistを作ってから、sequenceに値をいれる
```
(let* ((t-max *x-max-of-p-orig*) (step 0.05) (seq-num (round (+ 1 (/ t-max step)))))
  (setq *exp-jpos-deg1* (make-list seq-num))
  (setq *exp-tm-ms1* (make-list seq-num))
  (setq *exp-rc1* (make-list seq-num)) 
  (setq *exp-zmp-wc1* (make-list seq-num))
  (setq *exp-optional1* (make-list seq-num))
  (ast-make-sequence-in-advance
   *exp-jpos-deg1* *exp-tm-ms1* *exp-rc1* *exp-zmp-wc1* *exp-optional1*
   :step step :x-max t-max)
  )
```

```
send *ri* :stop-auto-balancer
```

```
send *ri* :stop-impedance :arms

```
* スイング
```
(experiment-angle-vector-sequence-full *exp-jpos-deg1* *exp-tm-ms1* *exp-rc1* *exp-zmp-wc1* *exp-optional1* :initial-time 30000 :final-time 5000 :log-fname "/tmp/angle-vector-sequence-full") 
```

* 実機では、まずこれで初期姿勢にしてから、STを入れると良い。
```
(experiment-angle-vector-sequence-full (list (car *exp-jpos-deg1*)) (list (car *exp-tm-ms1*)) (list (car *exp-rc1*)) (list (car *exp-zmp-wc1*)) (list (car *exp-optional1*)) :initial-time 30000 :final-time 0 :log-fname "/tmp/init")
```


# ZMPを抜いたversion
0. 空のリストを作り、中に値を入れていく。
```
(let* ((t-max *x-max-of-p-orig*) (step 0.05) (seq-num (round (+ 1 (/ t-max step)))))
  (setq *exp-jpos-deg1* (make-list seq-num))
  (setq *exp-tm-ms1* (make-list seq-num))
  (setq *exp-rc1* (make-list seq-num)) 
  (setq *exp-optional1* (make-list seq-num))
  (ast-make-sequence-in-advance-without-zmp
   *exp-jpos-deg1* *exp-tm-ms1* *exp-rc1* *exp-optional1*
   :step step :x-max t-max)
  )
```

# ZMPとRC(root-coords)を抜いたversionです。

```
(let* ((t-max *x-max-of-p-orig*) (step 0.05) (seq-num (round (+ 1 (/ t-max step)))))
  (setq *exp-jpos-deg1* (make-list seq-num))
  (setq *exp-tm-ms1* (make-list seq-num))
  (setq *exp-optional1* (make-list seq-num))
  (ast-make-sequence-in-advance-without-zmp-rc
   *exp-jpos-deg1* *exp-tm-ms1* *exp-optional1*
   :step step :x-max t-max)
  )
```

1. 値を

* スイング

```
(experiment-angle-vector-sequence-full-without-zmp *exp-jpos-deg1* *exp-tm-ms1* *exp-rc1* *exp-optional1* :initial-time 30000 :final-time 5000 :log-fname "/tmp/angle-vector-sequence-full") 
```

```
(experiment-angle-vector-sequence-full-without-zmp (list (car *exp-jpos-deg1*)) (list (car *exp-tm-ms1*)) (list (car *exp-rc1*)) (list (car *exp-optional1*)) :initial-time 30000 :final-time 0 :log-fname "/tmp/init")
```

# rcなし
```
(experiment-angle-vector-sequence-full-without-zmp-rc *exp-jpos-deg1* *exp-tm-ms1* *exp-optional1* :initial-time 30000 :final-time 5000 :log-fname "/tmp/angle-vector-sequence-full") 
```

```
(experiment-angle-vector-sequence-full-without-zmp-rc (list (car *exp-jpos-deg1*)) (list (car *exp-tm-ms1*)) (list (car *exp-optional1*)) :initial-time 30000 :final-time 0 :log-fname "/tmp/init")
```


# new version

```
(let* ((t-max *x-max-of-p-orig*) (step 0.01) (seq-num (round (+ 1 (/ t-max step)))))
  (setq *exp-jpos-deg1* (make-list seq-num))
  (setq *exp-tm-ms1* (make-list seq-num))
  (setq *exp-optional1* (make-list seq-num))
  (ast-make-sequence-in-advance-without-zmp-rc
   *exp-jpos-deg1* *exp-tm-ms1* *exp-optional1*
   :step step :x-max t-max)
  )
```

```
tm-new 5
```
で、時間のリストを長くする。

```
(experiment-angle-vector-sequence-full-without-zmp-rc *exp-jpos-deg1* tm-list *exp-optional1* :initial-time 20000 :final-time 5000 :log-fname "/tmp/angle-vector-sequence-full") 
```

```
(send *ri* :angle-vector (send *robot* :angle-vector (car *exp-jpos-deg1*)) 20000)
```