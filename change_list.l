(defun change (x)
  ;; thkハンドのdofを消して、mslハンドのdofを追加した
  (load (concatenate string "forehand/jerk/" (princ-to-string x) "_forehand_jerk.l"))

  (setq *new-exp-jpos-deg1*
	(mapcar
	 #'(lambda (av)
	     (concatenate float-vector (subseq av 0 33) #f(-50 0 70 0 100 14.3239 0 0 0 0 0 0))
	     )
	 *exp-jpos-deg1*)
	)
  
  (setq fname (concatenate string "new_" (princ-to-string x) "_forehand_jerk.l"))
  (setq fpath (concatenate string "/home/yuda/catkin_ws/ast_ws/src/new_util/forehand/jerk/"))
			   
  (dump-loadable-structure (concatenate string fpath fname)
			    *new-exp-jpos-deg1*
			    *exp-tm-ms1*
			    *exp-rc1*
			    *exp-zmp-wc1*
			    *exp-optional1*
			    *exp-jpos-rad1*
			    *exp-tm-s1*
			    *exp-pos1*
			    *exp-rpy1*
			    *exp-zmp-rc1*
			    *exp-wc1*
			    *exp-cog-wc1*
			    )
   )