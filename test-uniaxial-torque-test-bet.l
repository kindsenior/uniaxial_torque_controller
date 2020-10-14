;; (unix:system "export ROBOT=ichijiku")

;; gc print
(defun __gc_print (l1 l2) (print '!!!!!!!!!!gc-called!!!!!!!!!!))
(setq system:*gc-hook* '__gc_print)

(defmacro do-until-key-w/o-gc
    (&rest forms)
  `(let* ((str (list *standard-input*)))
     (while (null (select-stream str 0.0000001)) ,@forms)
     (let ((strm (car (select-stream str 0.1))))
       (if strm (read-line strm nil nil)))))

(load "package://control_tools/play-hrpsysfile.l")

;; non-realtime config
(load "package://trans_ros_bridge/euslisp/eus-shm-interface.l")
(initialize-sharedmemory)

;; realtime config
(load "package://trans_ros_bridge/euslisp/eus-realtime-interface.l")
(let ((context-cycle-ms 1)) ;; 1 msec
;; (let ((context-cycle-ms 2)) ;; 2 msec
;; (let ((context-cycle-ms 4)) ;; 4 msec
;; (let ((context-cycle-ms 10)) ;; 10 msec
  (setq *context-cycle* (* 1000 context-cycle-ms))) ;; [usec]
(initialize-context-realtime 99 *context-cycle*) ;; priority = 99
(reset-stat-realtime)

(sys::gc)

(start-realtime)
(let* ((cntr 0)
       (initial-angle-rad (elt (read-angle-shm #f(0)) 0))
       (initial-angle (rad2deg initial-angle-rad))

       (print-cycle (* 1000 1000)) ;; 1 sec = 1000 msec = 1e6 usec
       (step (round (/ print-cycle *context-cycle*)))

       (angle-range #f(30 150))

       ;; (ref-angle 20) ;; deg
       (ref-angle 90) ;; deg
       (prev-ref-angle initial-angle)
       ;; (ref-time 10.0) ;; sec
       (ref-time 5.0) ;; sec
       (ref-count (round (/ (* ref-time 1e6) *context-cycle*)))
       (remain-count ref-count)

       (com-angle-vec  #f(0))
       (com-torque-vec #f(0))
       (pgain-vec  #f(0))
       (dgain-vec #f(0))

       (cur-force-vec #f(0 0 0 0 0 0))

       ;; mass property (mass [kg], pos [m])
       ;; 1.25 kg
       (mass-vec    #f(0.432 0.05 1.25))
       (cog-pos-vec #f(0.25  0.5  0.5))
       ;; no weight
       ;; (mass-vec    #f(0.432 0.05))
       ;; (cog-pos-vec #f(0.25  0.5 ))
       (gravity-acc 9.8) ;; kg m/s^2

       (cur-mode :pos)
       )
  (format t "context cycle: ~a, print cycle: ~a[useq] =~a[msec], step: ~a~%" *context-cycle* print-cycle (/ print-cycle 1000) step)
  (format t "initial: ~a [rad], ~a [deg]~%" initial-angle-rad initial-angle)
  (format t "ref : ~a [deg], ~a [s] (~a)~%" ref-angle ref-time ref-count)

  (do-until-key-w/o-gc
   (setq cur-angle-rad (elt (read-angle-shm #f(0)) 0)
         cur-angle (rad2deg cur-angle-rad)

         cur-force-x (elt (read-force-sensor 0 #f(0 0 0 0 0 0)) 0)
         ;; cur-force (read-force-sensor)
         )

   ;; check
   (if (or (< cur-angle (elt angle-range 0)) (> cur-angle (elt angle-range 1)))
       (progn (warning-message 1 "[Error] Angle range over (~s)!!!~%" angle-range) (return)))

   ;; command
   (cond
    ((eq cur-mode :pos) ;; pos mode
     (cond
      ((> remain-count 0)
       (setq com-angle (+ prev-ref-angle (* (- 1 (/ remain-count (float ref-count))) (- ref-angle prev-ref-angle)))
             com-angle-rad (deg2rad com-angle)
             com-torque 0
             pgain 1 dgain 1
             )
       )
      (t (setq cur-mode :fftq
               pgain 0 dgain 0))
               ;; pgain 0 dgain 1))
               ;; pgain 0 dgain 0.1))
      )
     )
    ((eq cur-mode :fftq) ;; fftq mode
     (setq com-angle cur-angle
           com-angle-rad (deg2rad com-angle)
           gravity-torque (* (v. cog-pos-vec mass-vec) (sin cur-angle-rad) gravity-acc)
           com-torque gravity-torque
           )
     )
    (t)
    )

   ;; set command vectors
   (aset com-angle-vec  0 com-angle-rad)
   (aset com-torque-vec 0 com-torque)
   (aset pgain-vec 0 pgain)
   (aset dgain-vec 0 dgain)
   ;; write command
   (write-ref-angle-shm  com-angle-vec)
   (write-ref-torque-shm com-torque-vec)
   (write-pgain-shm pgain-vec)
   (write-dgain-shm dgain-vec)


   ;; print
   (cond
    ((= (mod cntr step) (- step 1))
     (format t "mode: ~a~%" cur-mode)
     (format t "jitter: ~a [ms], max: ~a [ms]~%"
             (/ (get-stat-norm-realtime) 1000)
             (/ (get-stat-max-realtime) 1000))
     ;; angle
     (format t "angle~%")
     (format t " cur: ~a [rad] ~a [deg]~%" cur-angle-rad cur-angle)
     (format t " com: ~a [rad] ~a [deg]~%" com-angle-rad com-angle)
     ;; torque
     (format t "torque~%")
     (format t " cur: ~a [Nm]~%" cur-force-x)
     (format t " com: ~a [Nm]~%" com-torque)

     (format t "ramain count: ~a~%" remain-count)
     (format t "~%")
     (reset-stat-realtime)
     ))

   ;; (decf remain-count)
   ;; (incf cntr)
   (setq remain-count (1- remain-count)
         cntr (1+ cntr))
   ;; (when (> cntr 500)
   ;; (when (> cntr 1000)
   (cond
    ((> cntr 10000)
     ;; (reset-stat-realtime)
     (setq cntr 0)
     ))

   ;; (when (= (mod cntr (* step 10)) (- (* step 10) 1))
   ;;   ;; (sys:thread-no-wait #'(lambda () (sys::gc)))
   ;;   ;; (sys::gc)
   ;;   )
   ;; (sys::gc) ;; 25ms ~ / depend on size of garbage
   ;; (write-ref-angle-shm (instantiate float-vector 1))
   ;; (dotimes (i 10) (solve-ik)) ;; single-ik 2ms / 2.6GHz
   (wait-next-realtime)
   )

  (write-ref-torque-shm #f(0))
  (write-dgain-shm #f(0))

  (sys::gc)
  )

;; (unix:system "sudo chown k-kojima:k-kojima -R ~/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/ichijiku/*")
;; (unix:system "cd ~/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/ichijiku/$(ls -t ~/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/ichijiku/ | head -n 1); ../../convert-rosbag-to-csv.sh")
