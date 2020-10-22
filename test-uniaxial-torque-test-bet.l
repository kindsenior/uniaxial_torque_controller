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
(load "reference-trajectory-generator.l")

;; non-realtime config
(load "package://trans_ros_bridge/euslisp/eus-shm-interface.l")
(initialize-sharedmemory)

;; realtime config
(load "package://trans_ros_bridge/euslisp/eus-realtime-interface.l")

(defclass uniaxial-torque-test-bet-controller
  :super object
  :slots (
          context-cycle-us
          print-cycle step-count
          counter

          initial-angle-rad initial-angle
          ref-angle prev-ref-angle
          ref-time ref-count remain-count
          angle-range

          com-angle-vec com-torque-vec pgain-vec dgain-vec cur-force-vec

          mass-vec cog-pos-vec gravity-acc

          cur-mode

          reference-trajectory-generator reference-torque-queue
          )
  )
(defmethod uniaxial-torque-test-bet-controller
  (:init
   (&rest args &key (context-cycle-ms 1) &allow-other-keys)
   (setq context-cycle-us (* 1000 1)) ;; 1 usec
   (initialize-context-realtime 99 context-cycle-us) ;; priority = 99

   (setq counter 0
         print-cycle (* 1000 1000) ;; 1 sec = 1000 msec = 1e6 usec
         step-count (round (/ print-cycle context-cycle-us))

         angle-range #f(30 150)

         ;; command and measured values
         com-angle-vec  #f(0)
         com-torque-vec #f(0)
         pgain-vec  #f(0)
         dgain-vec #f(0)
         cur-force-vec #f(0 0 0 0 0 0)

         ;; mass property (mass [kg], pos [m])
         ;; 1.25 kg
         ;; mass-vec    #f(0.432 0.05 1.25)
         ;; cog-pos-vec #f(0.25  0.5  0.5)
         ;; no weight
         mass-vec    #f(0.432 0.05)
         cog-pos-vec #f(0.25  0.5 )
         gravity-acc 9.8 ;; kg m/s^2

         cur-mode :pos
         )
   (format t "context cycle: ~a, print cycle: ~a[useq] =~a[msec], step-count: ~a~%" context-cycle-us print-cycle (/ print-cycle 1000) step-count)

   (send self :reset)
   )

  (:reset
   (&rest args &key (tq-gen (if (boundp '*tg*) *tg* nil)) &allow-other-keys)
   (if tq-gen (send tq-gen :print-param))
   (send self :set-val 'reference-trajectory-generator tq-gen)
   (send self :set-val 'reference-torque-queue (if tq-gen (send tq-gen :get-val 'reference-trajectory-list) nil))
   (setq cur-mode :pos)
   )

  (:start-test
   (&rest args &key (return-if-ref-nil nil) &allow-other-keys)
   ;; initial procedure
   (send* self :reset args)
   (reset-stat-realtime)
   (sys::gc)
   (start-realtime)

   (setq ;; read initial angle
    initial-angle-rad (elt (read-angle-shm #f(0)) 0)
    initial-angle (rad2deg initial-angle-rad)
    prev-ref-angle initial-angle

    ;; set reference (pose at starting test)
    ref-angle 90 ;; deg
    ref-time 5.0 ;; sec
    ref-count (round (/ (* ref-time 1e6) context-cycle-us))
    remain-count ref-count
    )
   (format t "initial: ~a [rad], ~a [deg]~%" initial-angle-rad initial-angle)
   (format t "ref : ~a [deg], ~a [s] (~a)~%" ref-angle ref-time ref-count)

   (do-until-key-w/o-gc
    (setq cur-angle-rad (elt (read-angle-shm #f(0)) 0)
          cur-angle (rad2deg cur-angle-rad)

          cur-force-x (elt (read-force-sensor 0 #f(0 0 0 0 0 0)) 0)
          )

    ;; check
    (if (and (or (< cur-angle (elt angle-range 0)) (> cur-angle (elt angle-range 1)))
             (not (equal cur-mode :pos)))
        (progn (warning-message 1 "[Error] Angle range over (~s)!!!~%" angle-range) (return)))
    (if (and return-if-ref-nil (not reference-torque-queue) (equal cur-mode :fftq))
        (progn (warning-message 3 "[Info] Torque commands finished!!!~%") (return)))

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
      (let ( (ext-torque (if reference-torque-queue (pop reference-torque-queue) 0)) )
        (setq com-angle cur-angle
              com-angle-rad (deg2rad com-angle)
              gravity-torque (* (v. cog-pos-vec mass-vec) (sin cur-angle-rad) gravity-acc)
              com-torque (+ gravity-torque ext-torque)
              )
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
     ((= (mod counter step-count) (- step-count 1))
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

    (setq remain-count (1- remain-count)
          counter (1+ counter))
    (cond
     ((> counter 10000)
      ;; (reset-stat-realtime)
      (setq counter 0)
      ))

    (wait-next-realtime)
    )

   (write-ref-torque-shm #f(0))
   (write-pgain-shm #f(0))
   (write-dgain-shm #f(0))

   (sys::gc)
   )

  (:save-log
   (&rest args &key (fname nil) &allow-other-keys)
   (let* ((tq-gen (send self :get-val 'reference-trajectory-generator))
          (if tq-gen (send tq-gen :print-param))
          (ref-trajectory-fname (if tq-gen (send tq-gen :file-name) ""))
          (saved-shm-log-name (save-shm-log :fname (format nil "~a~a" fname (if tq-gen (format nil "_~a" ref-trajectory-fname) ""))))
          (log-path (reg-replace "[^/]*.log$" ""  saved-shm-log-name))
          )
     (unix:system (format nil "chown k-kojima:k-kojima -R ~a" log-path))
     (if tq-gen (unix:system (format nil "cp ~a ~a" ref-trajectory-fname log-path)))
     )
   )
  )

(defun test-uniaxial-torque-test-bet (&rest args &key (fname nil) &allow-other-keys)
  (send* *uc* :start-test args)
  (if fname (send* *uc* :save-log args))
  )

(defun setup-uniaxial-torque-test-bet ()
  (setq *uc* (instance uniaxial-torque-test-bet-controller :init))
  )
(setup-uniaxial-torque-test-bet)

;; (unix:system "sudo chown k-kojima:k-kojima -R ~/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/ichijiku/*")
;; (unix:system "cd ~/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/ichijiku/$(ls -t ~/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/ichijiku/ | head -n 1); ../../convert-rosbag-to-csv.sh")
