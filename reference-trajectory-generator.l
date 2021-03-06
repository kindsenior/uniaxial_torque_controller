(load "package://control_tools/plot-tools.l")

(defclass trajectory-generator
  :super object
  :slots (dt
          time-index
          sgn term term-range amplitude amplitude-range
          ret-value init-value
          )
  )

(defmethod trajectory-generator
  (:init
   (&rest args &key (dt 0.001) &allow-other-keys)
   (send self :set-val 'dt dt)
   (setq term-range #f(0.5 2)
         amplitude-range #(0 1)
         sgn 1 term 1 amplitude 1
         init-value 0
         )
   (send self :reset-time)
   )
  (:trajectory-type
   ()
   (reverse (subseq (reverse (format nil "~s" (send (class self) :name))) 21)) ;; remove "-trajectory-generator"(= 21 chars) from class name
   )
  (:format-param () (format nil "~a~2,2fsec" (send self :trajectory-type) (send self :get-val 'term)))
  (:reset-generator
   (current-value)
   ;; (warn ":reset-generator ~a~%" current-value)
   (setq init-value current-value)
   (send self :reset-time) ;; reset time before setting ramdom parameters (for using time for setting parameters)
   (send self :set-random-params)
   )
  (:random-param
   (value-range)
   (+ (aref term-range 0) (* (random 1.0) (- (aref term-range 1) (aref term-range 0))))
   )
  (:set-random-params
   ()
   ;; (warn ":set-random-params~%")
   (setq sgn (if (> init-value 0) -1 1) ;; -1 (init-value > 0) : 1 (init-value <= 0)
         term (send self :random-param term-range)
         amplitude (send self :random-param amplitude-range))
   )
  (:term-finished () (> (send self :time) term))
  (:reset-time () (setq time-index 1))
  (:time () (* dt time-index))
  (:increment-time () (setq time-index (1+ time-index)))
  )

(defclass constant-trajectory-generator
  :super trajectory-generator
  :slots ()
  )
(defmethod constant-trajectory-generator
  (:format-param () (format nil "~a" (send-super :format-param)))
  (:process-cycle
   ()
   (send-super :increment-time)
   init-value
   )
  )

(defclass step-trajectory-generator
  :super trajectory-generator
  :slots ()
  )
(defmethod step-trajectory-generator
  (:format-param () (format nil "~a,~2,2fA" (send-super :format-param) (* sgn amplitude)))
  (:process-cycle
   ()
   (send-super :increment-time)
   (+ init-value (* sgn amplitude))
   )
  )

(defclass linear-trajectory-generator
  :super trajectory-generator
  :slots ()
  )
(defmethod linear-trajectory-generator
  (:format-param () (format nil "~a,~2,2f" (send-super :format-param) (* sgn amplitude)))
  (:process-cycle
   ()
   (setq ret-value (+ init-value (* sgn amplitude (/ (send self :time) term))))
   (send-super :increment-time)
   ret-value
   )
  )

(defclass trig-function-trajectory-generator
  :super trajectory-generator
  :slots (phase-offset
          frequency frequency-range
          )
  )
(defmethod trig-function-trajectory-generator
  (:init
   (&rest args &key &allow-other-keys)
   (send-super* :init args)
   (setq frequency-range #f(0.5 2))
   )
  (:set-random-params
   ()
   (send-super :set-random-params)
   (setq phase-offset (random (* 2 pi))
         frequency (send-super :random-param frequency-range))
   (if (> (* init-value (send self :eval-function (+ (send self :time) term))) 0) (setq sgn (* sgn -1))) ;; flip sgn depending on the final value f(term)
   )
  (:format-param () (format nil "~a,~2,2fA,~2,2frad,~2,2fHz" (send-super :format-param) (* sgn amplitude) phase-offset frequency))
  (:eval-function (time) (* sgn amplitude (- (sin (+ phase-offset (* 2 pi frequency time))) (sin phase-offset)))) ;; f(0) = 0 (without init-value)
  (:process-cycle
   ()
   (setq ret-value (+ init-value (send self :eval-function (send self :time))))
   (send-super :increment-time)
   ret-value
   )
  )

(defclass reference-trajectory-generator
  :super object
  :slots (trajectory-generator-choice-list
          target-generator
          current-value
          reference-trajectory-list time-list param-format-list
          total-time
          file-name
          )
  )
(defmethod reference-trajectory-generator
  (:init
   (&rest args &key &allow-other-keys)
   (setq current-value 0
         trajectory-generator-choice-list
         (list (instance* constant-trajectory-generator      :init args)
               (instance* step-trajectory-generator          :init args)
               (instance* linear-trajectory-generator        :init args)
               (instance* trig-function-trajectory-generator :init args)
               )
         )
   (send self :setup)
   )
  (:setup
   ()
   (send self :reset-time)
   (setq reference-trajectory-list nil
         time-list nil
         param-format-list nil
         total-time 0
         file-name "reference-trajectory"
         )
   )
  (:set-random-function
   ()
   (setq target-generator (elt trajectory-generator-choice-list (random (length trajectory-generator-choice-list))))
   )
  (:reset-generator
   (current-value)
   (send target-generator :reset-generator current-value)
   )
  (:check-and-reset-generator
   ()
   ;; (warn ":check-and-reset-generator ~a~%" current-value)
   (cond ((or (not target-generator) (send target-generator :term-finished))
          (send self :set-random-function)
          (send self :reset-generator current-value)
          (warn "time: ~5a, func: ~15a (init value: ~10a, term: ~10a, amp: ~10a, sgn: ~2a)~%"
                (send self :time) (send target-generator :trajectory-type) current-value
                (send target-generator :get-val 'term) (send target-generator :get-val 'amplitude) (send target-generator :get-val 'sgn)
                )
          (push-back (send target-generator :format-param) param-format-list)
          ))
   )
  (:reset-time () (setq time-index 0))
  (:time ()   (* (send target-generator :get-val 'dt) time-index))
  (:increment-time () (setq time-index (1+ time-index)))
  (:process-cycle
   ()
   (send self :increment-time)
   (push-back (send self :time) time-list) ;; generate time list
   (push-back (setq current-value (send target-generator :process-cycle)) reference-trajectory-list) ;; set current-value and generate trajectory list
   )
  (:generate-reference-trajectory
   (&key (total-time 5) &allow-other-keys)
   (send self :set-val 'total-time total-time)
   (do-until-key
    (send self :check-and-reset-generator)
    (if (=  (send self :time) total-time) (return))
    (send *tg* :process-cycle)
    )
   )
  (:format-param () (format nil "~asec_~a" total-time (reduce #'(lambda (ret param-format) (format nil "~a_~a" ret param-format)) param-format-list)))
  (:print-param
   ()
   (format t "[trajectory generator]~%")
   (format t " file: ~a~%" (send self :file-name))
   (format t " total time: ~a sec~%" total-time)
   (mapc #'(lambda (param-format) (format t "  ~a~%" param-format)) param-format-list)
   )
  (:file-name
   (&optional fname)
   (if fname (send self :set-val 'file-name fname))
   (send self :get-val 'file-name)
   )
  (:plot-graph
   ()
   (graph-view (list (send self :get-val 'reference-trajectory-list))
               (send self :get-val 'time-list))
   )
  )

;; Usage
;; (generate-reference-trajectory :fname "test-reference-trajectory" :total-time 5)
(defun generate-reference-trajectory (&rest args &key (fname nil) &allow-other-keys)
  (setq *tg* (instance reference-trajectory-generator :init))
  (send* *tg* :generate-reference-trajectory args)
  (send *tg* :plot-graph)

  ;; save to .l
  (let ( (fname (send *tg* :file-name (format nil "~asec_~a" (send *tg* :get-val 'total-time) (if fname (send *tg* :file-name fname) (send *tg* :file-name))))) )
    (warn "save the generate reference trajectory to ~a~%" (send *tg* :file-name))
    (dump-loadable-structure (format nil "~a.l" fname *tg*))
    (unix:system (format nil "sudo chown k-kojima:k-kojima ~a~%" fname))
    (unix:system (format nil "sudo chmod 664 ~a~%" fname))
    )
  nil
  )
