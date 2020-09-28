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
   (setq term-range #f(0 2)
         amplitude-range #(0 12)
         )
   (send self :reset-time)
   )
  (:reset-generator
   (current-value)
   ;; (warn ":reset-generator ~a~%" current-value)
   (setq init-value current-value)
   (send self :set-random-params)
   (send self :reset-time)
   )
  (:random-param
   (value-range)
   (+ (aref term-range 0) (* (random 1.0) (- (aref term-range 1) (aref term-range 0))))
   )
  (:set-random-params
   ()
   ;; (warn ":set-random-params~%")
   (setq sgn (expt -1 (random 2)) ;; -1 or 1
         term (send self :random-param term-range)
         amplitude (send self :random-param amplitude-range))
   )
  (:term-finished
   ()
   (> (send self :time) term)
   )
  (:reset-time
   ()
   (setq time-index 1)
   )
  (:time
   ()
   (/ (* dt time-index) term)
   )
  (:increment-time
   ()
   (setq time-index (1+ time-index))
   )
  )

(defclass constant-trajectory-generator
  :super trajectory-generator
  :slots ()
  )
(defmethod constant-trajectory-generator
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
          offset-value)
  )
(defmethod trig-function-trajectory-generator
  (:init
   (&rest args &key &allow-other-keys)
   (send-super* :init args)
   (setq frequency-range #f(0.5 2))
   (send self :set-random-params)
   )
  (:set-random-params
   ()
   (setq phase-offset (random (* 2 pi))
         frequency (send-super :random-param frequency-range))
   (send-super :set-random-params)
   (setq offset-value (* -1 term (sin phase-offset)))
   )
  (:process-cycle
   ()
   (setq ret-value (+ init-value offset-value (* term (sin (+ phase-offset (* 2 pi frequency (send self :time)))))))
   (send-super :increment-time)
   ret-value
   )
  )

(defclass reference-trajectory-generator
  :super object
  :slots (trajectory-generator-choice-list
          target-generator
          current-value)
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
   (send self :reset-time)
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
          (if target-generator (warn "time: ~a~%" (send target-generator :time)))
          (send self :set-random-function)
          (send self :reset-generator current-value)
          (warn "set func to ~a (value: ~a, term: ~a)~%" target-generator current-value (send target-generator :get-val 'term))
          ))
   )
  (:reset-time () (setq time-index 0))
  (:time ()   (* (send target-generator :get-val 'dt) time-index))
  (:increment-time () (setq time-index (1+ time-index)))
  (:process-cycle
   ()
   (send self :increment-time)
   (setq current-value (send target-generator :process-cycle))
   current-value
   )
  )
