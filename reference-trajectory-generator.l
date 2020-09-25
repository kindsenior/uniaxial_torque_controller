(load "package://control_tools/plot-tools.l")

(defclass trajectory-generator
  :super object
  :slots (dt
          time-index
          term term-range amplitude amplitude-range
          ret-value init-value
          )
  )

(defmethod trajectory-generator
  (:init
   (&rest args)
   (setq dt 0.001
         term-range #f(0 2)
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
   (setq term (send self :random-param term-range)
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
   (+ init-value amplitude)
   )
  )

(defclass linear-trajectory-generator
  :super trajectory-generator
  :slots ()
  )
(defmethod linear-trajectory-generator
  (:process-cycle
   ()
   (setq ret-value (+ init-value (* amplitude (/ (send self :time) term))))
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
   ()
   (send-super :init)
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
   ()
   (setq current-value 0
         trajectory-generator-choice-list
         (list (instance constant-trajectory-generator :init)
               (instance step-trajectory-generator :init)
               (instance linear-trajectory-generator :init)
               (instance trig-function-trajectory-generator :init)
               )
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
          (if target-generator (warn "time: ~a~%" (send target-generator :time)))
          (send self :set-random-function)
          (send self :reset-generator current-value)
          (warn "set func to ~a (value: ~a, term: ~a)~%" target-generator current-value (send target-generator :get-val 'term))
          ))
   )
  (:process-cycle
   ()
   (setq current-value (send target-generator :process-cycle))
   current-value
   )
  )
