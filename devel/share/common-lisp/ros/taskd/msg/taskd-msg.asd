
(cl:in-package :asdf)

(defsystem "taskd-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "task_status" :depends-on ("_package_task_status"))
    (:file "_package_task_status" :depends-on ("_package"))
  ))