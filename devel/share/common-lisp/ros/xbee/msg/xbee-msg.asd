
(cl:in-package :asdf)

(defsystem "xbee-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "cmd_ctrl" :depends-on ("_package_cmd_ctrl"))
    (:file "_package_cmd_ctrl" :depends-on ("_package"))
  ))