
(cl:in-package :asdf)

(defsystem "xbox-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "land" :depends-on ("_package_land"))
    (:file "_package_land" :depends-on ("_package"))
    (:file "marine" :depends-on ("_package_marine"))
    (:file "_package_marine" :depends-on ("_package"))
    (:file "op" :depends-on ("_package_op"))
    (:file "_package_op" :depends-on ("_package"))
  ))