
(cl:in-package :asdf)

(defsystem "robot_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "defineGlobalPath" :depends-on ("_package_defineGlobalPath"))
    (:file "_package_defineGlobalPath" :depends-on ("_package"))
    (:file "getPositions" :depends-on ("_package_getPositions"))
    (:file "_package_getPositions" :depends-on ("_package"))
    (:file "addToMap" :depends-on ("_package_addToMap"))
    (:file "_package_addToMap" :depends-on ("_package"))
  ))