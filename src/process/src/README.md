# Как запускать модель и как управлять
1) Запустить gold_autumn.xml в process/src/algo_xcorr </br>
2) Запустить топики с лидара rosrun algo_xcorr_cmp determine_lidar_pose </br>
3) Запустить ноду управления режимами и подруливания rosrun motion_controller motion_controller MC_out distance 0.4 1 , distanse - отрицательное число от -1 и меньше </br>
4) Запустить джойстик: rqt --standalone rqr_robot_steering(и указать топик /joystick в окне) </br>
5) Запустить gzclient: rosrun gazebo_ros gzclient </br>
6) Для управления режимами запустить command_interpreter: rosrun command_interpreter command_interpreter и читать в его README как менять режимы. </br>
alias для отправки сообщений: "rostopic pub -1 /CI_input std_msgs/String" </br>
Использование send_msg "set mode 0" </br>
Текущее желанное отклонение от борозды задается через аргументы motion_controller'a.
