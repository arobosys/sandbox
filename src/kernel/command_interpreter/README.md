# Интерпретатор команд
## Формат комманд
_start scenario|process id_ - запуск сценария/процесса по его id\
_stop scenario|process id_ - запуск сценария/процесса по его id\
_restart process id_ - перезапуск процесса по его id\
get state - получить текущее состояние системы, в будущем больше опций
## Запуск и работа
Запускается как обычная нода через rosrun command_interpreter command_interpreter\
По умолчанию слушает топик _/CI_input_, но первым аргументом строки можно передать
название топика для входящих сообщений.\
Дальнейшие взаимодействия осуществляются с помощь команды _rostopic pub -1 /topic_name std_msgs/String "your message"_\
ЧТОБЫ ПОМЕНЯТЬ РЕЖИМ УПРАВЛЕНИЯ НУЖНО НАПИСАТЬ "set mode 0" или "set mode 1". 0 - manual, 1 - auto-manual.
