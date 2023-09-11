# stm32_board_practice

전자 방위 프로젝트


In order to make stm32f411e discovery board work in PlatformIO, 

this setup is necessary in platformio.ini file :



[env:disco_f411ve]

platform = ststm32

board = disco_f411ve

framework = stm32cube


Arduino framework is not supported in this board - https://docs.platformio.org/en/latest/boards/ststm32/disco_f411ve.html 

Threfore, this project is planned to make Arduino platform compitable like the other board discor_f407ve(stm32f407ve discovery board)

https://docs.platformio.org/en/latest/boards/ststm32/genericSTM32F407VET6.html
