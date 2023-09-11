# stm32_board_practice

전자 방위 프로젝트


In order to make stm32f411e discovery board work in PlatformIO, 
this setup is necessary in platformio.ini file :

[env:disco_f411ve]
platform = ststm32
board = disco_f411ve
framework = stm32cube
