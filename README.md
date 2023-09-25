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



Arduino library support list by PlatformIO - https://github.com/stm32duino/Arduino_Core_STM32




Found :

TODO 1 - need test

According to the datasheet stm32f411ce (https://www.st.com/en/microcontrollers-microprocessors/stm32f411ce.html),

The STM32F411xC/xE are fully software and feature compatible with the STM32F4 series (page 13),

and stm32f413 has the same schematic with stm32f411 series.

Since stem32f413 board supports Arduino library, it seems possible to provide compitable Arduino code

: Failed (Arduino library in stm32f411 disco board doesn't work)






SonarLint installation



under .vscode, settings.json file needs to be created and :

{
    "sonarlint.pathToCompileCommands": "${workspaceFolder}/compile_commands.json"
}


should be added.


and select PlatformIO (tool bar on the left) -> Advanced -> Compilation Database

then, compile_commands.json filed will be created outside. This file needs to be added in gitignore due to absolute paths.

Refer to : https://github.com/platformio/platformio-core/issues/4219 


Sonarcloud can be also linked to sonarqube to check the quality of the code in cloud.

Register in https://sonarcloud.io, and link to github
